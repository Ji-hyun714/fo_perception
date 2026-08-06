#!/usr/bin/env python3.8
import rclpy
from rclpy.node import Node
import numpy as np
import torch
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Quaternion

# KalmanFilter 클래스 정의
class KalmanFilter:
    def __init__(self, init_state, process_variance=1e-2, measurement_variance=1e-1):
        self.state = np.array(init_state, dtype=np.float64)
        self.uncertainty = np.eye(len(self.state)) * 0.1
        self.process_variance = np.eye(len(self.state)) * process_variance
        self.measurement_variance = measurement_variance

    def predict(self):
        self.uncertainty += self.process_variance

    def update(self, measurement):
        kalman_gain = self.uncertainty / (self.uncertainty + self.measurement_variance)
        measurement_residual = measurement - self.state
        self.state += kalman_gain @ measurement_residual
        self.uncertainty = (np.eye(len(self.state)) - kalman_gain) @ self.uncertainty

    def get_state(self):
        return self.state


# 라벨 매핑 함수 정의
def map_label(label):
    label_mapping = {
        1: 1,  # car
        2: 2,  # truck -> Bus / Truck
        3: 2,  # construction_vehicle -> Bus / Truck
        4: 2,  # bus -> Bus / Truck
        5: 2,  # trailer -> Bus / Truck
        6: 0,  # barrier -> unknown
        7: 3,  # motorcycle -> Motorcycle
        8: 3,  # bicycle -> Motorcycle
        9: 4,  # pedestrian -> Pedestrian
        10: 0  # traffic_cone -> unknown
    }
    return label_mapping.get(label, 0)  # 해당 라벨이 없으면 0으로 처리


class LidarTrackingNode(Node):
    def __init__(self):
        super().__init__('lidar_tracking_node')

        self.before_dict = {i: {"coords": torch.full((3,), float('nan')), "label": 0, "is_tracked": False} for i in range(1, 33)}
        
        # 퍼블리셔 설정 (Detection3DArray로 퍼블리시)
        self.pub_lidar_11_TD = self.create_publisher(Detection3DArray, '/lidar_11_201/TD', 20)
        self.pub_lidar_12_TD = self.create_publisher(Detection3DArray, '/lidar_12_201/TD', 20)
        self.pub_lidar_13_TD = self.create_publisher(Detection3DArray, '/lidar_13_201/TD', 20)

        # Subscribers for object detection results
        self.create_subscription(Detection3DArray, '/lidar_11_201/TFdetect', self.lidar_callback_11, 10)
        self.create_subscription(Detection3DArray, '/lidar_12_201/TFdetect', self.lidar_callback_12, 10)
        self.create_subscription(Detection3DArray, '/lidar_13_201/TFdetect', self.lidar_callback_13, 10)

    def lidar_callback_11(self, data):
        self.process_lidar_data(data, self.pub_lidar_11_TD)

    def lidar_callback_12(self, data):
        self.process_lidar_data(data, self.pub_lidar_12_TD)

    def lidar_callback_13(self, data):
        self.process_lidar_data(data, self.pub_lidar_13_TD)

    def process_lidar_data(self, data, pub):
        pred_detections = data.detections
        matched_ids = set()

        # 모든 객체의 is_tracked를 False로 초기화
        for obj_id in self.before_dict:
            self.before_dict[obj_id]["is_tracked"] = False

        # 이전 프레임의 객체와 현재 프레임의 객체를 비교하여 매칭된 객체는 is_tracked = True로 설정하고 기존 ID 유지
        for det_num, det in enumerate(pred_detections):
            position = det.bbox.center.position
            orientation = det.bbox.center.orientation
            size = det.bbox.size
            score = det.results[0].hypothesis.score
            original_label = det.results[0].hypothesis.class_id

            matched_id = None
            for prev_id, prev_obj in self.before_dict.items():
                if self.is_same_object(prev_obj, torch.tensor([position.x, position.y, position.z])):
                    matched_id = prev_id
                    matched_ids.add(matched_id)
                    break

            # 매칭된 객체는 is_tracked = True로 상태 업데이트
            if matched_id is not None:
                self.before_dict[matched_id] = {
                    "coords": torch.tensor([position.x, position.y, position.z]),
                    "label": original_label,  # 출력에서는 원래 라벨 사용
                    "is_tracked": True,
                    "orientation": torch.tensor([orientation.x, orientation.y, orientation.z, orientation.w]),
                    "size": torch.tensor([size.x, size.y, size.z]),
                    "score": score
                }

        # 현재 프레임에서 매칭되지 않은 객체들을 모두 NaN으로 설정
        for obj_id in self.before_dict:
            if obj_id not in matched_ids:
                self.before_dict[obj_id] = {"coords": torch.full((3,), float('nan')), "label": 0, "is_tracked": False}

        # 매칭되지 않은 새로운 객체는 NaN 슬롯을 채워넣음
        for det_num, det in enumerate(pred_detections):
            if det_num + 1 not in matched_ids:
                position = det.bbox.center.position
                orientation = det.bbox.center.orientation
                size = det.bbox.size
                score = det.results[0].hypothesis.score
                original_label = det.results[0].hypothesis.class_id

                for i in range(1, 33):
                    if torch.isnan(self.before_dict[i]["coords"]).all():  # NaN 슬롯 찾기
                        self.before_dict[i] = {
                            "coords": torch.tensor([position.x, position.y, position.z]),
                            "label": original_label,  # 출력에서는 원래 라벨 사용
                            "is_tracked": False,
                            "orientation": torch.tensor([orientation.x, orientation.y, orientation.z, orientation.w]),
                            "size": torch.tensor([size.x, size.y, size.z]),
                            "score": score
                        }
                        break

        # 매 프레임마다 32개의 트래킹 객체 정보 출력 (원래 라벨 사용)
        print("\n=== Tracking Results for Current Frame ===")
        for obj_id, obj_data in self.before_dict.items():
            coords = obj_data['coords']
            label = obj_data['label']
            is_tracked = obj_data['is_tracked']
            # if label == '9':
            #     print(f"ID: {obj_id}, Coordinates: {coords}, Label: {label}, Tracked: {is_tracked}")
            print(f"ID: {obj_id}, Coordinates: {coords}, Label: {label}, Tracked: {is_tracked}")

        # 결과 퍼블리시 (새로 매핑된 라벨 사용)
        detection_array = Detection3DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        detection_array.header.frame_id = 'velodyne'  # 프레임 설정
        
        for obj_id, obj_data in self.before_dict.items():
            # 모든 객체가 32개가 퍼블리시 되도록 처리
            detection_array.detections.append(self.create_detection3d(
                obj_id,
                obj_data['coords'] if not torch.isnan(obj_data['coords']).all() else torch.zeros(3),  # NaN 좌표는 (0, 0, 0)으로 처리
                obj_data['orientation'] if 'orientation' in obj_data else torch.zeros(4),  # NaN 객체의 orientation 기본값
                map_label(int(obj_data['label'])),  # 퍼블리시할 때는 매핑된 라벨 사용
                obj_data['score'] if 'score' in obj_data else 0.0,  # score 기본값
                obj_data['size'] if 'size' in obj_data else torch.zeros(3),  # NaN 객체의 size 기본값
                obj_data['is_tracked']
            ))

        # 퍼블리시 호출 확인용 print 추가
        # for i in detection_array.detections.result[i]:
        #     if detection_array.detections.result[i].
        print(f"Publishing data to topic with {len(detection_array.detections)} detections")
        
        pub.publish(detection_array)  # 퍼블리싱 호출

    def create_detection3d(self, obj_id, coords, orientation, label, score, size, is_tracked):
        detection = Detection3D()
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(label)  # 퍼블리시할 때 매핑된 class_id 사용
        hypothesis.hypothesis.score = score  # score 값 설정
        detection.results.append(hypothesis)

        detection.bbox.center.position.x = coords[0].item()
        detection.bbox.center.position.y = coords[1].item()
        detection.bbox.center.position.z = coords[2].item()

        detection.bbox.center.orientation.x = orientation[0].item()
        detection.bbox.center.orientation.y = orientation[1].item()
        detection.bbox.center.orientation.z = orientation[2].item()
        detection.bbox.center.orientation.w = orientation[3].item()

        detection.bbox.size.x = size[0].item()
        detection.bbox.size.y = size[1].item()
        detection.bbox.size.z = size[2].item()

        # is_tracked가 True면 1.0, False면 0.0으로 covariance 설정
        covariance_value = 1.0 if is_tracked else 0.0
        detection.results[0].pose.covariance = [covariance_value] * 36

        detection.id = str(obj_id)  # Track ID 설정

        # if hypothesis.hypothesis.class_id == 4:
        #     print("Pedestrian x : "+detection.bbox.center.position.x+", y : "+detection.bbox.center.position.y+", z : "+detection.bbox.center.position.z)

        return detection

    def is_same_object(self, prev_obj, current_box):
        prev_coords = prev_obj['coords'][:3]
        current_coords = current_box[:3]

        kf = KalmanFilter(init_state=prev_coords.cpu().numpy())
        kf.predict()
        kf.update(current_coords.cpu().numpy())
        predicted_coords = kf.get_state()

        distance = np.linalg.norm(predicted_coords - current_coords.cpu().numpy())
        return distance < 1.3  # 동일 객체로 간주할 임계값 조정 가능


def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
