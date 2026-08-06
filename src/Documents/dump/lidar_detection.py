# rule_based_detection_with_tracking.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np
from filterpy.kalman import KalmanFilter

class RuleBasedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('rule_based_object_detection_node')

        # LiDAR 토픽 및 감지 결과 퍼블리셔 설정
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_11_201/TFpoints',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/detection_result', 10)
        self.bbox_publisher = self.create_publisher(Marker, '/detection_bbox', 10)

        # Kalman 필터 초기화
        self.car_tracker = self.create_kalman_filter()
        self.person_tracker = self.create_kalman_filter()

        self.get_logger().info("Rule-Based Object Detection Node with BBox and Tracking has started.")

    def create_kalman_filter(self):
        # Kalman 필터 객체 초기화 (x, y, z를 추적)
        kf = KalmanFilter(dim_x=6, dim_z=3)
        kf.F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])  # 상태 전이 행렬
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])  # 측정 함수
        kf.R *= 10  # 측정 노이즈
        kf.P *= 100  # 초기 추정값의 불확실성
        kf.Q *= 0.1  # 프로세스 노이즈
        return kf

    def lidar_callback(self, msg):
        # PointCloud2 데이터를 NumPy 배열로 변환
        point_cloud_data = self.convert_pointcloud2_to_array(msg)

        # 자동차 및 사람 감지
        car_detected, car_bbox = self.detect_object(point_cloud_data, 'car')
        person_detected, person_bbox = self.detect_object(point_cloud_data, 'person')

        # Kalman 필터로 추적 갱신
        if car_detected:
            self.update_tracking(self.car_tracker, car_bbox, "Car")
        if person_detected:
            self.update_tracking(self.person_tracker, person_bbox, "Person")

    def convert_pointcloud2_to_array(self, msg):
        cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        return cloud_data.reshape(-1, 4)

    def detect_object(self, point_cloud_data, object_type):
        # 객체 유형에 따른 감지 파라미터 설정
        if object_type == 'car':
            x_min, x_max = -20, 20
            y_min, y_max = -20, 20
            z_min, z_max = -1, 3
            density_threshold = 100
        elif object_type == 'person':
            x_min, x_max = -20, 20
            y_min, y_max = -20, 20
            z_min, z_max = -1, 2
            density_threshold = 30
        else:
            return False, None

        mask = (
            (point_cloud_data[:, 0] >= x_min) & (point_cloud_data[:, 0] <= x_max) &
            (point_cloud_data[:, 1] >= y_min) & (point_cloud_data[:, 1] <= y_max) &
            (point_cloud_data[:, 2] >= z_min) & (point_cloud_data[:, 2] <= z_max)
        )
        points_in_area = point_cloud_data[mask]

        # 밀집도 검사
        if len(points_in_area) > density_threshold:
            bbox = [
                np.min(points_in_area[:, 0]), np.max(points_in_area[:, 0]),
                np.min(points_in_area[:, 1]), np.max(points_in_area[:, 1]),
                np.min(points_in_area[:, 2]), np.max(points_in_area[:, 2])
            ]
            return True, bbox
        else:
            return False, None

    def update_tracking(self, tracker, bbox, label):
        # BBox 중심점 계산
        center_x = (bbox[0] + bbox[1]) / 2
        center_y = (bbox[2] + bbox[3]) / 2
        center_z = (bbox[4] + bbox[5]) / 2

        # 측정 업데이트 및 추정된 위치 얻기
        tracker.predict()
        tracker.update([center_x, center_y, center_z])
        x, y, z = float(tracker.x[0]), float(tracker.x[1]), float(tracker.x[2])  # 칼만 필터로 추정된 위치

        # BBox Marker 메시지 퍼블리시
        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = float(bbox[1] - bbox[0])
        marker.scale.y = float(bbox[3] - bbox[2])
        marker.scale.z = float(bbox[5] - bbox[4])
        marker.color.a = 0.5
        marker.color.r = 1.0 if label == "Car" else 0.0
        marker.color.g = 1.0 if label == "Person" else 0.0
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        self.bbox_publisher.publish(marker)

        # 감지 결과 퍼블리시
        result_msg = String()
        result_msg.data = f"{label} Detected at ({x:.2f}, {y:.2f}, {z:.2f})"
        self.publisher_.publish(result_msg)
        self.get_logger().info(result_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

