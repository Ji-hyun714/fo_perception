import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter
import math

class TrackedObject:
    def __init__(self, id, initial_position):
        # Kalman Filter 초기화
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])
        self.kf.R *= 0.1  # 측정 노이즈
        self.kf.P *= 10.0  # 초기 불확실성
        self.kf.Q *= 0.1  # 프로세스 노이즈

        # 초기 위치 설정
        self.kf.x[:3] = initial_position.reshape((3, 1))
        self.id = id
        self.age = 0
        self.missed = 0  # 매칭되지 않은 프레임 수

    def predict(self):
        self.kf.predict()
        self.age += 1
        return self.kf.x[:3].flatten()

    def update(self, measurement):
        self.kf.update(measurement)
        self.missed = 0  # 매칭 성공 시 초기화


class EnhancedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_object_detection_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ldiar_12_201/TFpoints',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/detection_result', 10)
        self.bbox_publisher = self.create_publisher(Marker, '/detection_bbox', 10)
        # 바닥 제거된 포인트 클라우드 퍼블리셔 추가
        self.filtered_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        
        # 트랙 관리
        self.tracks = []
        self.next_id = 1  # 새로운 객체 ID 부여

        self.get_logger().info("Enhanced Object Detection Node with RANSAC, DBSCAN, Size Filtering, and Height Histogram Analysis started.")

    def lidar_callback(self, msg):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        # 바닥 제거 및 -0.4m 이하 포인트 제거
        filtered_data = self.remove_ground_ransac(point_cloud_data)
        # 바닥 제거된 포인트 클라우드를 새로운 토픽으로 퍼블리시
        self.publish_filtered_points(filtered_data, msg.header)
        
        # 클러스터링 수행
        clusters = self.cluster_points(filtered_data)

        # 클러스터 중심 계산
        cluster_centroids = np.array([np.mean(cluster[:, :3], axis=0) for cluster in clusters])

        # 추적 업데이트
        self.update_tracks(cluster_centroids)

        # 각 클러스터에 대해 히스토그램 분석을 통해 사람과 자동차 구분
        for cluster in clusters:
            car_detected, car_bbox = self.detect_object(cluster, 'car')
            person_detected, person_bbox = self.detect_object(cluster, 'person')
            
            if car_detected:
                self.publish_detection("Car", car_bbox)
            if person_detected:
                self.publish_detection("Person", person_bbox)
                
    def publish_filtered_points(self, filtered_points, header):
        # PointCloud2 메시지 생성
        filtered_cloud_msg = PointCloud2()
        filtered_cloud_msg.header = header
        filtered_cloud_msg.height = 1
        filtered_cloud_msg.width = filtered_points.shape[0]
        filtered_cloud_msg.is_dense = True
        filtered_cloud_msg.is_bigendian = False
        filtered_cloud_msg.fields = [
            sensor_msgs.PointField(name='x', offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='y', offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='z', offset=8, datatype=sensor_msgs.PointField.FLOAT32, count=1),
        ]
        filtered_cloud_msg.point_step = 12
        filtered_cloud_msg.row_step = filtered_cloud_msg.point_step * filtered_points.shape[0]
        filtered_cloud_msg.data = np.asarray(filtered_points, dtype=np.float32).tobytes()
        
        # 퍼블리시
        self.filtered_cloud_publisher.publish(filtered_cloud_msg)

    def convert_pointcloud2_to_array(self, msg):
        cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        return cloud_data.reshape(-1, 4)

    def remove_ground_ransac(self, point_cloud_data, distance_threshold=0.15):
        # RANSAC을 사용한 바닥 제거
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_data[:, :3])
        
        # Voxel Grid Filtering을 사용한 다운샘플링
        pcd = pcd.voxel_down_sample(voxel_size=0.1)
        
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                                 ransac_n=3,
                                                 num_iterations=100)
        ground_removed = pcd.select_by_index(inliers, invert=True)
        filtered_points = np.asarray(ground_removed.points)
        
        # 추가 조건: -0.4m 이하 포인트 제거
        filtered_points = filtered_points[filtered_points[:, 2] > -0.2]
        
        # 인식 범위 제한
        x_min, x_max = -30, 30
        y_min, y_max = -30, 30
        z_min, z_max = -0.5, 3
        
        mask = (filtered_points[:, 0] >= x_min) & (filtered_points[:, 0] <= x_max) & \
               (filtered_points[:, 1] >= y_min) & (filtered_points[:, 1] <= y_max) & \
               (filtered_points[:, 2] >= z_min) & (filtered_points[:, 2] <= z_max)
        
        return filtered_points[mask]

    def cluster_points(self, point_cloud_data, eps=0.3, min_samples=5):
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(point_cloud_data[:, :3])
        
        clusters = []
        for label in set(labels):
            if label == -1:
                continue  # 노이즈 포인트 무시
            clusters.append(point_cloud_data[labels == label])
        
        return clusters

    def update_tracks(self, detections):
        if len(self.tracks) == 0:
            # 초기화 단계: 모든 탐지된 객체를 새 트랙으로 등록
            for detection in detections:
                self.tracks.append(TrackedObject(self.next_id, detection))
                self.next_id += 1
            return

        # 모든 트랙과 탐지 간의 거리 행렬 계산
        track_predictions = np.array([track.predict() for track in self.tracks])
        distance_matrix = np.linalg.norm(track_predictions[:, np.newaxis] - detections, axis=2)

        # Hungarian Algorithm으로 최적 매칭 수행
        row_ind, col_ind = linear_sum_assignment(distance_matrix)
        
        # 매칭 결과 적용
        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections)))

        for r, c in zip(row_ind, col_ind):
            if distance_matrix[r, c] < 1.0:  # 매칭 임계값
                self.tracks[r].update(detections[c])
                unmatched_tracks.remove(r)
                unmatched_detections.remove(c)

        # 매칭되지 않은 탐지들은 새로운 트랙으로 추가
        for idx in unmatched_detections:
            self.tracks.append(TrackedObject(self.next_id, detections[idx]))
            self.next_id += 1

        # 매칭되지 않은 트랙들의 age 증가
        for idx in unmatched_tracks:
            self.tracks[idx].missed += 1

        # 오래된 트랙 제거
        self.tracks = [track for track in self.tracks if track.missed < 5]

        # 트랙 시각화
        for track in self.tracks:
            pos = track.kf.x[:3]
            self.publish_detection(f"ID {track.id}", [pos[0], pos[1], pos[2], 1.0, 1.0, 1.0])

    def detect_object(self, point_cloud_data, object_type):
        # 객체 유형에 따른 필터 기준 설정
        if object_type == 'car':
            density_threshold = 30
            height_range = (-0.2, 2.0)  # 자동차의 높이 분포 범위
        elif object_type == 'person':
            density_threshold = 10
            height_range = (-0.2, 1.5)  # 사람의 높이 분포 범위
        else:
            return False, None

        if len(point_cloud_data) < density_threshold:
            return False, None

        # BBox 생성
        bbox = [
            np.min(point_cloud_data[:, 0]), np.max(point_cloud_data[:, 0]),
            np.min(point_cloud_data[:, 1]), np.max(point_cloud_data[:, 1]),
            np.min(point_cloud_data[:, 2]), np.max(point_cloud_data[:, 2])
        ]

        # 히스토그램 분석으로 높이 분포 필터링
        z_values = point_cloud_data[:, 2]
        histogram, bin_edges = np.histogram(z_values, bins=10, range=height_range)
        
        # 특정 높이 범위에 포인트가 집중되어 있으면 객체로 간주
        peak_density = np.max(histogram)
        if peak_density < density_threshold:
            return False, None

        return True, [bbox[0], bbox[2], bbox[4], bbox[1] - bbox[0], bbox[3] - bbox[2], bbox[5] - bbox[4]]

    def publish_detection(self, label, bbox):
        x, y, z, dx, dy, dz = [float(v) for v in bbox]

        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = dx
        marker.scale.y = dy
        marker.scale.z = dz
        marker.color.a = 0.5
        marker.color.r = 1.0 if label == "Car" else 0.0
        marker.color.g = 1.0 if label == "Person" else 0.0
        marker.color.b = 0.0
        marker.pose.position.x = x + dx / 2.0  # BBox 중심 설정
        marker.pose.position.y = y + dy / 2.0
        marker.pose.position.z = z + dz / 2.0
        self.bbox_publisher.publish(marker)

        result_msg = String()
        result_msg.data = f"{label} Detected at ({x + dx / 2.0:.2f}, {y + dy / 2.0:.2f}, {z + dz / 2.0:.2f})"
        self.publisher_.publish(result_msg)
        self.get_logger().info(result_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

