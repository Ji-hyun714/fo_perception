# rule_based_detection_with_knn_and_range.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np
from sklearn.cluster import DBSCAN
from filterpy.kalman import KalmanFilter

class RuleBasedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('rule_based_object_detection_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_12_201/TFpoints',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/detection_result', 10)
        self.bbox_publisher = self.create_publisher(Marker, '/detection_bbox', 10)
        
        # Kalman 필터 초기화
        self.car_tracker = self.create_kalman_filter()
        self.person_tracker = self.create_kalman_filter()
        
        self.get_logger().info("Rule-Based Object Detection Node with BBox, KNN, Range Limit, and Ground Removal has started.")

    def create_kalman_filter(self):
        kf = KalmanFilter(dim_x=6, dim_z=3)
        kf.F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        kf.R *= 10
        kf.P *= 100
        kf.Q *= 0.1
        return kf

    def lidar_callback(self, msg):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        # 인식 범위 제한 적용 및 바닥면 제거
        filtered_data = self.apply_detection_range(point_cloud_data)
        filtered_data = self.remove_ground_plane(filtered_data)
        
        # KNN 기반 클러스터링
        clusters = self.cluster_points(filtered_data)

        # 각 클러스터에 대해 자동차 및 사람 감지 수행
        for cluster in clusters:
            car_detected, car_bbox = self.detect_object(cluster, 'car')
            person_detected, person_bbox = self.detect_object(cluster, 'person')
            
            if car_detected:
                self.update_tracking(self.car_tracker, car_bbox, "Car")
            if person_detected:
                self.update_tracking(self.person_tracker, person_bbox, "Person")

    def convert_pointcloud2_to_array(self, msg):
        cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        return cloud_data.reshape(-1, 4)

    def apply_detection_range(self, point_cloud_data, x_range=(-20, 20), y_range=(-20, 20), z_range=(-0.5, 3)):
        # 설정된 x, y, z 범위 내 포인트만 남기기
        mask = (
            (point_cloud_data[:, 0] >= x_range[0]) & (point_cloud_data[:, 0] <= x_range[1]) &
            (point_cloud_data[:, 1] >= y_range[0]) & (point_cloud_data[:, 1] <= y_range[1]) &
            (point_cloud_data[:, 2] >= z_range[0]) & (point_cloud_data[:, 2] <= z_range[1])
        )
        return point_cloud_data[mask]

    def remove_ground_plane(self, point_cloud_data, z_threshold=0.0):
        # z_threshold 이하의 포인트들은 바닥면으로 간주하여 제거
        return point_cloud_data[point_cloud_data[:, 2] > z_threshold]

    def cluster_points(self, point_cloud_data, eps=0.5, min_samples=10):
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(point_cloud_data[:, :3])
        
        clusters = []
        for label in set(labels):
            if label == -1:
                continue  # 노이즈 포인트 무시
            clusters.append(point_cloud_data[labels == label])
        
        return clusters

    def detect_object(self, point_cloud_data, object_type):
        if object_type == 'car':
            density_threshold = 50
            bbox_thresholds = [1.5, 3.0]  # 최소 높이와 폭
        elif object_type == 'person':
            density_threshold = 20
            bbox_thresholds = [0.2, 1.0]
        else:
            return False, None

        if len(point_cloud_data) < density_threshold:
            return False, None

        bbox = [
            np.min(point_cloud_data[:, 0]), np.max(point_cloud_data[:, 0]),
            np.min(point_cloud_data[:, 1]), np.max(point_cloud_data[:, 1]),
            np.min(point_cloud_data[:, 2]), np.max(point_cloud_data[:, 2])
        ]
        
        if (bbox[1] - bbox[0] > bbox_thresholds[0] and
            bbox[3] - bbox[2] > bbox_thresholds[1]):
            return True, bbox
        else:
            return False, None

    def update_tracking(self, tracker, bbox, label):
        center_x = (bbox[0] + bbox[1]) / 2
        center_y = (bbox[2] + bbox[3]) / 2
        center_z = (bbox[4] + bbox[5]) / 2
        
        tracker.predict()
        tracker.update([center_x, center_y, center_z])
        
        x, y, z = float(tracker.x[0]), float(tracker.x[1]), float(tracker.x[2])

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
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        self.bbox_publisher.publish(marker)

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

