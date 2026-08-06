import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import struct
import math

CAR_BBOX = [2.0, 4.0, 1.5]  # 차량 크기 (길이, 너비, 높이)
PERSON_BBOX = [0.5, 0.5, 1.8]  # 사람 크기 (길이, 너비, 높이)

class EnhancedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_object_detection_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_12_201/TFpoints',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/detection_result', 10)
        self.bbox_publisher = self.create_publisher(Marker, '/detection_bbox', 10)
        self.filtered_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        
        self.get_logger().info("Enhanced Object Detection Node with RANSAC, DBSCAN, and Density-Based Filtering started.")

    def lidar_callback(self, msg):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        filtered_data = self.remove_ground_ransac(point_cloud_data)
        self.publish_filtered_points(filtered_data, msg.header)
        
        clusters = self.cluster_points(filtered_data)

        for cluster in clusters:
            object_type, bbox = self.classify_and_create_bbox(cluster)
            self.publish_detection(object_type, bbox)
                
    def publish_filtered_points(self, filtered_points, header):
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
        
        self.filtered_cloud_publisher.publish(filtered_cloud_msg)

    def convert_pointcloud2_to_array(self, msg):
        cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        return cloud_data.reshape(-1, 4)

    def remove_ground_ransac(self, point_cloud_data, distance_threshold=0.15):
        valid_mask = np.isfinite(point_cloud_data).all(axis=1)
        point_cloud_data = point_cloud_data[valid_mask]
        if point_cloud_data.shape[0] < 3:
           self.get_logger().warning("Not enough valid points for RANSAC ground removal.")
           return np.empty((0, 3))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_data[:, :3])
        pcd = pcd.voxel_down_sample(voxel_size=0.1)
        
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                                 ransac_n=3,
                                                 num_iterations=100)
        ground_removed = pcd.select_by_index(inliers, invert=True)
        filtered_points = np.asarray(ground_removed.points)
        
        filtered_points = filtered_points[filtered_points[:, 2] > -0.2]
        
        x_min, x_max = -30, 30
        y_min, y_max = -30, 30
        z_min, z_max = -0.5, 3
        
        mask = (filtered_points[:, 0] >= x_min) & (filtered_points[:, 0] <= x_max) & \
               (filtered_points[:, 1] >= y_min) & (filtered_points[:, 1] <= y_max) & \
               (filtered_points[:, 2] >= z_min) & (filtered_points[:, 2] <= z_max)
        
        return filtered_points[mask]

    def cluster_points(self, point_cloud_data, eps=1, min_samples=5):
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(point_cloud_data[:, :3])
        
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            clusters.append(point_cloud_data[labels == label])
        
        return clusters

    def classify_and_create_bbox(self, cluster):
        # 중심 좌표 계산
        center_x = np.mean(cluster[:, 0])
        center_y = np.mean(cluster[:, 1])
        center_z = np.mean(cluster[:, 2])

        # 클러스터의 밀도 기반으로 객체 분류
        area = (np.max(cluster[:, 0]) - np.min(cluster[:, 0])) * (np.max(cluster[:, 1]) - np.min(cluster[:, 1]))
        density = len(cluster) / area if area > 0 else 0

        # 밀도가 낮으면 차량, 높으면 사람으로 분류
        if density < 0.05:  # 차량의 밀도 기준
            object_type = "Car"
            bbox_size = CAR_BBOX
        else:
            object_type = "Person"
            bbox_size = PERSON_BBOX

        return object_type, [center_x, center_y, center_z, *bbox_size]

    def publish_detection(self, label, bbox):
        x, y, z, dx, dy, dz = bbox

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

