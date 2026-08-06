import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import open3d as o3d
import struct
from builtin_interfaces.msg import Duration

class EnhancedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_object_detection_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_12_201/TFpoints',
            self.lidar_callback,
            10)
        self.bbox_publisher = self.create_publisher(MarkerArray, '/detection_bbox', 10)
        self.filtered_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        
        # Tracking parameters
        self.frame_count = 0
        self.min_detection_threshold = 2  # Noise filtering 기준
        self.tracking_window = 20
        self.tracked_objects = {}  # 객체 ID별로 [히스토리, 바운딩 박스] 형태로 저장
        self.current_markers = {}  # 현재 표시 중인 마커들을 저장

        self.get_logger().info("Enhanced Object Detection Node with Euclidean Clustering and Persistent Tracking started.")

    def lidar_callback(self, msg):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        filtered_data = self.remove_ground_ransac(point_cloud_data)
        self.publish_filtered_points(filtered_data, msg.header)
        
        clusters = self.euclidean_clustering(filtered_data)
        
        # Tracking and noise filtering process
        self.frame_count += 1
        current_objects = []

        for cluster in clusters:
            bbox = self.create_bbox(cluster)
            object_id = self.get_object_id(bbox)  # 바운딩 박스를 기준으로 객체 ID 생성
            current_objects.append(object_id)

            # Track object frequency and update bbox if object already exists
            if object_id not in self.tracked_objects:
                # Initialize with detection history to ensure 10-frame retention
                self.tracked_objects[object_id] = ([1] + [0] * (self.tracking_window - 1), bbox)
            else:
                # Update detection history and bbox
                history, _ = self.tracked_objects[object_id]
                history[self.frame_count % self.tracking_window] = 1
                self.tracked_objects[object_id] = (history, bbox)

        # Update tracking history and filter noise after 10 frames
        if self.frame_count >= self.tracking_window:
            self.filter_tracked_objects(current_objects)

        # Publish persistent boxes for reliably tracked objects
        self.publish_persistent_boxes()

    def filter_tracked_objects(self, current_objects):
        filtered_objects = {}
        for object_id, (history, bbox) in self.tracked_objects.items():
            detection_count = sum(history)
            # Keep object if it meets the threshold or is currently detected
            if detection_count >= self.min_detection_threshold or object_id in current_objects:
                filtered_objects[object_id] = (history, bbox)
            else:
                # Remove the marker if the object is filtered out
                if object_id in self.current_markers:
                    del self.current_markers[object_id]
                self.get_logger().info(f"Object {object_id} filtered out as noise.")
        
        self.tracked_objects = filtered_objects

    def publish_persistent_boxes(self):
        marker_array = MarkerArray()
        marker_id = 0

        for object_id, (history, bbox) in self.tracked_objects.items():
            if sum(history) > 0:  # Only publish boxes with recent detections
                marker = self.create_marker(bbox, marker_id)
                marker_array.markers.append(marker)
                self.current_markers[object_id] = marker
                marker_id += 1

        self.bbox_publisher.publish(marker_array)

    def create_marker(self, bbox, marker_id):
        x, y, z, dx, dy, dz = bbox
        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = dx
        marker.scale.y = dy
        marker.scale.z = dz
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.lifetime = Duration(sec=2)  # 2초 동안 유지
        return marker

    def get_object_id(self, bbox):
        x, y, z, dx, dy, dz = bbox
        return hash((round(x, 1), round(y, 1), round(z, 1), round(dx, 1), round(dy, 1), round(dz, 1)))

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

    def euclidean_clustering(self, point_cloud_data, tolerance=0.5, min_cluster_size=3, max_cluster_size=1000):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_data[:, :3])
        
        labels = np.array(pcd.cluster_dbscan(eps=tolerance, min_points=min_cluster_size, print_progress=True))
        
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = point_cloud_data[labels == label]
            if min_cluster_size <= len(cluster) <= max_cluster_size:
                clusters.append(cluster)
        
        return clusters

    def create_bbox(self, cluster):
        center_x = np.mean(cluster[:, 0])
        center_y = np.mean(cluster[:, 1])
        center_z = np.mean(cluster[:, 2])

        dx = np.max(cluster[:, 0]) - np.min(cluster[:, 0])
        dy = np.max(cluster[:, 1]) - np.min(cluster[:, 1])
        dz = np.max(cluster[:, 2]) - np.min(cluster[:, 2])
        
        return [center_x, center_y, center_z, dx, dy, dz]

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

