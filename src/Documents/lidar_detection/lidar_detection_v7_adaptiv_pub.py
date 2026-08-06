import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker
import numpy as np
import open3d as o3d
from collections import deque
from builtin_interfaces.msg import Duration
import threading
from time import sleep

class EnhancedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_object_detection_node')
        # Subscribe Topics
        self.subscription_11 = self.create_subscription(PointCloud2, '/lidar_11_201/TFpoints', self.lidar_callback_11, 10)
        self.subscription_12 = self.create_subscription(PointCloud2, '/lidar_12_201/TFpoints', self.lidar_callback_12, 10)
        self.subscription_13 = self.create_subscription(PointCloud2, '/lidar_13_201/TFpoints', self.lidar_callback_13, 10)
        self.subscription_all = self.create_subscription(PointCloud2, '/merge_201/points', self.lidar_callback_all, 10)
        
        # Publishers for Detection3DArray
        self.detect_publisher_11 = self.create_publisher(Detection3DArray, '/lidar_11_201/TFdetect', 10)
        self.detect_publisher_12 = self.create_publisher(Detection3DArray, '/lidar_12_201/TFdetect', 10)
        self.detect_publisher_13 = self.create_publisher(Detection3DArray, '/lidar_13_201/TFdetect', 10)
        self.detect_publisher_all = self.create_publisher(Detection3DArray, '/merge_201/TFdetect', 10)
        
        # Publisher for BBOX visualization
        self.bbox_publisher = self.create_publisher(Marker, '/detection_bbox', 10)
        
        # Publisher for filtered PointCloud2
        self.filtered_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

        self.bounding_boxes = deque()
        self.max_duration = Duration(sec=2)
        self.bbox_id = 0
        
        self.bbox_thread = threading.Thread(target=self.publish_boxes_periodically)
        self.bbox_thread.daemon = True
        self.bbox_thread.start()

        self.get_logger().info("Enhanced Object Detection Node with Euclidean Clustering using Open3D started.")

    def lidar_callback_11(self, msg):
        self.process_lidar_data(msg, self.detect_publisher_11)

    def lidar_callback_12(self, msg):
        self.process_lidar_data(msg, self.detect_publisher_12)

    def lidar_callback_13(self, msg):
        self.process_lidar_data(msg, self.detect_publisher_13)

    def lidar_callback_all(self, msg):
        self.process_lidar_data(msg, self.detect_publisher_all)

    def process_lidar_data(self, msg, publisher):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        filtered_data = self.remove_ground_ransac(point_cloud_data)
        self.publish_filtered_points(filtered_data, msg.header)
        
        clusters = self.euclidean_clustering(filtered_data)

        detection_array = Detection3DArray()
        detection_array.header = msg.header

        for cluster in clusters:
            bbox = self.create_bbox(cluster)
            density = self.calculate_density(cluster)
            
            detection = Detection3D()
            detection.bbox.center.position.x = bbox[0]
            detection.bbox.center.position.y = bbox[1]
            detection.bbox.center.position.z = bbox[2]
            detection.bbox.center.orientation.x = 0.0
            detection.bbox.center.orientation.y = 0.0
            detection.bbox.center.orientation.z = 0.0
            detection.bbox.center.orientation.w = 1.0
            detection.bbox.size.x = bbox[3]
            detection.bbox.size.y = bbox[4]
            detection.bbox.size.z = bbox[5]

            hypothesis = ObjectHypothesisWithPose()
            # print("hypothesis : " , dir(hypothesis.hypothesis))
            if detection.bbox.size.x <=0.8 and detection.bbox.size.y <=0.8:
                # hypothesis.id = '4'
                hypothesis.hypothesis.class_id = '4'
            else:
                # hypothesis.id = '1'
                hypothesis.hypothesis.class_id = '1'
            # hypothesis.score = density
            hypothesis.hypothesis.score = density
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
            self.bounding_boxes.append((bbox, self.bbox_id))
            self.bbox_id += 1
        
        publisher.publish(detection_array)

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

    def publish_boxes_periodically(self):
        while rclpy.ok():
            if self.bounding_boxes:
                bbox, bbox_id = self.bounding_boxes.popleft()
                self.publish_bbox(bbox, bbox_id)
            sleep(0.1)

    def publish_bbox(self, bbox, bbox_id):
        x, y, z, dx, dy, dz = bbox
        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = bbox_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = dx
        marker.scale.y = dy
        marker.scale.z = dz

        if dx <= 0.8 and dy <= 0.8:
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.lifetime = self.max_duration
        self.bbox_publisher.publish(marker)

    def euclidean_clustering(self, point_cloud_data, min_cluster_size=3, max_cluster_size=1000):
        clusters = []
        close_points = point_cloud_data[np.linalg.norm(point_cloud_data[:, :2], axis=1) < 10]
        if len(close_points) > 0:
            clusters.extend(self.cluster_with_tolerance(close_points, tolerance=0.5, min_cluster_size=min_cluster_size, max_cluster_size=max_cluster_size))

        far_points = point_cloud_data[np.linalg.norm(point_cloud_data[:, :2], axis=1) >= 10]
        if len(far_points) > 0:
            clusters.extend(self.cluster_with_tolerance(far_points, tolerance=1.0, min_cluster_size=min_cluster_size, max_cluster_size=max_cluster_size))
        
        return clusters

    def cluster_with_tolerance(self, points, tolerance, min_cluster_size, max_cluster_size):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        labels = np.array(pcd.cluster_dbscan(eps=tolerance, min_points=min_cluster_size, print_progress=True))
        
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = points[labels == label]
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

    def calculate_density(self, cluster):
        return len(cluster) / (np.ptp(cluster[:, 0]) * np.ptp(cluster[:, 1]) * np.ptp(cluster[:, 2]))

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
