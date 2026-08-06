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
import struct
import math

class TrackedObject:
    def __init__(self, object_id, bbox):
        self.id = object_id
        self.bbox = bbox
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.eye(6)
        self.kf.F[:3, 3:] = np.eye(3) * 0.1  # Transition matrix with dt
        self.kf.H = np.eye(3, 6)
        self.kf.R *= 0.1
        self.kf.P *= 10
        self.kf.x[:3] = np.reshape(bbox[:3], (3, 1))
        self.time_since_update = 0

    def update(self, bbox):
        self.kf.update(np.reshape(bbox[:3], (3, 1)))
        self.bbox = bbox
        self.time_since_update = 0

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        predicted_pos = self.kf.x[:3].flatten()
        return [predicted_pos[0], predicted_pos[1], predicted_pos[2], self.bbox[3], self.bbox[4], self.bbox[5]]


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
        
        self.tracks = []
        self.next_id = 1
        
        self.get_logger().info("Enhanced Object Detection Node with Full Cluster Bounding Boxes and Tracking started.")

    def lidar_callback(self, msg):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        filtered_data = self.remove_ground_ransac(point_cloud_data)
        self.publish_filtered_points(filtered_data, msg.header)
        
        clusters = self.cluster_points(filtered_data)
        
        detections = [self.create_bbox_for_cluster(cluster) for cluster in clusters]
        self.update_tracks(detections)

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

    def cluster_points(self, point_cloud_data, eps=0.5, min_samples=5):
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(point_cloud_data[:, :3])
        
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            clusters.append(point_cloud_data[labels == label])
        
        return clusters

    def create_bbox_for_cluster(self, cluster):
        x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
        y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
        z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])

        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        center_z = (z_min + z_max) / 2.0
        length = x_max - x_min
        width = y_max - y_min
        height = z_max - z_min

        return [center_x, center_y, center_z, length, width, height]

    def update_tracks(self, detections):
        if len(self.tracks) == 0:
            for det in detections:
                self.tracks.append(TrackedObject(self.next_id, det))
                self.next_id += 1
        else:
            # Calculate cost matrix for Hungarian algorithm
            cost_matrix = []
            for track in self.tracks:
                track_pos = track.kf.x[:3]
                cost_row = [np.linalg.norm(track_pos - np.array(det[:3])) for det in detections]
                cost_matrix.append(cost_row)

            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            # Update matched tracks
            matched_detections = set()
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r][c] < 5.0:  # Threshold to consider it a match
                    self.tracks[r].update(detections[c])
                    matched_detections.add(c)

            # Add new tracks for unmatched detections
            for i, det in enumerate(detections):
                if i not in matched_detections:
                    self.tracks.append(TrackedObject(self.next_id, det))
                    self.next_id += 1

            # Remove stale tracks
            self.tracks = [t for t in self.tracks if t.time_since_update < 5]

        # Publish all tracks
        for track in self.tracks:
            predicted_bbox = track.predict()
            self.publish_detection(f"ID {track.id}", predicted_bbox)

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
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
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

