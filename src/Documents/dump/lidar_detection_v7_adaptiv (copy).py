import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import Marker
import numpy as np
import open3d as o3d
import struct
from collections import deque
from builtin_interfaces.msg import Duration
import threading
from time import sleep

class EnhancedObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_object_detection_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_12_201/TFpoints',
            self.lidar_callback,
            10)
        self.bbox_publisher = self.create_publisher(Marker, '/detection_bbox', 10)
        self.filtered_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

        # 바운딩 박스를 저장할 큐와 최대 유지 시간
        self.bounding_boxes = deque()
        self.max_duration = Duration(sec=2)  # 2초 동안 유지
        self.bbox_id = 0  # 바운딩 박스 ID
        
        # 별도의 스레드에서 바운딩 박스를 주기적으로 퍼블리시
        self.bbox_thread = threading.Thread(target=self.publish_boxes_periodically)
        self.bbox_thread.daemon = True
        self.bbox_thread.start()

        self.get_logger().info("Enhanced Object Detection Node with Euclidean Clustering using Open3D started.")

    def lidar_callback(self, msg):
        point_cloud_data = self.convert_pointcloud2_to_array(msg)
        
        filtered_data = self.remove_ground_ransac(point_cloud_data)
        self.publish_filtered_points(filtered_data, msg.header)
        
        clusters = self.euclidean_clustering(filtered_data)

        # 새로 인식된 클러스터에 대해 바운딩 박스를 생성하고 큐에 추가
        for cluster in clusters:
            bbox = self.create_bbox(cluster)
            self.bounding_boxes.append((bbox, self.bbox_id))  # 큐에 추가
            self.bbox_id += 1  # 각 바운딩 박스에 고유 ID 부여
                
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
        # 주기적으로 바운딩 박스를 퍼블리시
        while rclpy.ok():
            if self.bounding_boxes:
                bbox, bbox_id = self.bounding_boxes.popleft()  # FIFO 방식으로 바운딩 박스 가져오기
                self.publish_bbox(bbox, bbox_id)
            sleep(0.1)  # 0.1초마다 큐에서 바운딩 박스를 가져와 퍼블리시

    def publish_bbox(self, bbox, bbox_id):
        x, y, z, dx, dy, dz = bbox

        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = bbox_id  # 큐에 저장된 고유 ID 사용
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
        marker.lifetime = self.max_duration  # 2초 동안 유지
        self.bbox_publisher.publish(marker)

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
        # Open3D point cloud 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_data[:, :3])
        
        # 유클리드 클러스터링 수행
        labels = np.array(pcd.cluster_dbscan(eps=tolerance, min_points=min_cluster_size, print_progress=True))
        
        clusters = []
        for label in set(labels):
            if label == -1:  # 노이즈 포인트 제외
                continue
            cluster = point_cloud_data[labels == label]
            if min_cluster_size <= len(cluster) <= max_cluster_size:
                clusters.append(cluster)
        
        return clusters

    def create_bbox(self, cluster):
        # 중심 좌표 계산
        center_x = np.mean(cluster[:, 0])
        center_y = np.mean(cluster[:, 1])
        center_z = np.mean(cluster[:, 2])

        # 각 클러스터의 최대/최소 값을 이용한 바운딩 박스 생성
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

