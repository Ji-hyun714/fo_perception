import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import tf_transformations  # 좌표 변환을 위한 라이브러리

class LiDARTransformer(Node):
    def __init__(self):
        super().__init__('lidar_transformer')

        # LiDAR 토픽을 구독
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            # '/lidar_12_201/points',  # LiDAR 토픽 이름
            '/nonground/lidar_12',  # LiDAR 토픽 이름
            self.lidar_callback,
            1
        )

        # 변환된 LiDAR 데이터를 새로운 토픽으로 발행
        # self.transformed_pub = self.create_publisher(PointCloud2, '/lidar_12_201/TFpoints', 1)
        self.transformed_pub = self.create_publisher(PointCloud2, '/TFnonground/lidar_12', 1)

        # 변환 값 (translation과 rotation)
        self.translation = np.array([2.2, -1, 0])  # X, Y, Z 평행이동
        self.rotation_matrix = tf_transformations.quaternion_matrix([0, 0, -0.543835, 0.839192])[:3, :3]  # 3x3 회전 행렬 생성
        
        self.get_logger().info("Translate LiDAR_12 points")
        
        
    def lidar_callback(self, cloud_msg):
        # LiDAR 데이터를 받아서 변환 적용
        transformed_points = self.apply_transform(cloud_msg)

        if transformed_points is not None:
            # 변환된 데이터를 새로운 토픽으로 발행
            transformed_cloud_msg = self.create_point_cloud2(transformed_points, 'vehicle')
            self.transformed_pub.publish(transformed_cloud_msg)

    def apply_transform(self, cloud_msg):
        # PointCloud2 데이터를 받아서 변환 적용
        points = self.pointcloud2_to_xyz_intensity_array(cloud_msg)

        if points.size == 0:
            return None

        # 벡터 연산으로 회전 적용
        rotated_points = np.dot(points[:, :3], self.rotation_matrix.T)  # x, y, z에 회전 적용

        # 평행이동 적용 (벡터 연산)
        translated_points = rotated_points + self.translation  # 회전 후 평행이동 적용

        # intensity 유지하며 결합
        transformed_points = np.hstack((translated_points, points[:, 3:4]))  # intensity 유지

        return transformed_points

    def pointcloud2_to_xyz_intensity_array(self, cloud_msg):
        # PointCloud2 메시지를 XYZ 및 intensity 배열로 변환
        field_names = [field.name for field in cloud_msg.fields]
        points_list = []

        if "intensity" in field_names:
            for point in pc2.read_points(
                cloud_msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True
            ):
                points_list.append([point[0], point[1], point[2], point[3]])
        else:
            for point in pc2.read_points(
                cloud_msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ):
                points_list.append([point[0], point[1], point[2], 255.0])

        return np.array(points_list, dtype=np.float32)

    def create_point_cloud2(self, points, parent_frame):
        # 변환된 포인트 클라우드를 새로운 PointCloud2 메시지로 생성
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.is_dense = True
        msg.data = np.asarray(points, np.float32).tobytes()
        return msg

def main(args=None):
    rclpy.init(args=args)
    lidar_transformer = LiDARTransformer()
    rclpy.spin(lidar_transformer)
    lidar_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
