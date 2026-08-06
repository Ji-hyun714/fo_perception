import numpy as np

# numpy 호환성 처리
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class LiDARTransformer(Node):
    def __init__(self):
        super().__init__('lidar_transformer')

        # 입력 LiDAR 토픽
        self.input_topic = '/TFnonground/lidar_12'

        # 출력 토픽
        self.output_topic = '/result/lidar_12'

        # velo12 -> velo11 calibration transformation matrix
        self.transform_matrix = np.array([
            [ 9.96366443e-01,  8.50214918e-02, -5.02558782e-03,  2.39903469e+00],
            [-8.50843091e-02,  9.90986312e-01, -1.03473617e-01,  5.66384364e+00],
            [-3.81719253e-03,  1.03525238e-01,  9.94619502e-01,  2.01982353e-01],
            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
        ], dtype=np.float64)

        # LiDAR 토픽 구독
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.lidar_callback,
            1
        )

        # 변환된 PointCloud2 publish
        self.transformed_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            1
        )

        self.get_logger().info(
            f"LiDAR transformer started: {self.input_topic} -> {self.output_topic}"
        )
        self.get_logger().info(
            "Applying calibration matrix: velo12 -> velo11"
        )

    def lidar_callback(self, cloud_msg):
        transformed_points = self.apply_transform(cloud_msg)

        if transformed_points is None:
            self.get_logger().warn("Empty point cloud received.")
            return

        transformed_cloud_msg = self.create_point_cloud2(
            transformed_points,
            frame_id='velo11',
            stamp=cloud_msg.header.stamp
        )

        self.transformed_pub.publish(transformed_cloud_msg)

    def apply_transform(self, cloud_msg):
        points = self.pointcloud2_to_xyz_intensity_array(cloud_msg)

        if points.size == 0:
            return None

        xyz = points[:, :3]
        intensity = points[:, 3:4]

        # [x, y, z] -> [x, y, z, 1]
        ones = np.ones((xyz.shape[0], 1), dtype=np.float64)
        xyz_h = np.hstack((xyz.astype(np.float64), ones))

        # 4x4 transformation matrix 적용
        transformed_xyz_h = np.dot(xyz_h, self.transform_matrix.T)

        # x, y, z만 사용
        transformed_xyz = transformed_xyz_h[:, :3]

        # intensity 유지
        transformed_points = np.hstack((transformed_xyz, intensity))

        return transformed_points

    def pointcloud2_to_xyz_intensity_array(self, cloud_msg):
        points_list = []

        for point in pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        ):
            points_list.append([
                float(point[0]),
                float(point[1]),
                float(point[2]),
                float(point[3])
            ])

        if len(points_list) == 0:
            return np.empty((0, 4), dtype=np.float32)

        return np.asarray(points_list, dtype=np.float32)

    def create_point_cloud2(self, points, frame_id, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = points.shape[0]

        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True

        msg.data = np.asarray(points, dtype=np.float32).tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = LiDARTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LiDAR transformer stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()