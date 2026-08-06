#!/usr/bin/env python3

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class Open3DRansacGroundNode(Node):
    def __init__(self):
        super().__init__('open3d_ransac_ground_lidar_11')

        self.declare_parameter('input_topic', '/lidar_11_201/points')
        self.declare_parameter('ground_topic', '/ransac_ground/lidar_11_201')
        self.declare_parameter('nonground_topic', '/ransac_nonground/lidar_11_201')
        self.declare_parameter('distance_threshold', 0.15)
        self.declare_parameter('num_iterations', 100)

        self.input_topic = self.get_parameter('input_topic').value
        self.ground_topic = self.get_parameter('ground_topic').value
        self.nonground_topic = self.get_parameter('nonground_topic').value
        self.distance_threshold = float(self.get_parameter('distance_threshold').value)
        self.num_iterations = int(self.get_parameter('num_iterations').value)

        # qos = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )
        qos = 1

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            qos
        )

        self.ground_pub = self.create_publisher(PointCloud2, self.ground_topic, qos)
        self.nonground_pub = self.create_publisher(PointCloud2, self.nonground_topic, qos)

        self.get_logger().info('Open3D RANSAC ground node started')

    def callback(self, msg):
        field_names = [f.name for f in msg.fields]
        has_intensity = 'intensity' in field_names

        points = []

        if has_intensity:
            for p in pc2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=True
            ):
                points.append([p[0], p[1], p[2], p[3]])
        else:
            for p in pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            ):
                points.append([p[0], p[1], p[2], 0.0])

        if len(points) < 3:
            self.publish_cloud(msg.header, [], self.ground_pub)
            self.publish_cloud(msg.header, [], self.nonground_pub)
            return

        points = np.asarray(points, dtype=np.float32)

        xyz = points[:, :3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=3,
            num_iterations=self.num_iterations
        )

        inliers = np.asarray(inliers, dtype=np.int32)

        ground_mask = np.zeros(points.shape[0], dtype=bool)
        ground_mask[inliers] = True

        ground_points = points[ground_mask]
        nonground_points = points[~ground_mask]

        self.publish_cloud(msg.header, ground_points, self.ground_pub)
        self.publish_cloud(msg.header, nonground_points, self.nonground_pub)

        self.get_logger().info(
            f'total={len(points)}, ground={len(ground_points)}, nonground={len(nonground_points)}'
        )

    def publish_cloud(self, header, points, publisher):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        if isinstance(points, np.ndarray):
            cloud_data = points.astype(np.float32).tolist()
        else:
            cloud_data = points

        cloud_msg = pc2.create_cloud(header, fields, cloud_data)
        publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Open3DRansacGroundNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()