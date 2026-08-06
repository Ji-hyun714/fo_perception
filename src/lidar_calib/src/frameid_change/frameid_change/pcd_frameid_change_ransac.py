import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d


class PointCloudFrameChange(Node):

    def __init__(self):
        super().__init__('pcd_frameid_change')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('new_frame_id', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.new_frame_id = self.get_parameter('new_frame_id').value

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )

        self.get_logger().info(
            f'RANSAC Ground Removal: {input_topic} -> {output_topic}'
        )

    def callback(self, msg):

        # 1️⃣ XYZI 모두 읽기
        raw_points = np.array(list(
            point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True
            )
        ))

        if len(raw_points) < 50:
            return

        # 2️⃣ RANSAC은 xyz만 사용
        xyz = np.vstack((
            raw_points['x'],
            raw_points['y'],
            raw_points['z']
        )).T.astype(np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.33,
            ransac_n=3,
            num_iterations=100
        )

        if len(inliers) == 0:
            return

        # 3️⃣ inlier 제외 (negative=True 동일)
        mask = np.ones(len(raw_points), dtype=bool)
        mask[inliers] = False

        filtered_points = raw_points[mask]

        if len(filtered_points) == 0:
            return

        # 4️⃣ XYZI 필드 정의
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # 5️⃣ PointCloud2 재생성
        output_msg = point_cloud2.create_cloud(
            msg.header,
            fields,
            filtered_points
        )

        output_msg.header.frame_id = self.new_frame_id
        output_msg.header.stamp = msg.header.stamp

        self.pub.publish(output_msg)


def main():
    rclpy.init()
    node = PointCloudFrameChange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
