import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudFrameChange(Node):
    def __init__(self):
        super().__init__('pcd_frameid_change')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('new_frame_id', '')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.new_frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value

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
            f'Change started: {input_topic} -> {output_topic}, frame_id={self.new_frame_id}'
        )

    def callback(self, msg: PointCloud2):
        msg.header.frame_id = self.new_frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PointCloudFrameChange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
