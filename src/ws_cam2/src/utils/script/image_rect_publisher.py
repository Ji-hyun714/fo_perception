#!/usr/bin/env python3

import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageRectPublisher(Node):
    def __init__(self) -> None:
        super().__init__("image_rect_publisher")

        self.declare_parameter("image_path", "/home/wise/Downloads/guvcview_image-4.jpg")
        self.declare_parameter("topic_name", "/camera/image_rect")
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("frame_id", "camera")

        image_path = self.get_parameter("image_path").get_parameter_value().string_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        if not image_path:
            self.get_logger().fatal("Parameter 'image_path' is empty.")
            raise RuntimeError("image_path is required")

        if not os.path.isfile(image_path):
            self.get_logger().fatal(f"Image file does not exist: {image_path}")
            raise RuntimeError("image_path does not exist")

        self.cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if self.cv_image is None:
            self.get_logger().fatal(f"Failed to read image: {image_path}")
            raise RuntimeError("cannot read image")

        if publish_rate_hz <= 0.0:
            self.get_logger().warn(
                f"Invalid publish_rate_hz={publish_rate_hz}, using 25.0"
            )
            publish_rate_hz = 25.0

        self.encoding = self._infer_encoding(self.cv_image)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_once)

        self.get_logger().info(
            f"Publishing '{image_path}' to '{topic_name}' at {publish_rate_hz:.2f} Hz"
        )

    @staticmethod
    def _infer_encoding(cv_image) -> str:
        if len(cv_image.shape) == 2:
            return "mono8"
        channels = cv_image.shape[2]
        if channels == 3:
            return "bgr8"
        if channels == 4:
            return "bgra8"
        return "passthrough"

    def _publish_once(self) -> None:
        msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding=self.encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImageRectPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
