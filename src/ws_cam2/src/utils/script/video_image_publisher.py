#!/usr/bin/env python3

import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoImagePublisher(Node):
    def __init__(self) -> None:
        super().__init__("video_image_publisher")

        self.declare_parameter("video_path", "")
        self.declare_parameter("topic_name", "/camera/image_raw")
        self.declare_parameter("framerate", 30.0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)

        video_path = self.get_parameter("video_path").get_parameter_value().string_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        framerate = self.get_parameter("framerate").get_parameter_value().double_value
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value

        if not video_path:
            self.get_logger().fatal("Parameter 'video_path' is empty.")
            raise RuntimeError("video_path is required")

        if not os.path.isfile(video_path):
            self.get_logger().fatal(f"Video file does not exist: {video_path}")
            raise RuntimeError("video_path does not exist")

        if framerate <= 0.0:
            self.get_logger().fatal(f"Invalid framerate: {framerate}")
            raise RuntimeError("framerate must be greater than zero")

        if self.width <= 0 or self.height <= 0:
            self.get_logger().fatal(
                f"Invalid output size: width={self.width}, height={self.height}"
            )
            raise RuntimeError("width and height must be greater than zero")

        self.capture = cv2.VideoCapture(video_path)
        if not self.capture.isOpened():
            self.get_logger().fatal(f"Failed to open video: {video_path}")
            raise RuntimeError("cannot open video")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(1.0 / framerate, self._publish_frame)

        source_fps = self.capture.get(cv2.CAP_PROP_FPS)
        source_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        source_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.get_logger().info(
            "Publishing video '%s' to '%s' at %.2f Hz (source: %dx%d @ %.2f fps, output: %dx%d)"
            % (
                video_path,
                topic_name,
                framerate,
                source_width,
                source_height,
                source_fps,
                self.width,
                self.height,
            )
        )

    def _publish_frame(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().info("Reached end of video. Stopping publisher.")
            self._stop()
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.publisher.publish(msg)

    def _stop(self) -> None:
        if hasattr(self, "timer") and self.timer is not None:
            self.timer.cancel()
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
            self.capture = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VideoImagePublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
