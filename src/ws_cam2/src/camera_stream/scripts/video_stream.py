#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # video_path = "/media/wise/8908794e-faf6-421c-8a31-a72e82a370e0/test_720_clipped.mp4"
        # video_path = "/media/wise/sjy3/kcity_driving_720p.mp4"
        # file = "/home/wise/Downloads/test_720_clipped.mp4"
        file = "/home/wise/Downloads/g70_kcity_lane_test.mp4"
        pub_topic_name = "/camera/image_rect"

        self.declare_parameter('video_path', file)
        video_path = self.get_parameter('video_path').value

        # self.cap = cv2.VideoCapture('/home/wise/_examples/driving_test.mp4')  # Replace with your video path
        # self.cap = cv2.VideoCapture('/home/wise/Documents/yolonas-naseemap47/250110.mp4')  # Replace with your video path
        self.cap = cv2.VideoCapture(video_path) 
        self.br = CvBridge()
        self.get_logger().info(f"Video publisher started for {video_path}")

        # fps = self.cap.get(cv2.CAP_PROP_FPS)
        # if fps <= 0:
        #     fps = 25.0  # Default fallback
        #     self.get_logger().warn(f"Could not get FPS from video, using default: {fps}")
        
        # self.get_logger().info(f"Video FPS: {fps}")
        
        self.publisher_ = self.create_publisher(Image, pub_topic_name, 10)
        # self.timer = self.create_timer(0.045, self.timer_callback)  # ~30 FPS
        self.timer = self.create_timer(1.0 / 25.0, self.timer_callback)

        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # If video ends, reset to the beginning
            self.get_logger().info('Video has ended. Looping back to the start.')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()

        if ret:
            # Convert the frame to ROS Image message
            ros_image = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(ros_image)
            # self.get_logger().info('Publishing video frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
