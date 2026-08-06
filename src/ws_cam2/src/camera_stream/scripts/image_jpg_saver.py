#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # Parameters
        self.declare_parameter('topic_name', '/image_raw')
        self.declare_parameter('save_interval', 2.0)  # seconds
        self.declare_parameter('save_directory', '.')
        
        topic_name = self.get_parameter('topic_name').value
        save_interval = self.get_parameter('save_interval').value
        self.save_directory = self.get_parameter('save_directory').value
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
            self.get_logger().info(f'Created directory: {self.save_directory}')
        
        # Initialize
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = False
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        
        # Timer for periodic saving
        self.timer = self.create_timer(save_interval, self.timer_callback)
        
        self.get_logger().info(f'Image Saver Node started')
        self.get_logger().info(f'Subscribing to: {topic_name}')
        self.get_logger().info(f'Save interval: {save_interval} seconds')
        self.get_logger().info(f'Save directory: {self.save_directory}')
    
    def image_callback(self, msg):
        """Store the latest image"""
        if not self.image_lock:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {str(e)}')
    
    def timer_callback(self):
        """Save image periodically"""
        if self.latest_image is None:
            self.get_logger().warn('No image received yet')
            return
        
        self.image_lock = True
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # milliseconds
            filename = f'{timestamp}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            
            # Save image
            cv2.imwrite(filepath, self.latest_image)
            self.get_logger().info(f'Saved: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {str(e)}')
        
        finally:
            self.image_lock = False
    
    def destroy_node(self):
        self.get_logger().info('Shutting down Image Saver Node')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageSaverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()