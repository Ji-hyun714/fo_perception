#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

class QoSRelayNode(Node):
    def __init__(self):
        super().__init__('qos_relay_node')
        
        # Input: RELIABLE (rosbag)
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Output: BEST_EFFORT (downstream)
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.sub = self.create_subscription(
            Image,
            '/image_rect',
            self.callback,
            qos_reliable
        )
        
        self.pub = self.create_publisher(
            Image,
            '/image_rect_be',
            qos_best_effort
        )
        
        self.get_logger().info('QoS Relay: RELIABLE → BEST_EFFORT')
    
    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()