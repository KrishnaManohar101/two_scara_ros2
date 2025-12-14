#!/usr/bin/env python3
"""
Block Publisher Node - ROS2
Publishes positions of blocks on conveyor belt
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np

class BlockPublisher(Node):
    """
    Publishes block positions from Gazebo
    """
    
    def __init__(self):
        super().__init__('block_publisher')
        
        # Publishers
        self.block_positions_pub = self.create_publisher(
            Point,
            '/conveyor/block_positions',
            10
        )
        
        # Timer to publish positions periodically
        self.create_timer(0.5, self.publish_block_positions)
        
        # Store block positions
        self.block_positions = []
        
        self.get_logger().info('Block Publisher initialized')
    
    def publish_block_positions(self):
        """
        Publish current block positions
        Would integrate with Gazebo ModelState in actual implementation
        """
        # Simulated block positions
        # In real implementation, get from Gazebo ModelState subscriber
        for i, pos in enumerate(self.block_positions):
            msg = Point()
            msg.x = pos[0]
            msg.y = pos[1]
            msg.z = pos[2]
            
            self.block_positions_pub.publish(msg)
    
    def add_block_position(self, x, y, z):
        """Add block position to tracking"""
        self.block_positions.append([x, y, z])

def main(args=None):
    rclpy.init(args=args)
    node = BlockPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
