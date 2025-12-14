#!/usr/bin/env python3
"""
Gripper Action Server - ROS2
Handles gripper control for both SCARA robots
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray
import time

class GripperActionServer(Node):
    """
    Action server for gripper control
    """
    
    def __init__(self):
        super().__init__('gripper_action_server')
        
        # Action servers for each robot
        self.left_gripper_server = ActionServer(
            self,
            GripperCommand,
            'scara_left/gripper_command',
            self.execute_left_gripper
        )
        
        self.right_gripper_server = ActionServer(
            self,
            GripperCommand,
            'scara_right/gripper_command',
            self.execute_right_gripper
        )
        
        # Publishers for gripper commands
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/scara_left_gripper_controller/commands',
            10
        )
        
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/scara_right_gripper_controller/commands',
            10
        )
        
        # Gripper parameters
        self.max_effort = 100.0
        self.min_gap = 0.0
        self.max_gap = 0.05
        
        self.get_logger().info('Gripper Action Server initialized')
    
    async def execute_left_gripper(self, goal_handle):
        """Execute left gripper command"""
        self.get_logger().info(f'Left gripper command received: {goal_handle.request.command.position}')
        
        # Command gripper
        command_msg = Float64MultiArray()
        position = goal_handle.request.command.position
        
        # Clamp position to valid range
        position = max(self.min_gap, min(self.max_gap, position))
        command_msg.data = [position]
        
        self.left_gripper_pub.publish(command_msg)
        
        # Simulate execution time
        await self.move_gripper(goal_handle, 'left')
    
    async def execute_right_gripper(self, goal_handle):
        """Execute right gripper command"""
        self.get_logger().info(f'Right gripper command received: {goal_handle.request.command.position}')
        
        # Command gripper
        command_msg = Float64MultiArray()
        position = goal_handle.request.command.position
        
        # Clamp position to valid range
        position = max(self.min_gap, min(self.max_gap, position))
        command_msg.data = [position]
        
        self.right_gripper_pub.publish(command_msg)
        
        # Simulate execution time
        await self.move_gripper(goal_handle, 'right')
    
    async def move_gripper(self, goal_handle, side):
        """Simulate gripper movement"""
        try:
            # Wait for gripper to reach position
            for i in range(10):  # 1 second total
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f'{side} gripper cancelled')
                    goal_handle.canceled()
                    return
                
                time.sleep(0.1)
            
            # Success
            self.get_logger().info(f'{side} gripper reached position')
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f'Gripper error: {e}')
            goal_handle.abort()

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
