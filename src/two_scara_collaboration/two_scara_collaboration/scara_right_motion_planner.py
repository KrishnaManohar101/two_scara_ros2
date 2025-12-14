#!/usr/bin/env python3
"""
SCARA Right Motion Planner - ROS2 Node
Handles planning and execution for right SCARA robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
from two_scara_collaboration.kinematics import SCARAKinematics
from two_scara_collaboration.collision_avoidance import CollisionAvoidanceController
import time

class SCARARightMotionPlanner(Node):
    """
    Right SCARA Robot Motion Planner
    """
    
    def __init__(self):
        super().__init__('scara_right_motion_planner')
        
        # SCARA Geometry
        self.kinematics = SCARAKinematics(l1=1.5, l2=1.0)
        self.collision_avoidance = CollisionAvoidanceController(
            left_base=(0.3, 0),
            right_base=(-0.3, 0)
        )
        
        # Robot state
        self.current_joint_state = [0.0, 0.0]
        self.target_position = None
        
        # Parameters
        self.joint_names = [
            'scara_right_joint_1',
            'scara_right_joint_2'
        ]
        
        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/scara_right_arm_controller/commands',
            10
        )
        
        # Subscribers
        # Subscribers
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/scara_right/target_pose',
            self.target_pose_callback,
            10
        )
        
        # Timer
        self.create_timer(0.1, self.motion_planning_loop)
        
        self.get_logger().info('SCARA Right Motion Planner initialized')
    
    def joint_state_callback(self, msg):
        """Callback for joint state"""
        try:
            idx1 = msg.name.index(self.joint_names[0])
            idx2 = msg.name.index(self.joint_names[1])
            self.current_joint_state = [msg.position[idx1], msg.position[idx2]]
        except ValueError:
            pass
    
    def target_pose_callback(self, msg):
        """Callback for target pose"""
        self.target_position = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f'Right target: {self.target_position}')
    
    def motion_planning_loop(self):
        """Main planning loop"""
        if self.target_position is None:
            return
        
        # Inverse kinematics
        theta1, theta2 = self.kinematics.inverse_kinematics(
            self.target_position[0],
            self.target_position[1],
            elbow='up'
        )
        
        if theta1 is None:
            self.get_logger().warn('Right robot: Target unreachable')
            return
        
        # Linear interpolation to target
        target_joints = [theta1, theta2]
        alpha = 0.1  # Smoothing factor
        
        smoothed_joints = [
            self.current_joint_state[i] + alpha * (target_joints[i] - self.current_joint_state[i])
            for i in range(2)
        ]
        
        # Publish command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = smoothed_joints
        self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SCARARightMotionPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
