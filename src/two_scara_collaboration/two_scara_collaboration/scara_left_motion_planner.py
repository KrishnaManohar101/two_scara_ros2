#!/usr/bin/env python3
"""
SCARA Left Motion Planner - ROS2 Node
Handles planning and execution for left SCARA robot
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
from two_scara_collaboration.kinematics import SCARAKinematics, CollisionDetector
from two_scara_collaboration.collision_avoidance import CollisionAvoidanceController
import time

class SCARALeftMotionPlanner(Node):
    """
    Left SCARA Robot Motion Planner
    """
    
    def __init__(self):
        super().__init__('scara_left_motion_planner')
        
        # Kinematics        # SCARA Geometry
        # L1 = 1.6m visual length, Joint at 0.0 -> Joint at 1.5. So L1=1.5
        # L2 = 1.0m visual, Joint at 1.0. So L2=1.0.
        self.kinematics = SCARAKinematics(l1=1.5, l2=1.0)
        self.collision_detector = CollisionDetector(l1=0.5, l2=0.5)
        self.collision_avoidance = CollisionAvoidanceController(
            left_base=(0.3, 0),
            right_base=(-0.3, 0)
        )
        
        # Robot state
        self.current_joint_state = [0.0, 0.0]
        self.target_position = None
        self.is_moving = False
        
        # Parameters
        self.joint_names = [
            'scara_left_joint_1',
            'scara_left_joint_2'
        ]
        self.max_joint_speed = [2.0, 2.0]  # rad/s
        self.max_acceleration = [1.0, 1.0]  # rad/s^2
        
        # Publishers
        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/scara_left_arm_controller/commands',
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
            '/scara_left/target_pose',
            self.target_pose_callback,
            10
        )
        
        # Timer for motion planning loop
        self.create_timer(0.1, self.motion_planning_loop)
        
        self.get_logger().info('SCARA Left Motion Planner initialized')
    
    def joint_state_callback(self, msg):
        """Callback for current joint state feedback"""
        try:
            idx1 = msg.name.index(self.joint_names[0])
            idx2 = msg.name.index(self.joint_names[1])
            self.current_joint_state = [msg.position[idx1], msg.position[idx2]]
        except ValueError:
            pass
    
    def target_pose_callback(self, msg):
        """Callback for target pose command"""
        self.target_position = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f'Target position received: {self.target_position}')
    
    def motion_planning_loop(self):
        """Main motion planning loop"""
        if self.target_position is None:
            return
        
        # Calculate inverse kinematics
        theta1, theta2 = self.kinematics.inverse_kinematics(
            self.target_position[0],
            self.target_position[1],
            elbow='up'
        )
        
        if theta1 is None:
            self.get_logger().warn('Target position unreachable')
            return
        
        target_joints = [theta1, theta2]
        
        # Plan trajectory
        trajectory = self.plan_trajectory(
            self.current_joint_state,
            target_joints
        )
        
        if trajectory is not None:
            # Execute trajectory
            self.execute_trajectory(trajectory)
            self.target_position = None
    
    def plan_trajectory(self, start_joints, target_joints, duration=2.0):
        """
        Plan linear trajectory in joint space
        
        Args:
            start_joints: Starting joint angles
            target_joints: Target joint angles
            duration: Trajectory duration (seconds)
            
        Returns:
            List of joint trajectory points
        """
        num_points = int(duration / 0.1)
        trajectory = []
        
        for i in range(num_points + 1):
            t = i / num_points
            
            # Linear interpolation
            joints = [
                start_joints[j] + t * (target_joints[j] - start_joints[j])
                for j in range(2)
            ]
            
            # Calculate velocities
            velocities = [
                (target_joints[j] - start_joints[j]) / duration
                for j in range(2)
            ]
            
            point = JointTrajectoryPoint()
            point.positions = joints
            point.velocities = velocities
            point.time_from_start.sec = int(t * duration)
            point.time_from_start.nanosec = int((t * duration % 1.0) * 1e9)
            
            trajectory.append(point)
        
        return trajectory
    
    def execute_trajectory(self, trajectory):
        """Execute planned trajectory"""
        for point in trajectory:
            # Publish joint command
            cmd_msg = Float64MultiArray()
            cmd_msg.data = point.positions
            self.command_pub.publish(cmd_msg)
            
            # Small delay for execution
            time.sleep(0.1)
        
        self.get_logger().info('Trajectory execution complete')

def main(args=None):
    rclpy.init(args=args)
    node = SCARALeftMotionPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
