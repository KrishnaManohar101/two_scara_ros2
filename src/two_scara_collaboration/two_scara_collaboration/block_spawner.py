#!/usr/bin/env python3
"""
Block Spawner Node - ROS2
Spawns cylindrical blocks on the conveyor belt automatically
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import random
import time

class BlockSpawner(Node):
    """
    Spawns cylinder blocks on conveyor belt
    """
    
    def __init__(self):
        super().__init__('block_spawner')
        
        # Parameters
        self.spawn_interval = 2.0  # seconds
        self.spawn_count = 0
        self.max_blocks = 20
        
        # Conveyor parameters
        self.conveyor_x_min = -0.8
        self.conveyor_x_max = 0.8
        self.conveyor_y = 0.0
        self.conveyor_z = 0.15  # Height on conveyor
        
        # Create spawn client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Timer for spawning
        self.create_timer(self.spawn_interval, self.spawn_block)
        
        self.get_logger().info('Block Spawner initialized')
    
    def spawn_block(self):
        """Spawn a single cylinder block"""
        if self.spawn_count >= self.max_blocks:
            self.get_logger().info('Maximum blocks reached')
            return
        
        # Random position on conveyor
        x_pos = random.uniform(self.conveyor_x_min, self.conveyor_x_max)
        
        # Create pose
        pose = Pose()
        pose.position.x = x_pos
        pose.position.y = self.conveyor_y
        pose.position.z = self.conveyor_z
        pose.orientation.w = 1.0
        
        # Block URDF
        block_urdf = """
        <robot name="cylinder_block_{0}">
            <link name="cylinder_link">
                <visual>
                    <geometry>
                        <cylinder radius="0.05" length="0.1"/>
                    </geometry>
                    <material name="cyan">
                        <color rgba="0 1 1 1"/>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <cylinder radius="0.05" length="0.1"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
                </inertial>
            </link>
        </robot>
        """.format(self.spawn_count)
        
        # Spawn request
        request = SpawnEntity.Request()
        request.name = f'cylinder_block_{self.spawn_count}'
        request.xml = block_urdf
        request.robot_namespace = '/blocks'
        request.initial_pose = pose
        request.reference_frame = 'world'
        
        # Call spawn service
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        
        self.spawn_count += 1
    
    def spawn_callback(self, future):
        """Callback for spawn service"""
        try:
            result = future.result()
            self.get_logger().info(f'Block spawned: {result.success}')
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BlockSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
