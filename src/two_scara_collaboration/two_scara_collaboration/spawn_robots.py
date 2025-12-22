#!/usr/bin/env python3
"""
Robot Spawner Node - ROS2
Spawns SCARA robots and conveyor belt into Gazebo simulation
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import xacro
import os

class RobotSpawner(Node):
    """
    Spawns robots and environment models into Gazebo
    """
    
    def __init__(self):
        super().__init__('robot_spawner')
        
        # Declare profile parameter
        self.declare_parameter('profile', 'default')
        self.profile = self.get_parameter('profile').get_parameter_value().string_value
        
        # Set thickness based on profile
        self.thickness = "0.1"
        if self.profile == 'slim': self.thickness = "0.05"
        elif self.profile == 'heavy': self.thickness = "0.15"
        
        # Get package path
        from ament_index_python.packages import get_package_share_directory
        self.pkg_dir = get_package_share_directory('two_scara_collaboration')
        self.urdf_dir = os.path.join(self.pkg_dir, 'urdf')
        
        # Create spawn client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')
        
        self.get_logger().info(f'Robot Spawner node started with profile: {self.profile}')
        
        # Spawn robots on initialization
        self.spawn_all_robots()
    
    def load_urdf(self, xacro_file):
        """
        Load and process XACRO file to URDF
        """
        try:
            # Process XACRO file with thickness mapping
            doc = xacro.process_file(xacro_file, mappings={'thickness': self.thickness, 'robot_thickness': self.thickness})
            urdf_str = doc.toxml()
            return urdf_str
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
            return None
    
    def spawn_robot(self, name, xacro_file, namespace, position=(0, 0, 0)):
        """
        Spawn a single robot
        
        Args:
            name: Robot name
            xacro_file: Path to XACRO file
            namespace: Robot namespace
            position: (x, y, z) spawn position
        """
        # Load URDF
        urdf_content = self.load_urdf(xacro_file)
        if urdf_content is None:
            self.get_logger().error(f'Failed to spawn {name}')
            return False
        
        # Create pose
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        
        # Create spawn request
        request = SpawnEntity.Request()
        request.name = name
        request.xml = urdf_content
        request.robot_namespace = namespace
        request.initial_pose = pose
        request.reference_frame = 'world'
        
        # Call spawn service
        future = self.spawn_client.call_async(request)
        
        # Wait for result
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Successfully spawned {name}')
                return True
            else:
                self.get_logger().error(f'Failed to spawn {name}: {result.status_message}')
                return False
        except Exception as e:
            self.get_logger().error(f'Spawn service call failed: {e}')
            return False
    
    def spawn_conveyor(self):
        """Spawn conveyor belt"""
        xacro_file = os.path.join(self.urdf_dir, 'conveyor_belt.urdf.xacro')
        pose = (0.0, 0.0, 0.0)
        self.spawn_robot('conveyor_belt', xacro_file, '/conveyor', pose)
    
    def spawn_scara_left(self):
        """Spawn left SCARA robot"""
        xacro_file = os.path.join(self.urdf_dir, 'scara_left.urdf.xacro')
        pose = (0.3, 0.5, 0.0)  # Position on left side
        self.spawn_robot('scara_left', xacro_file, '/scara_left', pose)
    
    def spawn_scara_right(self):
        """Spawn right SCARA robot"""
        xacro_file = os.path.join(self.urdf_dir, 'scara_right.urdf.xacro')
        pose = (-0.3, 0.5, 0.0)  # Position on right side
        self.spawn_robot('scara_right', xacro_file, '/scara_right', pose)
    
    def spawn_all_robots(self):
        """Spawn all robots and environment"""
        self.get_logger().info('Starting robot spawning...')
        
        # Spawn in order
        self.spawn_conveyor()
        self.spawn_scara_left()
        self.spawn_scara_right()
        
        self.get_logger().info('All robots spawned successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = RobotSpawner()
    
    # Keep node running briefly to ensure spawning completes
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
