#!/usr/bin/env python3
"""
ROS2 Launch file for Two SCARA Robots Collaboration Simulation
Launches Gazebo simulator with both robots and conveyor belt
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'two_scara_collaboration'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_dir = os.path.join(pkg_share, 'urdf')
    config_dir = os.path.join(pkg_share, 'config')
    
    # URDF file paths
    scara_left_urdf = os.path.join(urdf_dir, 'scara_left.urdf.xacro')
    scara_right_urdf = os.path.join(urdf_dir, 'scara_right.urdf.xacro')
    conveyor_urdf = os.path.join(urdf_dir, 'conveyor_belt.urdf.xacro')
    world_urdf = os.path.join(urdf_dir, 'world.urdf.xacro')
    
    # Simulation parameters
    use_sim_time = True
    
    launch_description_content = [
        # Gazebo Server with pause=true initially
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '--pause'  # Start paused
            ],
            output='screen'
        ),
        
        # Gazebo Client (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        
        # Load URDF parameters and spawn robots using node
        Node(
            package='two_scara_collaboration',
            executable='spawn_robots.py',
            name='robot_spawner',
            output='screen',
            parameters=[
                {'scara_left_urdf': scara_left_urdf},
                {'scara_right_urdf': scara_right_urdf},
                {'conveyor_urdf': conveyor_urdf},
            ]
        ),
        
        # Block Spawner Node
        Node(
            package='two_scara_collaboration',
            executable='block_spawner.py',
            name='block_spawner',
            output='screen',
        ),
        
        # Block Publisher Node
        Node(
            package='two_scara_collaboration',
            executable='block_publisher.py',
            name='block_publisher',
            output='screen',
        ),
    ]
    
    return LaunchDescription(launch_description_content)
