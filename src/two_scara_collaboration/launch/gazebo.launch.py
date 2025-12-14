import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('two_scara_collaboration')
    
    # Path to World URDF
    world_urdf = os.path.join(pkg_share, 'urdf', 'world.urdf.xacro')

    # Process XACRO
    import xacro
    doc = xacro.process_file(world_urdf)
    robot_desc = doc.toxml()
    
    # Sim Time
    use_sim_time = {'use_sim_time': True}

    launch_description_content = [
        # Gazebo Server
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Gazebo Client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Robot State Publisher (Global)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        ),
        
        # Spawn Entity (World)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'dual_scara_world'],
            output='screen',
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

        # Motion Planners
        Node(
            package='two_scara_collaboration',
            executable='scara_left_motion_planner.py',
            name='scara_left_motion_planner',
            output='screen',
        ),
        Node(
            package='two_scara_collaboration',
            executable='scara_right_motion_planner.py',
            name='scara_right_motion_planner',
            output='screen',
        ),

        # Gripper Server
        Node(
            package='two_scara_collaboration',
            executable='gripper_action_server.py',
            name='gripper_action_server',
            output='screen',
        ),

        # Controllers (Global Manager)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scara_left_arm_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scara_left_gripper_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scara_right_arm_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scara_right_gripper_controller'],
            output='screen',
        ),
    ]
    
    return LaunchDescription(launch_description_content)
