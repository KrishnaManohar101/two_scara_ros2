import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

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

    # Gazebo Server
    gz_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo Client
    gz_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher (Global)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )
    
    # Spawn Entity (World) - Delayed to wait for Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'dual_scara_world', '-timeout', '120'],
        output='screen',
    )
    
    # Wrap spawn in timer
    delayed_spawn_robot = TimerAction(
        period=10.0,
        actions=[spawn_robot]
    )
    
    # Block Spawner Node - Delayed further
    block_spawner = Node(
        package='two_scara_collaboration',
        executable='block_spawner.py',
        name='block_spawner',
        output='screen',
    )
    
    delayed_block_spawner = TimerAction(
        period=15.0,
        actions=[block_spawner]
    )
    
    # Block Publisher Node
    block_publisher = Node(
        package='two_scara_collaboration',
        executable='block_publisher.py',
        name='block_publisher',
        output='screen',
    )

    # Motion Planners
    scara_left_planner = Node(
        package='two_scara_collaboration',
        executable='scara_left_motion_planner.py',
        name='scara_left_motion_planner',
        output='screen',
    )
    scara_right_planner = Node(
        package='two_scara_collaboration',
        executable='scara_right_motion_planner.py',
        name='scara_right_motion_planner',
        output='screen',
    )

    # Gripper Server
    gripper_server = Node(
        package='two_scara_collaboration',
        executable='gripper_action_server.py',
        name='gripper_action_server',
        output='screen',
    )

    # Controllers (Global Manager) - Spawn AFTER Robot
    # We can use RegisterEventHandler to spawn controllers after spawn_entity exits
    
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    load_left_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_left_arm_controller'],
        output='screen',
    )
    
    load_left_grip = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_left_gripper_controller'],
        output='screen',
    )
    
    load_right_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_right_arm_controller'],
        output='screen',
    )
    
    load_right_grip = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_right_gripper_controller'],
        output='screen',
    )
    
    # Group controllers
    delayed_controllers = TimerAction(
        period=20.0,
        actions=[load_jsb, load_left_arm, load_left_grip, load_right_arm, load_right_grip]
    )

    # Conveyor Driver
    conveyor_driver = Node(
        package='two_scara_collaboration',
        executable='conveyor_driver',
        name='conveyor_driver',
        output='screen',
    )

    return LaunchDescription([
        gz_server,
        gz_client,
        rsp,
        delayed_spawn_robot,
        delayed_block_spawner,
        block_publisher,
        # conveyor_driver, # Run manually: python3 src/.../conveyor_driver.py
        scara_left_planner,
        scara_right_planner,
        gripper_server,
        delayed_controllers
    ])
