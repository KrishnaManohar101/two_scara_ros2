# Two SCARA Robots Collaboration Project

## Overview
This project simulates a collaborative dual-arm SCARA robot system in Gazebo using ROS 2 (Humble). It includes custom kinematics, dynamics, trajectory planning, and collision avoidance logic.

## Key Components

### 1. Simulation & Control
*   **`launch/gazebo.launch.py`**: Main launch file. Starts Gazebo, spawns robots, loads controllers, and starts motion planners.
*   **`src/two_scara_collaboration/`**: Main ROS 2 package containing:
    *   **`kinematics.py`**: Core class for Forward/Inverse Kinematics, Dynamics (Torque/Mass Matrix), and Trajectory Generation.
    *   **`collision_avoidance.py`**: Logic to detect and prevent collisions between the two arms.
    *   **`scara_left_motion_planner.py`**: Loop-based planner for the left arm.
    *   **`gripper_action_server.py`**: Service to control the vacuum grippers.

### 2. Interactive Scripts (Root Directory)
*   **`direct_joint_control.py`**: 
    *   **Purpose**: Manually set Joint 1, Joint 2, and Z positions for either robot.
    *   **Features**: Validates inputs against joint limits and verifies with Forward Kinematics.
    *   **Usage**: `python3 direct_joint_control.py`

*   **`torque_control_sim.py`**:
    *   **Purpose**: Simulate Forward Dynamics (Torque $\to$ Motion).
    *   **Features**: Input Joint Torques and Duration, output resulting position.
    *   **Usage**: `python3 torque_control_sim.py`

*   **`keyboard_joint_control.py`**:
    *   **Purpose**: Teleoperate the robot using keyboard arrow keys.
    *   **Usage**: `python3 keyboard_joint_control.py`

*   **`live_kinematics_monitor.py`**:
    *   **Purpose**: Real-time dashboard showing current End-Effector positions and velocities.

### 3. Documentation
*   **`KINEMATICS_EXPLANATION.md`**: Detailed breakdown of the math used (DH Parameters, Dynamics Matrices).
*   **`Project_Report.md`**: Full project report.

## Getting Started

### Prerequisites
*   ROS 2 Humble
*   Gazebo
*   `gazebo_ros_pkgs`
*   `ros2_control`, `gazebo_ros2_control`

### Build
```bash
cd ~/two_scara_ros2
colcon build
source install/setup.bash
```

### Run Simulation
```bash
ros2 launch two_scara_collaboration gazebo.launch.py
```

### Run Custom Scripts
Open a new terminal, source the workspace, and run:
```bash
# Direct Angle Control
python3 direct_joint_control.py

# Forward Dynamics Sim
python3 torque_control_sim.py
```

## Physics Notes
*   **Gravity**: 9.81 m/s² in -Z.
*   **SCARA Plane**: X-Y plane is horizontal (no gravity component on J1/J2).
*   **Dynamics Model**: Simplified Lagrangian dynamics included in `kinematics.py`.
