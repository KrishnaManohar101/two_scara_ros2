# Two SCARA Robot Collaboration System

## Executive Summary
This project implements a comprehensive simulation of a dual-SCARA robot system designed for collaborative sorting and manipulation tasks on a conveyor belt. It leverages **ROS 2** and **Gazebo** to provide a realistic environment for testing kinematic modeling, dynamic analysis, collision avoidance, and synchronized control strategies.

For a deep dive into the mathematical and physical foundations (Kinematics, Dynamics, Singularity Analysis), please refer to the [Technical Documentation](Description_and_Physics.md).

## Features
- **Dual Robot Simulation**: Two SCARA robots operating in a shared workspace with a conveyor belt.
- **Advanced Kinematics**:
  - **Forward Kinematics (FK)**: Standard Denavit-Hartenberg implementation.
  - **Inverse Kinematics (IK)**: Analytical solution with "Elbow Up" configuration and singularity handling.
- **Collaborative Control**:
  - Symmetric replication logic for mirrored tasks.
  - Shared workspace management to prevent collisions.
- **Physics-Based Simulation**:
  - Realistic inertia and mass properties.
  - Friction and gravity modeling (where applicable).
- **ROS 2 Integration**:
  - `ros2_control` for joint trajectory execution.
  - Custom nodes for motion planning (`scara_left_motion_planner`, `scara_right_motion_planner`).
  - Gripper action servers and block spawning logic.

## System Architecture

### Robot Configuration
- **Model**: SCARA (Selective Compliance Assembly Robot Arm)
- **Degrees of Freedom**: 3 (2 Revolute + 1 Prismatic/Gripper per robot in this simulation context)
- **Workspace**: Toroidal shape defined by $R_{in}=0.5m$ and $R_{out}=2.5m$.
- **Placement**:
  - **Left Robot**: World $(2.0, -2.0, 0.2)$
  - **Right Robot**: World $(2.0, 2.0, 0.2)$

### Software Stack
- **Framework**: ROS 2 (Humble/Iron compatible)
- **Simulation**: Gazebo Classic
- **Visualization**: RViz2
- **Controllers**: `joint_state_broadcaster`, `forward_command_controller` / `joint_trajectory_controller`

## Installation

### Prerequisites
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Python 3.10+
- `numpy`

### Dependencies
Install the required ROS 2 packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-gazebo-ros2-control \
ros-humble-xacro
```

### Build Instructions
1.  **Create a Workspace** (if you haven't already):
    ```bash
    mkdir -p ~/two_scara_ws/src
    cd ~/two_scara_ws/src
    ```
2.  **Clone the Repository**:
    ```bash
    git clone https://github.com/KrishnaManohar101/two_scara_ros2.git
    ```
    *(Note: Ensure this folder is named `two_scara_collaboration` or placed correctly inside `src`)*

3.  **Build the Package**:
    ```bash
    cd ~/two_scara_ws
    colcon build --symlink-install
    ```

4.  **Source the Environment**:
    ```bash
    source install/setup.bash
    ```

## Usage

### Launching the Simulation
To start the Gazebo simulation with both robots, controllers, and the layout:
```bash
ros2 launch two_scara_ros2 gazebo.launch.py
```
This will:
- Open Gazebo.
- Spawn the `dual_scara_world`.
- Start the `robot_state_publisher`.
- Launch the MoveIt planners (if configured) or custom motion planner nodes.
- Spawn the blocks for sorting.

### Interactive Control
The system includes motion planner nodes that listen for targets. You can publish target coordinates or interacting via the provided nodes:

**Left Robot Planner**:
```bash
ros2 run two_scara_ros2 scara_left_motion_planner.py
```

**Right Robot Planner**:
```bash
ros2 run two_scara_ros2 scara_right_motion_planner.py
```

### Interactive Control Scripts
We provide standalone Python scripts for testing specific robot capabilities without running the full ROS planner stack.

#### 1. Direct Joint Control (Kinematics Verification)
Manually set joint angles and verify Forward Kinematics.
```bash
python3 direct_joint_control.py
```
*   **Input**: Joint 1 (deg), Joint 2 (deg), Z-Axis (m).
*   **Features**: Validates inputs against joint limits and verifies end-effector position.

#### 2. Forward Dynamics Simulator (Torque Control)
Simulate robot motion under applied torque/force over time.
```bash
python3 torque_control_sim.py
```
*   **Input**: Torque J1 (Nm), Torque J2 (Nm), Force Z (N), Duration (s).
*   **Features**: Uses Lagrangian dynamics (Inverse Mass Matrix) to calculate acceleration and integrate position.

#### 3. Keyboard Teleoperation
Control the robot joints using arrow keys.
```bash
python3 keyboard_joint_control.py
```

### Motion Planners (ROS 2)
The system includes motion planner nodes that listen for targets:
```bash
ros2 run two_scara_ros2 scara_left_motion_planner.py
ros2 run two_scara_ros2 scara_right_motion_planner.py
```
Safety margins are applied (e.g., 99% max reach) to prevent locking.

## 📖 Research and Documentation
For a comprehensive academic and technical deep dive, including all mathematical derivations (Lagrangian Dynamics, Trajectory Optimization via Lagrange Multipliers), please refer to:

👉 **[RESEARCH_PAPER_FULL.md](RESEARCH_PAPER_FULL.md)**

### Mapping Paper to Code
This project bridges the gap between advanced mathematical theory and robotics implementation. Below is a map of where paper concepts reside in the source code:

| Paper Section | Topic | Source File / Implementation |
| :--- | :--- | :--- |
| **Chapter 3** | Forward & Inverse Kinematics | `kinematics.py` -> `SCARAKinematics.forward_kinematics`, `inverse_kinematics` |
| **Chapter 4** | Jacobian & Singularities | `kinematics.py` -> `SCARAKinematics.jacobian` |
| **Chapter 5** | Lagrangian Dynamics | `kinematics.py` -> `SCARAKinematics.calculate_dynamics` |
| **Chapter 6** | Mirroring & Collaboration | `scara_right_motion_planner.py` (Replication Logic) |
| **Chapter 7** | Lagrange Multiplier Optimization | `kinematics.py` -> `generate_straight_line_path` |
| **Chapter 8** | Simulation & Launch | `launch/gazebo.launch.py`, `urdf/scara_left.urdf.xacro` |
| **Chapter 9** | Results & Data Logging | `replication_data.csv` (Logged simulation output) |

## License
This project is licensed under the Apache License 2.0.
