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
    git clone https://github.com/yangliu28/two_scara_collaboration.git
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
ros2 launch two_scara_collaboration gazebo.launch.py
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
ros2 run two_scara_collaboration scara_left_motion_planner.py
```

**Right Robot Planner**:
```bash
ros2 run two_scara_collaboration scara_right_motion_planner.py
```

## Technical Highlights

### Coordinate Systems
- **World Frame**: Global reference at $(0,0,0)$.
- **Base Frames**: Local frames for each robot, offset from World.
- **Transformation**: $P_{world} = P_{local} + P_{base}$.

### Singularity Management
The system monitors the Jacobian determinant $\det(J) = L_1 L_2 \sin(\theta_2)$ to avoid:
- **Outer Singularity**: $\theta_2 = 0^\circ$ (Full extension)
- **Inner Singularity**: $\theta_2 = 180^\circ$ (Folded back)

Safety margins are applied (e.g., 99% max reach) to prevent locking.

## License
This project is licensed under the Apache License 2.0.
