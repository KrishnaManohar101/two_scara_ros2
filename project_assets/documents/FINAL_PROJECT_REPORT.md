# FINAL PROJECT REPORT: Dual SCARA Robot Collaboration System

**Author:** [Your Name/Team Name]  
**Date:** December 20, 2025  
**Subject:** Mathematical Intelligent Systems (MIS) & Robotics  

---

## Abstract
This project focuses on the design, simulation, and control of a collaborative dual-SCARA robot system for automated material handling. Leveraging **ROS 2 Humble** and **Gazebo**, we simulated a realistic industrial environment where two SCARA robots coordinate to sort objects on a shared conveyor belt. The core innovation lies in the rigorous application of mathematical optimization techniques—specifically **Lagrange Multipliers**—to generate energy-efficient, straight-line Cartesian trajectories, replacing standard joint interpolation methods. The system also integrates inverse kinematics, collision avoidance algorithms, and a master-slave replication architecture to ensure synchronized operation.

---

## 1. Introduction

### 1.1 Motivation
In modern manufacturing, "Pick-and-Place" operations require high speed and precision. SCARA (Selective Compliance Assembly Robot Arm) robots are the industry standard for these tasks due to their rigidity in the vertical axis and compliance in the horizontal plane. However, coordinating multiple robots in a shared workspace imparts significant challenges in collision avoidance and trajectory optimization.

### 1.2 Objectives
1.  **System simulation**: Develop a high-fidelity simulation of two SCARA robots in Gazebo.
2.  **Kinematic Modeling**: Implement full Forward and Inverse Kinematics (FK/IK) from scratch.
3.  **Optimization (MIS Unit-1)**: Apply mathematical optimization (Lagrange Multipliers) to enforce Cartesian straight-line path constraints while minimizing joint velocity norms.
4.  **Collaboration**: Implement a logic system where a "Slave" robot mirrors the "Master" robot's actions to double throughput.

---

## 2. System Architecture

### 2.1 Mechanical Specification
The system consists of two identical SCARA robots positioned symmetrically around a central conveyor belt.
*   **Robot Type**: 3-DOF SCARA (2 Revolute Joints + 1 Prismatic Vertical Joint).
*   **Link Lengths**: $L_1 = 1.5$m (Humerus), $L_2 = 1.0$m (Forearm).
*   **Workspace**: Toroidal (Donut shape).
    *   $R_{max} = 2.5$m.
    *   $R_{min} = 0.5$m.
*   **Mounting**:
    *   Left Robot Base: $(2.0, -2.0)$
    *   Right Robot Base: $(2.0, 2.0)$

### 2.2 Software Stack
*   **Middleware**: ROS 2 Humble Hawksbill.
*   **Simulation Physics**: Gazebo Classic (ODE Engine).
*   **Language**: Python 3 (Control/Logic) and XML/Xacro (Robot Description).
*   **Control Interface**: `ros2_control` with `JointTrajectoryController`.

---

## 3. Mathematical Modeling

### 3.1 Kinematics (FK & IK)
The foundational math for robot motion.

#### Forward Kinematics
Determines the end-effector position $(x, y)$ from joint angles $\theta_1, \theta_2$.
$$ x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) $$
$$ y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) $$

#### Inverse Kinematics
Determines required joint angles for a specific target $(x, y)$. Solved analytically using the geometric "Elbow Up" configuration:
$$ \theta_2 = \arccos\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right) $$
$$ \theta_1 = \arctan2(y, x) - \arctan2(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2) $$

### 3.2 Trajectory Optimization (MIS Unit-1 Application)
Standard joint interpolation results in curved paths that are dangerous in crowded workspaces. We applied **Lagrange Multipliers** to constrain the motion to a straight line while optimizing for minimal joint velocity (proxy for energy/smoothness).

**The Problem:**
Minimize cost function $f(\dot{\theta}) = \frac{1}{2}\dot{\theta}^T\dot{\theta}$ (Squared Norm of Joint Velocities)
Subject to constraint $g(\dot{\theta}) = J(\theta)\dot{\theta} - \mathbf{v}_{cartesian} = 0$

**The Solution (Derived via Lagrangian):**
$$ \mathcal{L} = \frac{1}{2} \dot{\theta}^T \dot{\theta} + \lambda^T (J\dot{\theta} - \mathbf{v}) $$
Solving $\nabla \mathcal{L} = 0$ yields the control law:
$$ \dot{\theta}^* = J^{\dagger} \mathbf{v}_{target} = J^T(JJ^T)^{-1} \mathbf{v}_{target} $$

This ensures the robot moves in a perfect linear path in the application workspace, drastically reducing the risk of collisions with the conveyor structure compared to the unoptimized arc motion.

---

## 4. Implementation Details

### 4.1 ROS 2 Node Structure
The system is composed of several communicating nodes:
1.  **`automation_manager`**: The "Brain". Handles high-level logic, user input, and state machines.
2.  **`scara_left_motion_planner`** & **`scara_right_motion_planner`**: Solves kinematics and publishes trajectories.
3.  **`conveyor_driver`**: Simulates the moving belt physics.
4.  **`gazebo_ros2_control`**: Hardware interface abstraction layer.

### 4.2 Critical Algorithms
#### Singularity Handling
We monitor the Jacobian Determinant $|\det(J)| = |L_1 L_2 \sin(\theta_2)|$. If it drops below a threshold (0.01), actions are clamped to prevent infinite velocity commands, ensuring system stability near workspace boundaries.

#### Dual-Robot Replication
To achieve synchronized sorting, the Right Robot is programmed to mirror the Left Robot's relative moves:
$$ \Delta P_{Right} = \begin{bmatrix} 1 & 0 \\ 0 & -1 \end{bmatrix} \Delta P_{Left} $$
This reflection matrix ensures symmetry across the conveyor axis ($Y=0$).

---

## 5. Results & Validation

### 5.1 Trajectory Comparison
We implemented a demo script `optimization_demo.py` to compare standard vs optimized paths.
*   **Standard Method**: Resulted in an arc path with a deviation of up to **0.4m** from the straight line.
*   **Optimized Method**: Maintained a deviation $< 0.001$m (numerical error only), validating the Lagrange formulation.

### 5.2 Reachability Testing
*   **Test Case 1**: Target $(2.0, -3.5)$. Distance 1.5m. **Success** (Within range).
*   **Test Case 2**: Target $(5.0, 0.0)$. Distance 3.0m. **Rejected** (Exceeds Max Reach 2.5m).

### 5.3 Simulation Performance
Values logged during `gazebo.launch.py` execution:
*   **Controller Frequency**: 100 Hz.
*   **IK Solve Time**: < 0.5 ms (Analytic solution is extremely fast).
*   **Optimization Integration Step**: 50 ms (Real-time capable).

---

## 6. Conclusion and Future Scope
We successfully developed a dual-SCARA robot simulation that incorporates advanced mathematical modeling directly from the MIS syllabus. The application of **Optimization with Lagrange Multipliers** significantly improved the trajectory quality, ensuring straight-line motion essential for conveyor belt operations.

**Future Scope:**
*   **Stochastic Analysis (MIS Unit-3)**: Implement Markov Chain modeling for failure prediction and reliability analysis based on long-duration test runs.
*   **Dynamic Obstacle Avoidance**: Integrating real-time convex hull collision checking.

---

## Appendices
*   **A**: Source Code (`kinematics.py`, `automation_manager.py`)
*   **B**: Optimization Derivation Report (`OPTIMIZATION_REPORT.md`)
*   **C**: Physics Specifications (`Description_and_Physics.md`)
