# Technical Documentation: Two SCARA Robot Collaboration System

## Executive Summary
This document provides a comprehensive technical analysis of a dual-SCARA robot system designed for collaborative sorting and manipulation tasks. It details the kinematic modeling, dynamic analysis, control architecture, and workspace management strategies employed to achieve synchronized, collision-free operation in a shared environment.

---

## 1. Introduction

### 1.1 Background and Motivation
The automation of material handling often requires collaborative multi-robot systems. This project explores the mathematical and physical foundations of coordinating two SCARA robots to perform mirrored tasks across a conveyor belt, necessitating precise kinematic control and workspace management.

### 1.2 Document Scope
This documentation covers:
*   Physical specifications and geometric modeling.
*   The rigorous derivation of Forward and Inverse Kinematics.
*   Coordinate transformations and frame hierarchies.
*   Dynamic equations of motion and control implications.
*   Singularity analysis using the Jacobian matrix.
*   Dual-robot coordination and replication logic.

### 1.3 Terminology and Notation
*   **$\{W\}$**: World Coordinate Frame.
*   **$\{B_L\}, \{B_R\}$**: Base Frames for Left and Right robots.
*   **$L_1, L_2$**: Link lengths (Humerus and Forearm).
*   **$\theta_1, \theta_2$**: Joint angles (Shoulder and Elbow).
*   **FK/IK**: Forward/Inverse Kinematics.

---

## 2. System Architecture and Specifications

### 2.1 Robot Geometry and Physical Configuration

#### 2.1.1 SCARA Configuration Rationale
The SCARA (Selective Compliance Assembly Robot Arm) configuration was chosen for its high rigidity in the vertical (Z) axis and compliance in the X-Y plane, making it ideal for "pick-and-place" operations on a horizontal conveyor.

#### 2.1.2 Dimensional Specifications
Based on the URDF and simulation models:
-   **Link 1 ($L_1$)**: 1.5 meters.
-   **Link 2 ($L_2$)**: 1.0 meters.
-   **Total Reach ($R_{max}$)**: 2.5 meters.
-   **Minimum Reach ($R_{min}$)**: $|L_1 - L_2| = 0.5$ meters.

#### 2.1.3 Workspace Geometry
Each robot operates in a toroidal (doughnut-shaped) workspace defined by two concentric circles:
*   **Outer Radius**: $R_{out} = L_1 + L_2 = 2.5$m.
*   **Inner Radius**: $R_{in} = |L_1 - L_2| = 0.5$m.

### 2.2 Dual-Robot Spatial Configuration

#### 2.2.1 Position and Orientation in Global Frame
The robots are mounted symmetrically relative to the workspace center:
*   **Left Robot $\{B_L\}$**: Positioned at global $(2.0, -2.0, 0.2)$.
*   **Right Robot $\{B_R\}$**: Positioned at global $(2.0, 2.0, 0.2)$.
*   **Orientation**: Both base frames share the same orientation as the World Frame (Identity Rotation Matrix).

#### 2.2.2 Workspace Overlap and Collaborative Region
The workspace overlap allows both robots to access the central conveyor belt region. The collaborative zone is the intersection of the two toroidal workspaces, primarily centered along the Y-axis between $y=-0.5$ and $y=0.5$.

---

## 3. Kinematic Analysis

### 3.1 Forward Kinematics (FK)

#### 3.1.1 Problem Definition
Determine the end-effector position $(x, y)$ given the joint angles $\theta_1$ and $\theta_2$.

#### 3.1.2 Mathematical Derivation
Using the Denavit-Hartenberg (DH) convention for planar manipulators:
$$ x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) $$
$$ y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) $$

#### 3.1.3 Interpretation
The equations represent vector summation in the 2D plane. The first term is the vector to the elbow; the second term is the vector from the elbow to the wrist.

#### 3.1.4 Numerical Example
Let $L_1 = 1.5, L_2 = 1.0$.
If $\theta_1 = 0^\circ, \theta_2 = 90^\circ$:
$x = 1.5 \cos(0) + 1.0 \cos(90) = 1.5$
$y = 1.5 \sin(0) + 1.0 \sin(90) = 1.0$
Result: End-effector is at $(1.5, 1.0)$.

### 3.2 Inverse Kinematics (IK)

#### 3.2.1 Problem Definition and Challenges
Determine the joint angles ($\theta_1, \theta_2$) required to reach a target $(x, y)$. This problem is non-linear and may have multiple solutions ("Elbow Up" vs "Elbow Down").

#### 3.2.2 Reachability Check
Before processing, we verify:
$$ R_{min}^2 \le x^2 + y^2 \le R_{max}^2 $$
$$ 0.5^2 \le x^2 + y^2 \le 2.5^2 $$

#### 3.2.3 Step 1: Elbow Angle Calculation
Using the Law of Cosines on the triangle formed by origin, elbow, and wrist:
$$ d^2 = L_1^2 + L_2^2 - 2 L_1 L_2 \cos(\pi - \theta_2) $$
$$ \theta_2 = \pm \arccos\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right) $$

#### 3.2.4 Step 2: Shoulder Angle Calculation
Using geometric difference of angles:
$$ \alpha = \arctan2(y, x) $$
$$ \beta = \arctan2(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2) $$
$$ \theta_1 = \alpha - \beta $$

#### 3.2.5 Numerical Example: IK Solution
Target $(1.5, 1.0)$, $L_1=1.5, L_2=1.0$.
1.  $d^2 = 3.25$.
2.  $\cos\theta_2 = (3.25 - 2.25 - 1.0) / 3.0 = 0$. $\theta_2 = 90^\circ$.
3.  $\alpha = \arctan(1/1.5) = 33.69^\circ$.
4.  $\beta = \arctan(1.0/1.5) = 33.69^\circ$.
5.  $\theta_1 = 0^\circ$.
Result: $\theta_1 = 0^\circ, \theta_2 = 90^\circ$, matching the FK example.

### 3.3 Geometric Interpretation
The IK solution corresponds to finding the intersection points of two circles: one centered at the origin with radius $L_1$, and one centered at target $(x,y)$ with radius $L_2$.

---

## 4. Coordinate Transformations and Frame Conventions

### 4.1 Frame Hierarchy
The software maintains a strict hierarchy:
`World` -> `Left_Base` -> `Left_Shoulder` ...
`World` -> `Right_Base` -> `Right_Shoulder` ...

### 4.2 World to Local Transformation

#### 4.2.1 Transformation Principles
Robots operate in local coordinates relative to their base. User inputs are global.

#### 4.2.2 Left Robot Transformation
$$ P_{local\_L} = P_{world} - \begin{bmatrix} 2.0 \\ -2.0 \end{bmatrix} $$

#### 4.2.3 Right Robot Transformation
$$ P_{local\_R} = P_{world} - \begin{bmatrix} 2.0 \\ 2.0 \end{bmatrix} $$

#### 4.2.4 Transformation Matrix Representation (2D)
For the Left Robot (Rotation is Identity):
$$
^{Base}T_{World} = \begin{bmatrix} 1 & 0 & -2.0 \\ 0 & 1 & 2.0 \\ 0 & 0 & 1 \end{bmatrix}
$$

#### 4.2.5 3D Homogeneous Transformation Matrix
$$
^{B}T_{W} = \begin{bmatrix}
1 & 0 & 0 & -x_{base} \\
0 & 1 & 0 & -y_{base} \\
0 & 0 & 1 & -z_{base} \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### 4.3 Inverse Transform: Local to World
$$ P_{world} = P_{local} + P_{base} $$
Used for visualization in RViz and verification.

### 4.4 Replication Math: Symmetric Task Execution

#### 4.4.1 Mimicry Principle
The system's core feature is symmetric replication. If the Left robot moves relative to its base by $(dx, dy)$, the Right robot should move relative to its base by $(dx, -dy)$ to maintain symmetry across the conveyor axis.

#### 4.4.2 Vector-Based Approach
Let $\vec{v}_L$ be the vector from Left Base to Left Target.
Let $\vec{v}_R$ be the vector from Right Base to Right Target.
$$ \vec{v}_R = \mathbf{M} \vec{v}_L $$
Where $\mathbf{M} = \text{diag}(1, -1)$ is the reflection matrix across the X-axis.

#### 4.4.3 Geometric Insight
This ensures that both robots approach the conveyor centerline (Local Y > 0 for Left, Local Y < 0 for Right) simultaneously.

#### 4.4.4 Mathematical Verification
Left Target relative $(0, 1)$ -> "Move Forward 1m".
Right Target relative $(0, -1)$ -> "Move Forward 1m" (since Right is on the opposite side, $-Y$ direction is towards the center).

---

## 5. Dynamics and Control Physics

### 5.1 Equation of Motion
The dynamic behavior is governed by the standard Lagrangian equation of motion:
$$ M(\theta)\ddot{\theta} + V(\theta, \dot{\theta}) + G(\theta) + F(\dot{\theta}) = \tau $$

#### 5.1.1 Newton-Euler Formulation
Alternatively, the recursive Newton-Euler method computes forces/torques link-by-link from base to tip (forward velocity) and tip to base (backward force) to derive $\tau$.

#### 5.1.2 Inertia Matrix $M(\theta)$
A symmetric, positive-definite matrix representing the mass distribution.
For SCARA:
$$ M_{11} = I_1 + I_2 + m_1 r_1^2 + m_2(L_1^2 + r_2^2 + 2L_1 r_2 \cos\theta_2) $$
$M(\theta)$ changes as the robot arm extends and retracts, altering the effective moment of inertia.

#### 5.1.3 Velocity-Dependent Terms $V(\theta, \dot{\theta})$
Includes:
*   **Coriolis Forces**: Interaction between links (e.g., shoulder rotation affecting elbow).
*   **Centrifugal Forces**: Forces pushing adjacent links outward during rotation.

#### 5.1.4 Gravity Term $G(\theta)$
Since our SCARA robots operate planar (X-Y), the gravity vector acts strictly in the Z-direction, orthogonal to the plane of motion. Thus, for joints $\theta_1, \theta_2$:
$$ G(\theta) = 0 $$
*(Z-axis prismatic joint would have a gravity term)*.

#### 5.1.5 Friction Term $F(\dot{\theta})$
Modeled as Coulomb + Viscous friction:
$$ F = F_v \dot{\theta} + F_c \text{sgn}(\dot{\theta}) $$

### 5.2 Control Implications

#### 5.2.1 Torque Budget
The controllers must provide enough torque $\tau$ to overcome inertia and friction. Rapid acceleration phases ($ \ddot{\theta} $) dominate the torque requirements.

#### 5.2.2 Speed Advantage
Because gravity does not oppose planar motion, SCARA robots can achieve very high lateral speeds and accelerations compared to articulated arms.

---

## 6. Workspace Analysis

### 6.1 Workspace Definition and Boundaries

#### 6.1.1 Reachable Workspace
Defined as the set of all points $P \in \mathbb{R}^3$ that the end-effector can reach.
$$ W = \{ P(x,y) \mid R_{in} \le \|P\| \le R_{out} \} $$

#### 6.1.2 Mathematical Boundary Conditions
1.  **Outer Boundary**: Singular configuration where $\theta_2 = 0$. Reach $= L_1 + L_2$.
2.  **Inner Boundary**: Singular configuration where $\theta_2 = 180^\circ$. Reach $= |L_1 - L_2|$.

### 6.2 Dual-Robot Collaborative Workspace

#### 6.2.1 Individual Workspaces
Two disjoint toroids separated by distance $D_{base\_y} = 4.0m$.

#### 6.2.2 Workspace Intersection (Collaborative Region)
The robots can only collaborate where their workspaces overlap.
Given $R_{max} = 2.5m$ and separation $4.0m$, overlap exists in the central strip.
Condition for overlap: $2 \times R_{max} > \text{Base\_Separation}$.
$5.0m > 4.0m$ $\implies$ Overlap exists.

---

## 7. Singularity Theory and Management

### 7.1 Singularity Concept and Definition

#### 7.1.1 Jacobian and Kinematic Singularities
The Jacobian $J$ maps joint velocities to cartesian velocities: $\dot{X} = J\dot{q}$. A singularity occurs when $J$ is non-invertible.

#### 7.1.2 Singularity Definition
Mathematically, where $\det(J) = 0$.

#### 7.1.3 Determinant Calculation
For a 2-link planer arm:
$$ \det(J) = L_1 L_2 \sin(\theta_2) $$
Singularity condition: $\sin(\theta_2) = 0 \implies \theta_2 = 0, \pi$.

### 7.2 Physical Interpretation of Singularities

#### 7.2.1 Fully Extended Configuration ($\theta_2 = 0^\circ$)
The arm is straight. The robot cannot move directly towards or away from the base (radial direction) because the joint velocities are perpendicular to the radius.

#### 7.2.2 Fully Folded Configuration ($\theta_2 = 180^\circ$)
The arm is folded back on itself. Similar constraint applies; radial motion is impossible.

### 7.3 Consequences and Impact on Control

#### 7.3.1 Velocity Singularity
 Near singularity, small cartesian velocities require infinite joint velocities.
 $$ \dot{q} = J^{-1} \dot{X} $$
 As $\det(J) \to 0$, $J^{-1} \to \infty$.

#### 7.3.2 Force Singularity
Conversely, the robot can resist infinite forces in the singular direction (locking the arm straight).

### 7.4 Singularity Avoidance and Management Strategies

#### 7.4.1 Workspace Margin
Software limits are set to prevent the arm from reaching full extension (e.g., max reach capped at 99%).

#### 7.4.2 Manipulability Index
We monitor the measure $w = \sqrt{\det(JJ^T)}$ to assess how close the robot is to singularity.

#### 7.4.3 Damped Least-Squares (DLS) Method
In close-to-singular regions, an inverse that minimizes norm error + damping factor is used to maintain stability (not implemented in this prototype but recommended for production).

---

## 8. Dual-Robot Coordination

### 8.1 Synchronized Motion Architecture

#### 8.1.1 Task Specification
The Master (Left) robot receives a task $T_{left}$. The Slave (Right) robot automatically receives $T_{right} = \text{Mirror}(T_{left})$.

#### 8.1.2 Motion Planning Pipeline
1.  **Input**: User Cartesian Target.
2.  **Manager**: Splits and Mirrors target.
3.  **Planner (L/R)**: Computes IK and generates trajectory points.
4.  **Hardware Interface**: Executes synchronized joint commands.

### 8.2 Replication Mathematics Deep Dive

#### 8.2.1 Vector Replication Formula
$$ \vec{P}_R = \vec{P}_{BaseR} + \begin{bmatrix} 1 & 0 \\ 0 & -1 \end{bmatrix} (\vec{P}_{Target} - \vec{P}_{BaseL}) $$

---

## 9. Numerical Validation and Case Studies

### 9.1 Case Study 1: Reachable Target Validation
**Input**: Global $(2.0, -3.5)$.
*   Rel Left: $(0, -1.5)$. $d=1.5$. Reachable ($0.5 \le 1.5 \le 2.5$).
*   Robot moves.

### 9.2 Case Study 2: Unreachable Target Rejection
**Input**: Global $(5.0, -2.0)$.
*   Rel Left: $(3.0, 0)$. $d=3.0$.
*   $3.0 > 2.5$ ($R_{max}$).
*   **Result**: IK Solver rejects target, logs warning.

### 9.3 Case Study 3: Dual-Robot Replication
**Input**: $(2.0, -3.0)$ (Forward 1m from Left Base).
*   Right Robot commanded to $(2.0, 1.0)$ (Forward 1m from Right Base).
*   Symmetry maintained.

---

## 10. Operational Guidelines and Safety

### 10.1 Workspace Constraints and Limits
*   **Max Reach**: 2.45m (Safety buffer applied).
*   **Joint Velocity Limit**: 2.0 rad/s.
*   **Joint 1 Range**: $\pm 150^\circ$.
*   **Joint 2 Range**: $\pm 150^\circ$.

### 10.2 Singularity Avoidance Procedures
Operators should avoid commanding targets near the base (Inner Singularity) or at the extreme edge (Outer Singularity).

---

## 11. Appendices

### 11.1 A. Mathematical Reference
*   Law of Cosines: $c^2 = a^2 + b^2 - 2ab \cos C$.
*   Rotation Matrix 2D: $R(\theta) = [[c\theta, -s\theta], [s\theta, c\theta]]$.

### 11.2 B. Implementation Checklist
*   [x] URDF Model Verification ($L_1, L_2$).
*   [x] IK Solver Test.
*   [x] Mirroring Logic Test.
*   [x] Controller Tuning.

### 11.3 C. Numerical Constants Summary
| Parameter | Value | Unit |
| :--- | :--- | :--- |
| $L_1$ | 1.5 | m |
| $L_2$ | 1.0 | m |
| Base Separation | 4.0 | m |
| $K_p$ | 10.0 | - |
