# Comprehensive Guide to `kinematics.py`

This document provides a full, line-by-line technical and conceptual breakdown of the `kinematics.py` module. It includes the source code, explanations of the logic, and the physics-based reasoning behind the implementation.

---

## 🏗️ Part 1: Imports and Initialization

```python
1: #!/usr/bin/env python3
2: """
3: Kinematics module for SCARA robots - Forward and Inverse Kinematics
4: Includes Optimization-based Trajectory Generation
5: """
6: 
7: import numpy as np
8: import math
```
*   **Lines 1-8**: Standard setup. `numpy` is the engine for all matrix math and trigonometry.

```python
10: class SCARAKinematics:
11:     """
12:     Forward and Inverse Kinematics for 3D SCARA robot
13:     Includes Dynamics calculations (Inertia, Gravity, Torque)
14:     """
15:     
16:     def __init__(self, l1=1.5, l2=1.0, h_base=2.0, mass_l1=5.0, mass_l2=3.0, mass_payload=1.0):
26:         self.l1 = l1
27:         self.l2 = l2
28:         self.h_base = h_base ...
34:         self.g = 9.81  # Gravity m/s^2
```
*   **Initialization**: Defines the **Digital Twin** of the robot. 
    *   `l1, l2`: Lengths determine the workspace.
    *   `mass`: Used later to calculate how hard the motors must work (Dynamics).
    *   `h_base`: The height from which the prismatic (vertical) joint extends.

---

## 🎯 Part 2: Forward Kinematics (FK)
**Concept**: "Where is the tip if I know my motor angles?"

```python
36:     def forward_kinematics(self, theta1, theta2, d3):
43:         x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
44:         y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
47:         z = self.h_base - d3
49:         return x, y, z
```
*   **Logic**: Uses vector addition. 
    *   The first arm is at `theta1`. 
    *   The second arm's orientation in the world is the sum of both joint angles (`theta1 + theta2`). 
    *   `z` is height. Since `d3` extends downwards, it is subtracted from the base height.

---

## 🔄 Part 3: Inverse Kinematics (IK)
**Concept**: "What motor angles do I need to reach $(x, y, z)$?"

```python
51:     def inverse_kinematics(self, x, y, z, elbow='up'):
58:         d3 = self.h_base - z
61:         d_sq = x**2 + y**2
62:         d = np.sqrt(d_sq)
```
*   **Line 58**: Solving for elevation is simple subtraction.
*   **Lines 61-64**: Calculates the reach distance $D$. If $D > L1 + L2$, the target is too far away, and the function returns `None`.

```python
67:         cos_theta2 = (d_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
68:         cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
71:             theta2 = np.arccos(cos_theta2) # Elbow Up
...
75:         alpha = np.arctan2(y, x)
76:         beta = np.arctan2(self.l2 * np.sin(theta2), 
77:                          self.l1 + self.l2 * np.cos(theta2))
79:         theta1 = alpha - beta
```
*   **Logic**: Uses the **Law of Cosines**. 
    *   `cos_theta2` calculates the "bending" of the elbow.
    *   `alpha` is the angle of the target line from the base.
    *   `beta` is the compensation angle for the triangle formed by the links.

---

## ⚖️ Part 4: Dynamics (Torque & Force)
**Concept**: "Newton's laws for a rotating arm."

```python
83:     def calculate_dynamics(self, theta1, theta2, q_ddot):
91:         M11 = (self.m1 + self.m2 + self.m_p) * self.l1**2 + \
92:               (self.m2 + self.m_p) * (self.l2**2 + 2 * self.l1 * self.l2 * np.cos(theta2))
```
*   **Line 91-92: The Inertia Matrix**: Notice the `cos(theta2)` term. When the robot is "folded," its inertia is smaller (it's easier to rotate). When extended, it's harder.
*   **Lines 97-99**: Gravity only affects Joint 3 (`G3`). Joints 1 and 2 move horizontally, so gravity doesn't push them forwards or backwards.

---

## ⚡ Part 5: The Jacobian and Trajectories
**Concept**: "How to move in a perfectly straight line."

```python
108:     def jacobian(self, theta1, theta2):
115:         J = np.array([
116:             [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12],
117:             [self.l1 * c1 + self.l2 * c12, self.l2 * c12]
118:         ])
```
*   **The Jacobian**: This is the "sensitivity matrix." It tells the robot: "If I move a motor by $0.1$ degrees, exactly how many millimeters does the tooltip move?"

```python
121:     def generate_straight_line_path(self, ...):
151:                 J_inv = np.linalg.pinv(J)
152:                 q_dot = np.dot(J_inv, np.array([vx, vy]))
157:             q = q + q_dot * dt
```
*   **Logic**: To move in a straight line, we take the desired Cartesian velocity ($V_x, V_y$) and multiply it by the **Inverse Jacobian**. This calculates the exact motor speeds (`q_dot`) needed to stay on the line.

---

## 🛡️ Part 6: Collision Detection
**Concept**: "Treat the robot as a physical object, not just a point."

```python
192:     def get_link_positions(self, theta1, theta2, base_x=0, base_y=0):
203:         positions = [(base_x, base_y)] # Base
208:         positions.append((j1_x, j1_y))    # Elbow
213:         positions.append((ee_x, ee_y))    # Tip
```
*   **Logic**: This function converts the joint angles into 3 vital $(x,y)$ points on the map.

```python
217:     def check_collision(self, ...):
237:                 dist = self._segment_distance(...)
244:         return min_distance < (2 * self.robot_radius)
```
*   **Logic**: It treats the arms as line segments. It calculates the shortest distance between any part of Robot A and any part of Robot B. If this "gap" is smaller than the safety radius ($5cm$), it flags a collision.

---

## 💡 Summary: How it works in practice

1.  **Input**: User says "Move to (1.0, 0.5, 1.8)".
2.  **Logic**: `inverse_kinematics` checks if the point is reachable.
3.  **Path**: `generate_straight_line_path` calculates a smooth trajectory of intermediate points.
4.  **Verification**: At every tiny step, `check_collision` ensures the other SCARA robot isn't in the way.
5.  **Execution**: The motors are given the joint values calculated by the path generator.
