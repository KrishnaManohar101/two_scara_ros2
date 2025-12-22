# Kinematics and Dynamics Upgrade: Moving to 3D

## 1. Overview
As per the project discussion and teacher requirements, the SCARA robot simulation has been upgraded from a 2D planar model to a **3D spatial model**. This upgrade incorporates a prismatic (sliding) Z-axis and a basic dynamics engine to account for mass, inertia, and gravity.

## 2. 3D Kinematics Implementation

### 2.1 Forward Kinematics (FK)
The Forward Kinematics now calculates the 3D position $(x, y, z)$ of the end-effector based on the three joint states:
- **$\theta_1$**: Shoulder Rotation (rad)
- **$\theta_2$**: Elbow Rotation (rad)
- **$d_3$**: Z-axis Translation (m)

**The Math:**
$$ x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) $$
$$ y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) $$
$$ z = H_{base} - d_3 $$

### 2.2 Inverse Kinematics (IK)
The IK solver now accepts a 3D target $(x, y, z)$ and returns the necessary joint values. Because the SCARA's vertical motion is decoupled from its horizontal motion, the Z-axis is calculated independently:
- **$d_3 = H_{base} - z$**

## 3. Dynamics and Physics Integration

To satisfy the requirement for "demonstrating the effect of changing forces," the `SCARAKinematics` class now includes a **Dynamics Engine**.

### 3.1 The Inertia Matrix ($M$)
We calculate a simplified Inertia Matrix that changes based on the robot's configuration. As the arm extends ($\theta_2 \to 0$), the moment of inertia increases, requiring more torque to achieve the same acceleration.

### 3.2 Gravity Compensation ($G$)
In the 2D model, gravity was ignored. In 3D, gravity ($g = 9.81 m/s^2$) acts directly on the Z-axis.
- **Requirement:** The motor must exert a constant force of $F = m \cdot g$ just to keep the arm from falling.

### 3.3 Torque Calculation ($\tau$)
The system now estimates the Torque/Force required for a move:
$$ \tau = M(\theta) \cdot \ddot{\theta} + G(\theta) $$
This allows the evaluator to see how changing the **Mass** variables live in the code affects the "Torque Budget" of the robot.

## 4. How to Demonstrate to the Teacher
1. **The "Data One, Data Two" Test:** Run the kinematics script. Show that moving Joint 3 (Z) updates the Z-output in Forward Kinematics without affecting X and Y.
2. **The "Mass Change" Test:** Increase the `payload_mass` in the code and show that the calculated `required_torque` increases, even for the same trajectory.
3. **The "3D Reach" Test:** Command the robot to a Z-height. Show that the IK solver correctly calculates the prismatic extension.
