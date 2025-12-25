#!/usr/bin/env python3
import sys
import os
import numpy as np

# Add src to path
sys.path.append(os.path.join(os.getcwd(), 'src/two_scara_collaboration'))
from two_scara_collaboration.kinematics import SCARAKinematics

def test_display():
    # 1. Setup Robot with specific masses
    # Link1=5kg, Link2=3kg, Payload=2kg
    scara = SCARAKinematics(l1=1.5, l2=1.0, h_base=2.0, mass_l1=5.0, mass_l2=3.0, mass_payload=2.0)
    
    print("==========================================")
    print("   SCARA 3D DYNAMICS & KINEMATICS REPORT  ")
    print("==========================================")
    
    # 2. Show Physical Parameters
    print("\n[PHYSICAL PARAMETERS]")
    print(f"Link 1 (Shoulder): Length={scara.l1}m, Mass={scara.m1}kg")
    print(f"Link 2 (Elbow):    Length={scara.l2}m, Mass={scara.m2}kg")
    print(f"Payload (Gripper): Mass={scara.m_p}kg")
    print(f"Total System Mass: {scara.m1 + scara.m2 + scara.m_p}kg")
    
    # 3. Test Forward Kinematics
    # Joint 1 = 0, Joint 2 = 90 deg (1.57 rad), Joint 3 = 0.5m
    q = [0.0, 1.57, 0.5]
    x, y, z = scara.forward_kinematics(*q)
    print("\n[FORWARD KINEMATICS (FK)]")
    print(f"Input Joints:  Step1={q[0]}rad, Step2={q[1]}rad, Step3={q[2]}m")
    print(f"Output Target: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
    
    # 4. Show Inertia Matrix at this configuration
    # M depends on theta2
    # M11 = (m1+m2+mp)*L1^2 + (m2+mp)*(L2^2 + 2*L1*L2*cos(theta2))
    M11 = (scara.m1 + scara.m2 + scara.m_p) * scara.l1**2 + \
          (scara.m2 + scara.m_p) * (scara.l2**2 + 2 * scara.l1 * scara.l2 * np.cos(q[1]))
    M22 = (scara.m2 + scara.m_p) * scara.l2**2
    M33 = scara.m1 + scara.m2 + scara.m_p
    
    print("\n[INERTIA MATRIX M(q)]")
    print(f"Note: M11 changes as the arm folds/unfolds (theta2={q[1]}rad)")
    print(f"| {M11:7.2f}  0.00  0.00 |")
    print(f"|   0.00  {M22:7.2f}  0.00 |")
    print(f"|   0.00    0.00  {M33:7.2f} |")
    
    # 5. Show Torque Calculation
    # Assume we want to accelerate joints at 1.0 rad/s^2
    accel = [1.0, 1.0, 1.0]
    torques = scara.calculate_dynamics(q[0], q[1], accel)
    
    print("\n[DYNAMICS / TORQUE CALCULATION]")
    print(f"Target Acceleration: {accel} rad/s^2")
    print(f"Required Torque (Joint 1): {torques[0]:.2f} Nm")
    print(f"Required Torque (Joint 2): {torques[1]:.2f} Nm")
    print(f"Required Force  (Joint 3): {torques[2]:.2f} N  <-- (Fighting Gravity: {(scara.m2+scara.m_p)*9.81:.2f}N)")
    print("\n==========================================")

if __name__ == "__main__":
    test_display()
