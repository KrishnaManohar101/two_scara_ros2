#!/usr/bin/env python3
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Add src to path to import kinematics
sys.path.append(os.path.join(os.getcwd(), 'src/two_scara_collaboration'))

from two_scara_collaboration.kinematics import SCARAKinematics

def run_comparison():
    print("Initializing SCARA Kinematics 3D (L1=1.5, L2=1.0, Base_H=2.0)...")
    scara = SCARAKinematics(l1=1.5, l2=1.0, h_base=2.0, mass_payload=2.0)
    
    # Define 3D Trajectory Task
    # Move from (1.5, 1.0, 1.8) -> (0.5, 2.0, 0.5)
    start_pos = (1.5, 1.0, 1.8) 
    target_pos = (0.5, 2.0, 0.5)
    
    joints_start = scara.inverse_kinematics(*start_pos)
    joints_end = scara.inverse_kinematics(*target_pos)
    
    if joints_start[0] is None or joints_end[0] is None:
        print("Error: Points unreachable.")
        return

    print(f"Start Joints: [{joints_start[0]:.2f}, {joints_start[1]:.2f}, {joints_start[2]:.2f}]")
    print(f"End Joints:   [{joints_end[0]:.2f}, {joints_end[1]:.2f}, {joints_end[2]:.2f}]")
    
    # --- METHOD 1: Joint Interpolation ---
    print("\nCalculating Method 1: Joint Space Interpolation...")
    steps = 50
    path1_x, path1_y, path1_z = [], [], []
    
    for i in range(steps + 1):
        alpha = i / steps
        t1 = joints_start[0] + alpha * (joints_end[0] - joints_start[0])
        t2 = joints_start[1] + alpha * (joints_end[1] - joints_start[1])
        d3 = joints_start[2] + alpha * (joints_end[2] - joints_start[2])
        
        x, y, z = scara.forward_kinematics(t1, t2, d3)
        path1_x.append(x)
        path1_y.append(y)
        path1_z.append(z)
        
    # --- METHOD 2: Optimization (Straight Line) ---
    print("Calculating Method 2: Lagrangian Optimization (Cartesian Linear)...")
    traj_points = scara.generate_straight_line_path(
        joints_start,
        target_pos,
        duration=5.0,
        dt=0.1
    )
    
    path2_x, path2_y, path2_z = [], [], []
    
    # Dynamics Tracking
    max_torque = [0, 0, 0]
    
    # Get initial Inertia Matrix for display
    # (M11 varies with theta2)
    M11_start = (scara.m1 + scara.m2 + scara.m_p) * scara.l1**2 + \
                (scara.m2 + scara.m_p) * (scara.l2**2 + 2 * scara.l1 * scara.l2 * np.cos(joints_start[1]))
    M22 = (scara.m2 + scara.m_p) * scara.l2**2
    
    print("\n--- Physical Configuration & Inertia ---")
    print(f"Link 1 Mass: {scara.m1}kg | Link 2 Mass: {scara.m2}kg")
    print(f"Payload Mass: {scara.m_p}kg")
    print(f"Total System Mass: {scara.m1 + scara.m2 + scara.m_p}kg")
    print(f"Initial Inertia Matrix M(q):")
    print(f"  [{M11_start:7.2f},    0.00,    0.00]")
    print(f"  [    0.00, {M22:7.2f},    0.00]")
    print(f"  [    0.00,    0.00, {scara.m1+scara.m2+scara.m_p:7.2f}]")

    print("\nProcessing path points...")
    for t, t1, t2, d3 in traj_points:
        x, y, z = scara.forward_kinematics(t1, t2, d3)
        path2_x.append(x)
        path2_y.append(y)
        path2_z.append(z)
        
        # Calculate approximate torques for demonstration
        # Assume an acceleration of 0.5 rad/s^2 for the motor move
        accel = [0.5, 0.5, 0.5] 
        torques = scara.calculate_dynamics(t1, t2, accel)
        max_torque = [max(max_torque[i], abs(torques[i])) for i in range(3)]
        
    print(f"\n--- Dynamics Result (Move Duration: 5s) ---")
    print(f"Peak Shoulder Torque: {max_torque[0]:6.2f} Nm")
    print(f"Peak Elbow Torque:    {max_torque[1]:6.2f} Nm")
    print(f"Peak Z-Axis Force:    {max_torque[2]:6.2f} N (Gravity + Acceleration)")

    # --- PLOTTING ---
    plt.figure(figsize=(10, 8))
    
    circle_out = plt.Circle((0,0), 2.5, color='gray', fill=False, linestyle='--', label='Max Reach')
    circle_in = plt.Circle((0,0), 0.5, color='gray', fill=False, linestyle=':', label='Min Reach')
    plt.gca().add_patch(circle_out)
    plt.gca().add_patch(circle_in)
    
    plt.plot(path1_x, path1_y, 'r-o', label='Method 1: Arced Path (Joint Space)', markersize=4, alpha=0.5)
    plt.plot(path2_x, path2_y, 'g-x', label='Method 2: Straight Path (Cartesian Space)', markersize=4)
    
    plt.plot(start_pos[0], start_pos[1], 'ko', markersize=10, label='Start')
    plt.plot(target_pos[0], target_pos[1], 'b*', markersize=15, label='Target')
    
    plt.title('Comparison of 3D Trajectory Methods (X-Y Projection)')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    
    output_file = 'trajectory_comparison.png'
    plt.savefig(output_file)
    print(f"\nSuccess! Comparison plot saved to {output_file}")
    print(f"Z-Axis height moved from {start_pos[2]}m to {target_pos[2]}m")

if __name__ == "__main__":
    run_comparison()

if __name__ == "__main__":
    run_comparison()
