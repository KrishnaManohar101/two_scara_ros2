#!/usr/bin/env python3
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Add src to path to import kinematics
sys.path.append(os.path.join(os.getcwd(), 'src/two_scara_collaboration'))

from two_scara_collaboration.kinematics import SCARAKinematics

def run_comparison():
    print("Initializing SCARA Kinematics (L1=1.5, L2=1.0)...")
    scara = SCARAKinematics(l1=1.5, l2=1.0)
    
    # Define Trajectory Task
    # Move from (1.5, 1.0) -> (0.5, 2.0)
    # Start: theta1=0, theta2=90 (approx)
    start_pos = (1.5, 1.0) 
    target_pos = (0.5, 2.0)
    
    theta1_start, theta2_start = scara.inverse_kinematics(*start_pos)
    theta1_end, theta2_end = scara.inverse_kinematics(*target_pos)
    
    if theta1_start is None or theta1_end is None:
        print("Error: Points unreachable.")
        return

    print(f"Start Joints: {theta1_start:.2f}, {theta2_start:.2f}")
    print(f"End Joints: {theta1_end:.2f}, {theta2_end:.2f}")
    
    # --- METHOD 1: Joint Interpolation (Old) ---
    print("\nCalculated Method 1: Joint Interpolation...")
    steps = 50
    path1_x = []
    path1_y = []
    
    for i in range(steps + 1):
        alpha = i / steps
        t1 = theta1_start + alpha * (theta1_end - theta1_start)
        t2 = theta2_start + alpha * (theta2_end - theta2_start)
        
        x, y = scara.forward_kinematics(t1, t2)
        path1_x.append(x)
        path1_y.append(y)
        
    # --- METHOD 2: Optimization / Jacobian (New) ---
    print("Calculated Method 2: Lagrangian Optimization (Straight Line)...")
    traj_points = scara.generate_straight_line_path(
        (theta1_start, theta2_start),
        target_pos,
        duration=5.0,
        dt=0.1
    )
    
    path2_x = []
    path2_y = []
    
    for t, t1, t2 in traj_points:
        x, y = scara.forward_kinematics(t1, t2)
        path2_x.append(x)
        path2_y.append(y)
        
    # --- PLOTTING ---
    plt.figure(figsize=(10, 8))
    
    # Plot Workspace Boundaries (approx)
    circle_out = plt.Circle((0,0), 2.5, color='gray', fill=False, linestyle='--', label='Max Reach')
    circle_in = plt.Circle((0,0), 0.5, color='gray', fill=False, linestyle=':', label='Min Reach')
    plt.gca().add_patch(circle_out)
    plt.gca().add_patch(circle_in)
    
    # Plot Trajectories
    plt.plot(path1_x, path1_y, 'r-o', label='Method 1: Joint Interpolation (Unoptimized)', markersize=4)
    plt.plot(path2_x, path2_y, 'g-x', label='Method 2: Lagrange Optimization (Constrained)', markersize=4)
    
    # Plot Start/End
    plt.plot(start_pos[0], start_pos[1], 'ko', markersize=10, label='Start')
    plt.plot(target_pos[0], target_pos[1], 'b*', markersize=15, label='Target')
    
    plt.title('Comparison of Trajectory Optimization Methods')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    
    output_file = 'trajectory_comparison.png'
    plt.savefig(output_file)
    print(f"\nSuccess! Comparison plot saved to {output_file}")
    print("Open this image to see the difference!")

if __name__ == "__main__":
    run_comparison()
