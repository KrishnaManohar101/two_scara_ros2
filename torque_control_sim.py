#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import os
import numpy as np
import time

# Add path to kinematics
sys.path.append(os.path.join(os.getcwd(), 'src', 'two_scara_collaboration', 'two_scara_collaboration'))

try:
    from kinematics import SCARAKinematics
except ImportError:
    print("Error: Could not import SCARAKinematics. Aborting.")
    sys.exit(1)

class TorqueControlSim(Node):
    def __init__(self):
        super().__init__('torque_control_sim')
        
        # Publishers
        self.left_arm_pub = self.create_publisher(Float64MultiArray, '/scara_left_arm_controller/commands', 10)
        self.left_z_pub = self.create_publisher(Float64MultiArray, '/scara_left_gripper_controller/commands', 10)
        
        self.right_arm_pub = self.create_publisher(Float64MultiArray, '/scara_right_arm_controller/commands', 10)
        self.right_z_pub = self.create_publisher(Float64MultiArray, '/scara_right_gripper_controller/commands', 10)
        
        self.kinematics = SCARAKinematics()
        
        # Simulation State
        self.dt = 0.01  # Time step (seconds)
        self.sim_speed = 1.0 # Real-time factor
        
        print("Torque Control Simulation Node Initialized.")

    def run_simulation(self, robot_choice, start_q, torques, duration):
        """
        Simulate motion given constant torques
        """
        # Initial State [q1, q2, d3]
        # Note: input start_q is degrees/meters, convert to radians
        q = np.array([np.radians(start_q[0]), np.radians(start_q[1]), start_q[2]])
        q_dot = np.array([0.0, 0.0, 0.0]) # Start from rest
        
        # Input Torques [tau1, tau2, force3]
        tau = np.array(torques)
        
        steps = int(duration / self.dt)
        print(f"\nScanning Simulation: {steps} steps ({duration}s)...")
        
        # Prepare data for plotting/printing
        # We will just print start and end for CLI, but maybe streaming updates
        
        for i in range(steps):
            # 1. Forward Dynamics: Get Acceleration
            # t1, t2 are q[0], q[1]
            acc = self.kinematics.forward_dynamics(q[0], q[1], tau)
            q_ddot = np.array(acc)
            
            # 2. Integration (Euler)
            q_dot = q_dot + q_ddot * self.dt
            
            # Damping/Friction (Simple Viscous Friction to prevent infinite spin)
            # Torque = -b * velocity
            damping = 0.5 
            q_dot = q_dot * (1.0 - damping * self.dt)
            
            q = q + q_dot * self.dt
            
            # 3. Publish to Robot (Visual)
            # Assuming robot controller accepts Position commands, we just send the new position
            # This makes the Gazebo robot 'follow' the simulation
            self.publish_state(robot_choice, q)
            
            # Sleep to match real-time roughly
            time.sleep(self.dt / self.sim_speed)
            
            if i % 10 == 0:
                print(f"\rStep {i}/{steps}: J1={np.degrees(q[0]):.1f}, J2={np.degrees(q[1]):.1f}, Z={q[2]:.2f}", end="")
        
        print("\n\n--- Simulation Complete ---")
        return q

    def publish_state(self, robot_choice, q):
        arm_msg = Float64MultiArray()
        arm_msg.data = [float(q[0]), float(q[1])]
        
        z_msg = Float64MultiArray()
        z_msg.data = [float(q[2])]
        
        if robot_choice.lower().startswith('l'):
            self.left_arm_pub.publish(arm_msg)
            self.left_z_pub.publish(z_msg)
        else:
            self.right_arm_pub.publish(arm_msg)
            self.right_z_pub.publish(z_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TorqueControlSim()
    
    print("\n--- Forward Dynamics Simulator ---")
    print("NOTE: SCARA X-Y plane is frictionless in this model.")
    print("      Constant torque = Continuous Acceleration -> Infinite Speed.")
    print("      A small damping factor (0.5) is added to stabilize.")
    
    try:
        while True:
            robot = input("\nSelect Robot (left/right): ").strip()
            if robot.lower() == 'q': break
            
            # Initial State
            print("\nDefine Initial State:")
            j1_start = float(input("  Start J1 (deg): "))
            j2_start = float(input("  Start J2 (deg): "))
            z_start  = float(input("  Start Z  (m)  : "))
            
            # Move robot to start first
            node.publish_state(robot, [np.radians(j1_start), np.radians(j2_start), z_start])
            print("Robot moved to start. Ready for Force Input.")
            
            # Force Input
            print("\nDefine Applied Forces:")
            tau1 = float(input("  Torque J1 (Nm): "))
            tau2 = float(input("  Torque J2 (Nm): "))
            f3   = float(input("  Force Z   (N) : "))
            
            duration = float(input("Simulation Duration (s): "))
            
            final_q = node.run_simulation(robot, [j1_start, j2_start, z_start], [tau1, tau2, f3], duration)
            
            print(f"Final State:")
            print(f"  J1: {np.degrees(final_q[0]):.2f} deg")
            print(f"  J2: {np.degrees(final_q[1]):.2f} deg")
            print(f"  Z : {final_q[2]:.2f} m")
            
            # Calculate final FK
            fk_x, fk_y, fk_z = node.kinematics.forward_kinematics(final_q[0], final_q[1], -final_q[2])
            print(f"  Cartesian: ({fk_x:.2f}, {fk_y:.2f}, {fk_z:.2f})")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
