#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import os
import numpy as np

# Add the path to kinematics module
# Assuming structure: workspace/src/two_scara_collaboration/two_scara_collaboration/kinematics.py
# and this script is in workspace/
sys.path.append(os.path.join(os.getcwd(), 'src', 'two_scara_collaboration', 'two_scara_collaboration'))

try:
    from kinematics import SCARAKinematics
except ImportError:
    print("Warning: Could not import SCARAKinematics. FK calculation will be disabled.")
    SCARAKinematics = None

class DirectJointControl(Node):
    def __init__(self):
        super().__init__('direct_joint_control')
        
        # Publishers for Left Arm
        self.left_arm_pub = self.create_publisher(Float64MultiArray, '/scara_left_arm_controller/commands', 10)
        self.left_z_pub = self.create_publisher(Float64MultiArray, '/scara_left_gripper_controller/commands', 10)
        
        # Publishers for Right Arm
        self.right_arm_pub = self.create_publisher(Float64MultiArray, '/scara_right_arm_controller/commands', 10)
        self.right_z_pub = self.create_publisher(Float64MultiArray, '/scara_right_gripper_controller/commands', 10)
        
        # Initialize Kinematics (Default parameters)
        if SCARAKinematics:
            self.kinematics = SCARAKinematics()
        else:
            self.kinematics = None
            
        print("Direct Joint Control Node Initialized.")

    def publish_joints(self, robot_choice, theta1_deg, theta2_deg, z_val):
        # Convert degrees to radians
        t1_rad = np.radians(theta1_deg)
        t2_rad = np.radians(theta2_deg)
        
        # Publish
        arm_msg = Float64MultiArray()
        arm_msg.data = [float(t1_rad), float(t2_rad)]
        
        z_msg = Float64MultiArray()
        # Ensure Z is within safe limits (0.0 to -0.3 roughly for this robot, check URDF if unsure)
        # Typically Z is negative for down in this setup
        z_safe = np.clip(z_val, -0.6, 0.0)
        z_msg.data = [float(z_safe)]
        
        if robot_choice.lower() == 'left' or robot_choice.lower() == 'l':
            self.left_arm_pub.publish(arm_msg)
            self.left_z_pub.publish(z_msg)
            print(f"Published to LEFT: J1={t1_rad:.2f}rad, J2={t2_rad:.2f}rad, Z={z_safe:.2f}m")
        elif robot_choice.lower() == 'right' or robot_choice.lower() == 'r':
            self.right_arm_pub.publish(arm_msg)
            self.right_z_pub.publish(z_msg)
            print(f"Published to RIGHT: J1={t1_rad:.2f}rad, J2={t2_rad:.2f}rad, Z={z_safe:.2f}m")
        else:
            print("Invalid robot choice.")
            return

        # Calculate and Show FK
        if self.kinematics:
            # Note: d3 input to FK usually expects positive extension or specific mapping. 
            # In kinematics.py: z = h_base - d3. 
            # If our z_input is directly the Z coordinate (e.g. -0.1), we might need to adjust.
            # But here we are controlling the JOINT d3. 
            # In the URDF/Gazebo, d3 is usually 0 at top and -0.2 (or similar) at bottom?
            # Let's assume z_val IS the joint value d3.
            # However, kinematics.py usually treats d3 as the extension from top.
            
            # Let's use the forward_kinematics method directly with the radian angles
            # Passing z_val as d3 (assuming kinematics expects the joint value or similar)
            # Actually kinematics.py says: z = h_base - d3
            # So if we pass the joint value, we get the world Z. 
            # We usually use positive d3 for extension in standard DH, but let's see.
            # For now, we print what the helper class checks.
            try:
                # We interpret the input z_val as the 'd3' parameter for the class
                # CAUTION: The user input 'z' might be intended as Z-coordinate or Joint Value.
                # Given "enter degrees of joints", likely they mean Joint Value.
                
                # In kinematics.py: z = h_base (2.0) - d3
                # If z_val is the prismatic joint value (e.g., -0.1), then d3 might be -z_val?
                # Let's just pass it and show the result.
                
                # Use absolute value for d3 if the class expects positive extension, 
                # but let's pass it raw first or check logic.
                # Logic in kinematics.py: z = 2.0 - d3.
                # If d3 is the joint value (e.g. 0.0 to 0.5), z is 2.0 to 1.5. 
                # The prismatic joint in simulation is usually negative (0 to -0.6).
                # So d3_calc = -z_val might be more appropriate if z_val is the sim joint.
                
                d3_calc = -z_safe 
                
                fk_x, fk_y, fk_z = self.kinematics.forward_kinematics(t1_rad, t2_rad, d3_calc)
                print(f"--- Forward Kinematics Verification ---")
                print(f"Calculated End-Effector Pose:")
                print(f"X: {fk_x:.4f} m")
                print(f"Y: {fk_y:.4f} m")
                print(f"Z: {fk_z:.4f} m")
                print(f"---------------------------------------")
                
            except Exception as e:
                print(f"Calculation Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DirectJointControl()
    
    print("\n--- Direct Joint Control Interface ---")
    print("Ranges:")
    print("  Joint 1: -148.9° to 148.9° (-2.6 to 2.6 rad)")
    print("  Joint 2: -171.8° to 171.8° (-3.0 to 3.0 rad)")
    print("  Z-Axis : 0.0m (Top) to -0.6m (Bottom)")
    print("Enter 'q' to quit.")
    
    try:
        while True:
            robot = input("\nSelect Robot (left/right): ").strip()
            if robot.lower() == 'q': break
            if robot.lower() not in ['left', 'right', 'l', 'r']:
                print("Unknown robot. Please enter 'left' or 'right'.")
                continue
                
            try:
                t1_input = input("Enter Joint 1 Angle (Degrees) [-148.9 to 148.9]: ")
                if t1_input.lower() == 'q': break
                t1 = float(t1_input)
                
                t2_input = input("Enter Joint 2 Angle (Degrees) [-171.8 to 171.8]: ")
                if t2_input.lower() == 'q': break
                t2 = float(t2_input)
                
                z_input  = input("Enter Z-Axis Joint Value [-0.6 to 0.0]: ")
                if z_input.lower() == 'q': break
                z  = float(z_input)
                
                # Manual validation warning
                if not (-149 <= t1 <= 149):
                    print("WARNING: J1 out of standard range (-149 to 149). Robot might hit limits.")
                if not (-172 <= t2 <= 172):
                    print("WARNING: J2 out of standard range (-172 to 172). Robot might hit limits.")
                if not (-0.6 <= z <= 0.0):
                    print("WARNING: Z out of range (-0.6 to 0.0). Constraining to limits.")
                
                node.publish_joints(robot, t1, t2, z)
                
            except ValueError:
                print("Invalid input. Please enter numbers.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
