#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import threading
import math
import csv
import os
from datetime import datetime


# Import Kinematics
from two_scara_collaboration.kinematics import SCARAKinematics

class AutomationManager(Node):
    def __init__(self):
        super().__init__('automation_manager')
        
        # --- KINEMATICS ---
        # L1=1.5, L2=1.0 based on URDF analysis
        self.kinematics = SCARAKinematics(l1=1.5, l2=1.0)
        
        # --- PUBLISHERS (Direct Joint Control) ---
        self.arm_pub = self.create_publisher(Float64MultiArray, '/scara_left_arm_controller/commands', 10)
        self.quill_pub = self.create_publisher(Float64MultiArray, '/scara_left_gripper_controller/commands', 10)
        
        # Publisher for Right Robot (Direct Joint Control)
        self.right_arm_pub = self.create_publisher(Float64MultiArray, '/scara_right_arm_controller/commands', 10)
        self.right_quill_pub = self.create_publisher(Float64MultiArray, '/scara_right_gripper_controller/commands', 10)
        
        # Robot Base Position
        self.base_x = 2.0
        self.base_y = -2.0
        
        # State variables
        self.state = 'IDLE'
        self.target_block = None
        self.manual_target_world = (0.0, 0.0)
        self.move_start_time = 0
        
        self.right_state = 'IDLE' 
        self.right_start_delay = 0
        self.last_left_rel_target = None
        self.left_move_duration = 0.0
        
        # Logging Setup
        self.log_file = 'replication_data.csv'
        self.init_log_file()
        
        self.get_logger().info('Automation Manager Initialized. Type coordinates in terminal!')
        
        # Thread for Manual Input
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.timer = self.create_timer(0.5, self.control_loop)
        
        # Joint State Subscriber for Timing
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.has_started_physically = False
        self.settle_start_time = None
        self.actual_duration = None
        self.last_joint_positions = None

    def input_loop(self):
        time.sleep(1.0)
        while rclpy.ok():
            try:
                print("\n--- Enter Target Coordinates (World Frame) ---")
                print("Example: 2.0 -4.0 1.5")
                x_str = input("Enter World X: ")
                y_str = input("Enter World Y: ")
                z_str = input("Enter World Z (Height): ")
                
                try:
                    x = float(x_str)
                    y = float(y_str)
                    z = float(z_str)
                    self.manual_target_world = (x, y, z)
                    self.target_block = "Manual_Input"
                    print(f"Commanding Robot to World({x}, {y}, {z})...")
                except ValueError:
                    print("Invalid number. Try again.")
            except EOFError:
                break

    def joint_state_callback(self, msg):
        """Monitors position changes to detect when robot actually stops"""
        if self.state != 'MOVING':
            self.last_joint_positions = None
            return
            
        try:
            # Extract positions of Left Robot Joints
            current_positions = []
            valid_names = ['scara_left_joint_1', 'scara_left_joint_2', 'scara_left_gripper_joint']
            
            for name in valid_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    if len(msg.position) > idx:
                        current_positions.append(msg.position[idx])
            
            if len(current_positions) != 2:
                return

            current_time = time.time()
            
            # Initialize history
            if self.last_joint_positions is None:
                self.last_joint_positions = current_positions
                return

            # Calculate total change in position (motion)
            delta = sum([abs(c - l) for c, l in zip(current_positions, self.last_joint_positions)])
            self.last_joint_positions = current_positions # Update for next frame
            
            # Threshold for "Moving"
            MOTION_THRESHOLD = 0.0001
            
            # 1. Detect motion start
            if delta > MOTION_THRESHOLD: 
                self.has_started_physically = True
                self.settle_start_time = None 
                
            # 2. Detect motion stop
            if self.has_started_physically and delta < MOTION_THRESHOLD:
                if self.settle_start_time is None:
                    self.settle_start_time = current_time
                elif current_time - self.settle_start_time > 0.5:
                    # Stable for 0.5s -> Stopped
                    if self.actual_duration is None:
                        self.actual_duration = self.settle_start_time - self.move_start_time

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

    def execute_smooth_motion(self, arm_pub, quill_pub, start_joints, target_joints, duration=2.0):
        """Interpolates and publishes commands sequentially: XY first, then Z"""
        # Step 1: Translation (X-Y Plane)
        xy_start_time = time.time()
        print(f" -> Phase 1: TRANSLATION (Moving Arm to X,Y position)...")
        points = int(duration * 20)
        for i in range(points + 1):
            alpha = i / points
            j1 = start_joints[0] + alpha * (target_joints[0] - start_joints[0])
            j2 = start_joints[1] + alpha * (target_joints[1] - start_joints[1])
            
            msg = Float64MultiArray()
            msg.data = [j1, j2]
            arm_pub.publish(msg)
            time.sleep(1.0 / 20.0)
        
        # Step 2: Insertion (Z-Axis)
        print(f" -> Phase 2: INSERTION (Moving Gripper to Z height)...")
        for i in range(points + 1):
            alpha = i / points
            # Keep X/Y at final target
            j3_math = start_joints[2] + alpha * (target_joints[2] - start_joints[2])
            
            z_msg = Float64MultiArray()
            z_msg.data = [-j3_math] 
            quill_pub.publish(z_msg)
            time.sleep(1.0 / 20.0)
        
        total_move_time = time.time() - xy_start_time
        return total_move_time

    def control_loop(self):
        # LEFT ROBOT LOGIC
        if self.target_block == "Manual_Input" and self.state == 'IDLE':
            self.total_start_time = time.time()
            rel_x = self.manual_target_world[0] - self.base_x
            rel_y = self.manual_target_world[1] - self.base_y
            rel_z = self.manual_target_world[2]
            
            self.last_left_rel_target = (rel_x, rel_y, rel_z)
            
            dist = math.sqrt(rel_x**2 + rel_y**2)
            if dist > 2.5:
                 self.get_logger().warn(f"Target Out of Reach!")
                 self.target_block = None
                 return

            self.get_logger().info('Left Robot starting movement (Sequential XY then Z)...')
            self.left_exec_time = self.send_arm_command(rel_x, rel_y, rel_z)
            
            self.state = 'MOVING'
            self.move_start_time = time.time()
            self.has_started_physically = False
            self.actual_duration = None
            
            self.target_block = None
            
        elif self.state == 'MOVING':
            # Simplified for demo: wait for the smooth motion to end (4 seconds total)
            if (time.time() - self.move_start_time) > 4.5:
                self.left_end_time = time.time()
                self.get_logger().info(f'Left Robot Complete in {self.left_exec_time:.2f}s.')
                self.state = 'IDLE'
                
                # Schedule Right Robot
                self.right_state = 'WAITING'
                self.right_start_delay = time.time()

        # RIGHT ROBOT LOGIC (Replication)
        if self.right_state == 'WAITING':
            # The delay after left robot finishes
            delay = time.time() - self.left_end_time
            if delay >= 0.5: # 0.5 second safety delay
                self.right_actual_start = time.time()
                self.get_logger().info(f'Right Robot starting replication after {delay:.2f}s delay...')
                
                rx, ry, rz = self.last_left_rel_target
                self.right_exec_time = self.send_right_arm(rx, ry, rz)
                
                self.right_state = 'MOVING'
                self.right_move_start = time.time()

        elif self.right_state == 'MOVING':
             if time.time() - self.right_move_start > 4.5:
                 total_combined_time = time.time() - self.total_start_time
                 self.get_logger().info('--- PERFORMANCE SUMMARY ---')
                 self.get_logger().info(f'Left Robot Time:  {self.left_exec_time:.2f}s')
                 self.get_logger().info(f'Inter-Robot Delay: {0.50:.2f}s')
                 self.get_logger().info(f'Right Robot Time: {self.right_exec_time:.2f}s')
                 self.get_logger().info(f'TOTAL MISSION TIME: {total_combined_time:.2f}s')
                 self.get_logger().info('---------------------------')
                 self.right_state = 'IDLE'

    def send_arm_command(self, x, y, z):
        theta1, theta2, d3 = self.kinematics.inverse_kinematics(x, y, z, elbow='up')
        if theta1 is None: return 0.0
        
        if self.last_joint_positions: start_joints = self.last_joint_positions
        else: start_joints = [0.0, 0.0, 0.0] 

        target_joints = [float(theta1), float(theta2), float(d3)]
        return self.execute_smooth_motion(self.arm_pub, self.quill_pub, start_joints, target_joints, duration=2.0)
        
    def send_right_arm(self, x, y, z):
        theta1, theta2, d3 = self.kinematics.inverse_kinematics(x, y, z, elbow='up')
        if theta1 is None: return 0.0
             
        start_joints = [0.0, 0.0, 0.0] 
        target_joints = [float(theta1), float(theta2), float(d3)]
        return self.execute_smooth_motion(self.right_arm_pub, self.right_quill_pub, start_joints, target_joints, duration=2.0)

    def init_log_file(self):
        """Creates CSV file with headers if it doesn't exist or is outdated"""
        expected_header = [
            'Timestamp', 
            'Left_Input_World_X', 'Left_Input_World_Y',
            'Left_Base_X', 'Left_Base_Y',
            'Calculated_Rel_X', 'Calculated_Rel_Y', 
            'Left_Duration_Sec',
            'Right_Base_X', 'Right_Base_Y',
            'Right_Output_World_X', 'Right_Output_World_Y'
        ]
        
        should_write_header = False
        
        if not os.path.exists(self.log_file):
            should_write_header = True
        else:
            # Check if header matches
            with open(self.log_file, 'r') as f:
                header_line = f.readline().strip()
                current_cols = header_line.split(',')
                if len(current_cols) != len(expected_header):
                    self.get_logger().warn("CSV Header mismatch detecting! Recreating log file...")
                    should_write_header = True
                    
        if should_write_header:
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(expected_header)
                
    def log_calculation(self, rx, ry, rb_x, rb_y, rt_x, rt_y, duration):
        """Saves the replication calculation details to CSV"""
        # Reconstruct Left Input for records
        left_in_x = self.base_x + rx
        left_in_y = self.base_y + ry
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            data = [
                timestamp,
                f"{left_in_x:.2f}", f"{left_in_y:.2f}",
                self.base_x, self.base_y,
                f"{rx:.2f}", f"{ry:.2f}",
                f"{duration:.2f}",
                rb_x, rb_y,
                f"{rt_x:.2f}", f"{rt_y:.2f}"
            ]
            writer.writerow(data)
        self.get_logger().info(f"Calculation saved to {self.log_file}")

def main(args=None):
    rclpy.init(args=args)
    node = AutomationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
