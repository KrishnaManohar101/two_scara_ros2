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
        
        # Kinematics Solver (L1=1.5, L2=1.0 based on URDF)
        self.kinematics = SCARAKinematics(l1=1.5, l2=1.0)
        
        # Publisher for Left Robot (Direct Joint Control)
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
                print("Example: 2.0 -4.0")
                x_str = input("Enter X: ")
                y_str = input("Enter Y: ")
                
                try:
                    x = float(x_str)
                    y = float(y_str)
                    self.manual_target_world = (x, y)
                    self.target_block = "Manual_Input"
                    print(f"Commanding Robot to World({x}, {y})...")
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
            valid_names = ['scara_left_joint_1', 'scara_left_joint_2']
            
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

    def control_loop(self):
        # LEFT ROBOT LOGIC
        if self.target_block == "Manual_Input" and self.state == 'IDLE':
            # Transform World -> Robot Frame
            rel_x = self.manual_target_world[0] - self.base_x
            rel_y = self.manual_target_world[1] - self.base_y
            
            self.last_left_rel_target = (rel_x, rel_y)
            
            # Check Reachability (Max Reach ~2.5m)
            dist = math.sqrt(rel_x**2 + rel_y**2)
            if dist > 2.5:
                 self.get_logger().warn(f"Target Out of Reach! Dist: {dist:.2f}m > Max: 2.5m. Try closer to Base(2.0, -2.0).")
                 self.target_block = None
                 return

            self.get_logger().info(f'Left: Moving to World{self.manual_target_world} (Rel: {rel_x:.2f}, {rel_y:.2f})...')
            self.send_arm_command(rel_x, rel_y)
            
            self.state = 'MOVING'
            self.move_start_time = time.time()
            # Reset Timing Flags
            self.has_started_physically = False
            self.settle_start_time = None
            self.actual_duration = None
            self.last_joint_positions = None
            
            self.target_block = None # Clear trigger
            
        elif self.state == 'MOVING':
            # Wait for 5 seconds for motion to complete
            # Wait for physical completion or max timeout (whichever comes first)
            is_done = False
            
            # If we detected a stop physically
            if self.actual_duration is not None:
                is_done = True
            # Or if it's been too long (backup timeout 8s)
            elif (time.time() - self.move_start_time) > 8.0:
                is_done = True
                self.actual_duration = 8.0

            if is_done:
                # Determine what to log
                log_time = self.actual_duration
                self.left_move_duration = log_time
                
                self.get_logger().info(f'Left: Motion Complete. Actual Time: {log_time:.3f}s.')
                self.state = 'IDLE'
                
                # Schedule Right Robot
                self.right_state = 'WAITING'
                self.right_start_delay = time.time()

        # RIGHT ROBOT LOGIC (Replication)
        if self.right_state == 'WAITING':
            if time.time() - self.right_start_delay > 0.0:
                self.get_logger().info('Right: Replication Started! Mimicking Left Robot...')
                
                # Retrieve Relative Move
                rx, ry = self.last_left_rel_target
                
                # Calculate Right Robot Global Target (RightBase + Relative)
                right_base_x = 2.0
                right_base_y = 2.0
                
                target_x = right_base_x + rx
                target_y = right_base_y + ry
                
                self.get_logger().info(f'Right: Base({right_base_x}, {right_base_y}) + Rel({rx:.2f}, {ry:.2f}) -> Global({target_x:.2f}, {target_y:.2f})')
                
                # Log Data
                self.log_calculation(rx, ry, right_base_x, right_base_y, target_x, target_y, self.left_move_duration)
                
                self.send_right_arm(rx, ry)
                
                self.right_state = 'MOVING'
                self.right_move_start = time.time()

        elif self.right_state == 'MOVING':
             if time.time() - self.right_move_start > 5.0:
                 self.get_logger().info('Right: Done.')
                 self.right_state = 'IDLE'

    def send_arm_command(self, x, y):
        # Inverse Kinematics
        theta1, theta2 = self.kinematics.inverse_kinematics(x, y, elbow='up')
        
        if theta1 is None:
            self.get_logger().error("Target Unreachable (IK Failed)")
            return

        # Execute Smooth Motion (Left)
        # We need to know current position. If unknown (first move), assume 0.0 or read from self.last_joint_positions
        if self.last_joint_positions:
            start_joints = self.last_joint_positions
        else:
            start_joints = [0.0, 0.0] # Default start

        target_joints = [float(theta1), float(theta2)]
        self.execute_smooth_motion(self.arm_pub, start_joints, target_joints, duration=2.0)
        
    def send_right_arm(self, x, y):
        # Inverse Kinematics
        theta1, theta2 = self.kinematics.inverse_kinematics(x, y, elbow='up')
        
        if theta1 is None:
             self.get_logger().error("Right Target Unreachable (IK Failed)")
             return
             
        # For Right Robot, we don't track its state in self.last_joint_positions (that's for Left).
        # We will assume it starts from "Home" or where we last sent it. 
        # Ideally we should subscribe to right robot states too. 
        # usage: Just snap for now? No, user asked for smooth.
        # Let's approximate start as 0,0 or just do a simple interpolation
        start_joints = [0.0, 0.0] 
        target_joints = [float(theta1), float(theta2)]
        
        self.execute_smooth_motion(self.right_arm_pub, start_joints, target_joints, duration=2.0)

    def execute_smooth_motion(self, publisher, start_joints, target_joints, duration=2.0):
        """Interpolates and publishes commands over time"""
        points = int(duration * 20) # 20 Hz
        
        for i in range(points + 1):
            alpha = i / points
            
            # Linear Interpolation
            j1 = start_joints[0] + alpha * (target_joints[0] - start_joints[0])
            j2 = start_joints[1] + alpha * (target_joints[1] - start_joints[1])
            
            msg = Float64MultiArray()
            msg.data = [j1, j2]
            publisher.publish(msg)
            time.sleep(1.0 / 20.0) # wait 50ms

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
