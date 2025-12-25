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
        
        self.last_joint_positions = None
        self.last_right_joint_positions = None
        
        self.get_logger().info('Automation Manager Initialized. Type coordinates in terminal!')
        
        # Thread for Manual Input
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.timer = self.create_timer(0.1, self.control_loop) # Fast response
        
        # Joint State Subscriber
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.has_started_physically = False

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
        """Monitors position changes for both robots"""
        try:
            # Helper to extract joint positions by prefix [Joint1, Joint2, Gripper]
            def get_positions(prefix):
                pos = []
                # Match URDF joint names
                names = [f'{prefix}_joint_1', f'{prefix}_joint_2', f'{prefix}_gripper_joint']
                for name in names:
                    if name in msg.name:
                        idx = msg.name.index(name)
                        if len(msg.position) > idx:
                            pos.append(msg.position[idx])
                return pos

            left_pos = get_positions('scara_left')
            right_pos = get_positions('scara_right')

            if len(left_pos) == 3:
                self.last_joint_positions = left_pos
            if len(right_pos) == 3:
                self.last_right_joint_positions = right_pos

        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

    def execute_smooth_motion(self, arm_pub, quill_pub, start_joints, target_joints, duration=2.0):
        """Sequential Motion with Active Locking: 1. Translation (XY), 2. Insertion (Z), 3. Withdrawal (Z)"""
        xy_start_time = time.time()
        
        # Step 1: Translation (X-Y Plane) - LOCK Z at start position
        print(f" -> Phase 1: TRANSLATION (Moving Arm to X,Y position)...")
        points = int(duration * 20)
        for i in range(points + 1):
            alpha = i / points
            j1 = start_joints[0] + alpha * (target_joints[0] - start_joints[0])
            j2 = start_joints[1] + alpha * (target_joints[1] - start_joints[1])
            
            # Active Locking: Send J1, J2 AND keep Z at start_joints[2]
            msg = Float64MultiArray()
            msg.data = [j1, j2]
            arm_pub.publish(msg)
            
            z_msg = Float64MultiArray()
            z_msg.data = [-start_joints[2]] # Hold start Z (Z is inverted in URDF command usually)
            quill_pub.publish(z_msg)
            
            time.sleep(1.0 / 20.0)
        
        # Step 2: Insertion (Z-Axis) - LOCK Arm at final target_joints[0,1]
        print(f" -> Phase 2: INSERTION (Moving Gripper to target Z)...")
        for i in range(points + 1):
            alpha = i / points
            j3_math = start_joints[2] + alpha * (target_joints[2] - start_joints[2])
            
            # Active Locking: Send new Z AND force J1, J2 to stay at target
            z_msg = Float64MultiArray()
            z_msg.data = [-j3_math] 
            quill_pub.publish(z_msg)
            
            msg = Float64MultiArray()
            msg.data = [target_joints[0], target_joints[1]] # Hold final XY
            arm_pub.publish(msg)
            
            time.sleep(1.0 / 20.0)

        # Step 3: Withdrawal (Return Z) - LOCK Arm at final target_joints[0,1]
        print(f" -> Phase 3: WITHDRAWAL (Returning Gripper to original height)...")
        for i in range(points + 1):
            alpha = i / points
            j3_math = target_joints[2] - alpha * (target_joints[2] - start_joints[2])
            
            # Active Locking: Send returning Z AND force J1, J2 to stay at target
            z_msg = Float64MultiArray()
            z_msg.data = [-j3_math] 
            quill_pub.publish(z_msg)
            
            msg = Float64MultiArray()
            msg.data = [target_joints[0], target_joints[1]] # Hold final XY
            arm_pub.publish(msg)
            
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

            world_coord = self.manual_target_world
            self.get_logger().info(f'World Target Received: {world_coord}')
            self.get_logger().info(f'Left Robot (Base: {self.base_x}, {self.base_y}) -> Relative Target: ({rel_x:.2f}, {rel_y:.2f}, {rel_z:.2f})')
            self.get_logger().info('Left Robot starting movement (XY -> Insertion -> Withdrawal)...')
            
            self.state = 'MOVING'
            self.move_start_time = time.time()
            
            # BLOCKING CALL - Executing motion
            self.left_exec_time = self.send_arm_command(rel_x, rel_y, rel_z)
            
            # Movement finished
            self.left_end_time = time.time()
            self.get_logger().info(f'Left Robot Complete in {self.left_exec_time:.2f}s.')
            self.state = 'IDLE'
            
            # Schedule Right Robot Replication
            self.right_state = 'WAITING'
            self.target_block = None
            
        elif self.state == 'MOVING':
            # This is a safety state; logic handled in blocking call above
            pass

        # RIGHT ROBOT LOGIC (Replication)
        if self.right_state == 'WAITING':
            delay = time.time() - self.left_end_time
            if delay >= 0.5: # Safety delay
                rx, ry, rz = self.last_left_rel_target
                right_base_x, right_base_y = 2.0, 2.0
                
                self.get_logger().info(f'Right Robot starting replication after {delay:.2f}s delay...')
                self.get_logger().info(f'Right Robot (Base: {right_base_x}, {right_base_y}) -> Relative Target: ({rx:.2f}, {ry:.2f}, {rz:.2f})')
                
                self.right_state = 'MOVING'
                self.right_move_start = time.time()
                
                # BLOCKING CALL
                self.right_exec_time = self.send_right_arm(rx, ry, rz)
                
                # Replication finished
                total_combined_time = time.time() - self.total_start_time
                world_coord = self.manual_target_world
                
                self.get_logger().info('--- PERFORMANCE SUMMARY ---')
                self.get_logger().info(f'Target World Coord: {world_coord}')
                self.get_logger().info(f'Relative Movement:  ({rx:.2f}, {ry:.2f}, {rz:.2f})')
                self.get_logger().info(f'Left Robot Time:    {self.left_exec_time:.2f}s (3-Phase)')
                self.get_logger().info(f'Inter-Robot Delay:  {delay:.2f}s')
                self.get_logger().info(f'Right Robot Time:   {self.right_exec_time:.2f}s (3-Phase)')
                self.get_logger().info(f'TOTAL MISSION TIME:  {total_combined_time:.2f}s')
                self.get_logger().info('---------------------------')
                
                self.right_state = 'IDLE'

    def send_arm_command(self, x, y, z):
        theta1, theta2, d3 = self.kinematics.inverse_kinematics(x, y, z, elbow='up')
        if theta1 is None: return 0.0
        
        # Current state from feedback
        if self.last_joint_positions: 
            start_joints = self.last_joint_positions
        else: 
            start_joints = [0.0, 0.0, 0.0] 

        target_joints = [float(theta1), float(theta2), float(d3)]
        return self.execute_smooth_motion(self.arm_pub, self.quill_pub, start_joints, target_joints, duration=2.0)
        
    def send_right_arm(self, x, y, z):
        theta1, theta2, d3 = self.kinematics.inverse_kinematics(x, y, z, elbow='up')
        if theta1 is None: return 0.0
             
        # Current state from feedback
        if self.last_right_joint_positions:
            start_joints = self.last_right_joint_positions
        else:
            start_joints = [0.0, 0.0, 0.0] 

        target_joints = [float(theta1), float(theta2), float(d3)]
        return self.execute_smooth_motion(self.right_arm_pub, self.right_quill_pub, start_joints, target_joints, duration=2.0)

    def init_log_file(self):
        """Creates CSV file with headers if it doesn't exist"""
        expected_header = [
            'Timestamp', 'Left_In_X', 'Left_In_Y', 'Rel_X', 'Rel_Y', 'Duration', 'Right_Out_X', 'Right_Out_Y'
        ]
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(expected_header)
                
    def log_calculation(self, rx, ry, rb_x, rb_y, rt_x, rt_y, duration):
        """Saves details to CSV"""
        left_in_x = self.base_x + rx
        left_in_y = self.base_y + ry
        timestamp = datetime.now().strftime("%H:%M:%S")
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, f"{left_in_x:.2f}", f"{left_in_y:.2f}", f"{rx:.2f}", f"{ry:.2f}", f"{duration:.2f}", f"{rt_x:.2f}", f"{rt_y:.2f}"])

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
