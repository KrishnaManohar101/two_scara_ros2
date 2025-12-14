#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import time
import threading
import math

class AutomationManager(Node):
    def __init__(self):
        super().__init__('automation_manager')
        
        # Publisher for Left Robot
        self.arm_pub = self.create_publisher(PoseStamped, '/scara_left/target_pose', 10)
        self.quill_pub = self.create_publisher(Float64MultiArray, '/scara_left_gripper_controller/commands', 10)
        
        # Publisher for Right Robot
        self.right_arm_pub = self.create_publisher(PoseStamped, '/scara_right/target_pose', 10)
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
        
        self.get_logger().info('Automation Manager Initialized. Type coordinates in terminal!')
        
        # Thread for Manual Input
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.timer = self.create_timer(0.5, self.control_loop)

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
            self.target_block = None # Clear trigger
            
        elif self.state == 'MOVING':
            # Wait for 5 seconds for motion to complete
            if time.time() - self.move_start_time > 5.0:
                self.get_logger().info('Left: Reached. Scheduling Right Robot in 10s...')
                self.state = 'IDLE'
                
                # Schedule Right Robot
                self.right_state = 'WAITING'
                self.right_start_delay = time.time()

        # RIGHT ROBOT LOGIC (Replication)
        if self.right_state == 'WAITING':
            if time.time() - self.right_start_delay > 10.0:
                self.get_logger().info('Right: Replication Started! Mimicking Left Robot...')
                
                # Retrieve Relative Move
                rx, ry = self.last_left_rel_target
                
                # Calculate Right Robot Global Target (RightBase + Relative)
                right_base_x = 2.0
                right_base_y = 2.0
                
                target_x = right_base_x + rx
                target_y = right_base_y + ry
                
                self.get_logger().info(f'Right: Base({right_base_x}, {right_base_y}) + Rel({rx:.2f}, {ry:.2f}) -> Global({target_x:.2f}, {target_y:.2f})')
                self.send_right_arm(target_x, target_y)
                
                self.right_state = 'MOVING'
                self.right_move_start = time.time()

        elif self.right_state == 'MOVING':
             if time.time() - self.right_move_start > 5.0:
                 self.get_logger().info('Right: Done.')
                 self.right_state = 'IDLE'

    def send_arm_command(self, x, y):
        msg = PoseStamped()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        self.arm_pub.publish(msg)
        
    def send_right_arm(self, x, y):
        msg = PoseStamped()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        self.right_arm_pub.publish(msg)

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
