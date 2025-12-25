#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty

# Help Message
msg = """
SCARA Keyboard Joint Control
---------------------------
Joint 1 (Shoulder):  Left / Right Arrow
Joint 2 (Elbow):     Up / Down Arrow
Joint 3 (Z-axis):    W (Up) / S (Down)

Q : Reset All to 0
ESC/Ctrl+C : Quit
---------------------------
"""

class KeyboardJointControl(Node):
    def __init__(self):
        super().__init__('keyboard_joint_control')
        
        # Publishers
        self.arm_pub = self.create_publisher(Float64MultiArray, '/scara_left_arm_controller/commands', 10)
        self.z_pub = self.create_publisher(Float64MultiArray, '/scara_left_gripper_controller/commands', 10)
        
        # Current simulated joint states (for incrementing)
        self.j1 = 0.0
        self.j2 = 0.0
        self.j3 = 0.1 # Start slightly down
        
        self.step = 0.05 # Radians/Meters per keypress
        
        self.settings = termios.tcgetattr(sys.stdin)
        print(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        if key == '\x1b': # ESC or Arrow sequence
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                changed = True
                if key == '\x1b[A': # Up Arrow
                    self.j2 += self.step
                elif key == '\x1b[B': # Down Arrow
                    self.j2 -= self.step
                elif key == '\x1b[C': # Right Arrow
                    self.j1 -= self.step
                elif key == '\x1b[D': # Left Arrow
                    self.j1 += self.step
                elif key == 'w' or key == 'W':
                    self.j3 -= self.step # Moving Z up
                elif key == 's' or key == 'S':
                    self.j3 += self.step # Moving Z down
                elif key == 'q' or key == 'Q':
                    self.j1, self.j2, self.j3 = 0.0, 0.0, 0.0
                elif key == '\x03' or key == '\x1b': # Ctrl+C or ESC
                    break
                else:
                    changed = False
                
                if changed:
                    # Clip values to safety limits (X-Y Revolute)
                    self.j1 = np.clip(self.j1, -2.6, 2.6)
                    self.j2 = np.clip(self.j2, -3.0, 3.0)
                    
                    # Clip Z-axis (Prismatic) to match URDF limits [-0.6, 0.0]
                    # In URDF 0.0 is top, -0.6 is bottom.
                    # W (Up) -> j3 increases towards 0.0
                    # S (Down) -> j3 decreases towards -0.6
                    self.j3 = np.clip(self.j3, -0.6, 0.0) 
                    
                    # Publish Arm
                    arm_msg = Float64MultiArray()
                    arm_msg.data = [float(self.j1), float(self.j2)]
                    self.arm_pub.publish(arm_msg)
                    
                    # Publish Z (Prismatic Joint)
                    z_msg = Float64MultiArray()
                    z_msg.data = [float(self.j3)]
                    self.z_pub.publish(z_msg)
                    
                    # Mapping: In Math d3 = -j3 (So 0 is top, 0.6 is bottom)
                    math_d3 = -self.j3
                    print(f"\r[CMD] J1: {self.j1:5.2f} | J2: {self.j2:5.2f} | J3(Sim): {self.j3:5.2f} (Math d3: {math_d3:5.2f})", end="")

        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    # Need numpy for clipping
    import numpy as np
    global np
    
    rclpy.init(args=args)
    node = KeyboardJointControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
