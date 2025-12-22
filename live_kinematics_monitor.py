#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import os
import sys
import threading

# Add src to path
sys.path.append(os.path.join(os.getcwd(), 'src/two_scara_collaboration'))
from two_scara_collaboration.kinematics import SCARAKinematics

class PremiumDashboard(Node):
    def __init__(self):
        super().__init__('premium_dashboard')
        
        # --- Live Physical State ---
        self.mass = 2.0
        self.gravity = 9.81
        self.l1, self.l2 = 1.5, 1.0
        
        self.engine = SCARAKinematics(l1=self.l1, l2=self.l2, mass_payload=self.mass)
        self.engine.g = self.gravity
        
        # --- Data Buffers ---
        self.joint_names = ['scara_left_joint_1', 'scara_left_joint_2', 'scara_left_gripper_joint']
        self.q = [0.0, 0.0, 0.0] # J1 (rad), J2 (rad), J3 (sim m)
        
        # --- ROS Setup ---
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_timer(0.05, self.update_display) # 20Hz Refresh
        
        # --- Tuning Thread ---
        self.thread = threading.Thread(target=self.kb_listener, daemon=True)
        self.thread.start()

    def joint_cb(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                self.q[i] = msg.position[msg.name.index(name)]

    def kb_listener(self):
        import tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                char = sys.stdin.read(1)
                if char.lower() == 'm': self.mass += 0.5
                if char.lower() == 'n': self.mass = max(0.1, self.mass - 0.5)
                if char.lower() == 'g': self.gravity += 0.1
                if char.lower() == 'f': self.gravity = max(0.0, self.gravity - 0.1)
                self.engine = SCARAKinematics(l1=self.l1, l2=self.l2, mass_payload=self.mass)
                self.engine.g = self.gravity
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def draw_bar(self, val, min_v, max_v, width=15):
        percent = (val - min_v) / (max_v - min_v)
        percent = max(0, min(1, percent))
        filled = int(width * percent)
        return "[" + "#" * filled + "-" * (width - filled) + "]"

    def update_display(self):
        # 1. KINEMATICS CALCULATIONS
        math_d3 = -self.q[2] # Sim to Math mapping
        # FORWARD KINEMATICS (FK)
        x, y, z = self.engine.forward_kinematics(self.q[0], self.q[1], math_d3)
        # INVERSE KINEMATICS (IK)
        ik_q1, ik_q2, ik_d3 = self.engine.inverse_kinematics(x, y, z)
        
        # 2. DYNAMICS CALCULATIONS (Inverse Dynamics: Motion -> Torque)
        # We assume a nominal acceleration to show "Required Force"
        nominal_accel = [1.0, 1.0, 1.0] 
        torques = self.engine.calculate_dynamics(self.q[0], self.q[1], nominal_accel)
        
        # 3. RENDER (Using ANSI to reduce flicker)
        sys.stdout.write("\033[H") # Move to top
        out = []
        out.append("\033[94m" + "╔══════════════════════════════════════════════════════════╗" + "\033[0m")
        out.append("\033[94m" + "║         SCARA MASTER COMMAND TELEMETRY DASHBOARD         ║" + "\033[0m")
        out.append("\033[94m" + "╠══════════════════════════════════════════════════════════╣" + "\033[0m")
        
        # --- PHYSICAL CONSTANTS ---
        out.append(f"  PAYLOAD MASS:  {self.mass:5.1f} kg  {self.draw_bar(self.mass, 0, 10)}  (Key: M/N)")
        out.append(f"  GRAVITY CONST: {self.gravity:5.2f} m/s² {self.draw_bar(self.gravity, 0, 20)}  (Key: G/F)")
        out.append("╟──────────────────────────────────────────────────────────╢")

        # --- KINEMATICS SECTION ---
        out.append("\033[92m" + f"  [FORWARD KINEMATICS]          [INVERSE KINEMATICS CHECK]" + "\033[0m")
        out.append(f"  X-COORD: {x:6.2f} m             IK-J1: {ik_q1 if ik_q1 else 0.0:6.2f} rad")
        out.append(f"  Y-COORD: {y:6.2f} m             IK-J2: {ik_q2 if ik_q2 else 0.0:6.2f} rad")
        out.append(f"  Z-COORD: {z:6.2f} m             IK-J3: {ik_d3 if ik_d3 else 0.0:6.2f} m")
        out.append("╟──────────────────────────────────────────────────────────╢")

        # --- DYNAMICS SECTION ---
        out.append("\033[93m" + f"  [INVERSE DYNAMICS: FORCE/TORQUE ESTIMATION]" + "\033[0m")
        out.append(f"  J1 SHOULDER TORQUE: {torques[0]:7.2f} Nm  {self.draw_bar(abs(torques[0]),0,100)}")
        out.append(f"  J2 ELBOW TORQUE:    {torques[1]:7.2f} Nm  {self.draw_bar(abs(torques[1]),0,50)}")
        out.append(f"  J3 VERTICAL FORCE:  {torques[2]:7.2f} N   {self.draw_bar(abs(torques[2]),0,150)}")
        
        # --- RAW INPUTSection ---
        out.append("╟──────────────────────────────────────────────────────────╢")
        out.append(f"  LIVE JOINT INPUT:  [{self.q[0]:5.2f}, {self.q[1]:5.2f}, {self.q[2]:5.2f}]")
        out.append("\033[94m" + "╚══════════════════════════════════════════════════════════╝" + "\033[0m")
        
        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()

def main():
    rclpy.init()
    node = PremiumDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
