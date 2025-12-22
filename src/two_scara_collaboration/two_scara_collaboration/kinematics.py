#!/usr/bin/env python3
"""
Kinematics module for SCARA robots - Forward and Inverse Kinematics
Includes Optimization-based Trajectory Generation
"""

import numpy as np
import math

class SCARAKinematics:
    """
    Forward and Inverse Kinematics for 3D SCARA robot
    Includes Dynamics calculations (Inertia, Gravity, Torque)
    """
    
    def __init__(self, l1=1.5, l2=1.0, h_base=2.0, mass_l1=5.0, mass_l2=3.0, mass_payload=1.0):
        """
        Initialize SCARA kinematics with physical properties
        
        Args:
            l1, l2: Length of links (m)
            h_base: Height of the base (m)
            mass_l1, mass_l2: Mass of the links (kg)
            mass_payload: Mass of the object being carried (kg)
        """
        self.l1 = l1
        self.l2 = l2
        self.h_base = h_base
        
        # Physics Parameters
        self.m1 = mass_l1
        self.m2 = mass_l2
        self.m_p = mass_payload
        self.g = 9.81  # Gravity m/s^2
    
    def forward_kinematics(self, theta1, theta2, d3):
        """
        --- 3D FORWARD KINEMATICS EQUATION ---
        Input: Joint Angles (theta1, theta2) and Prismatic Extension (d3)
        Output: Cartesian Position (x, y, z)
        """
        # Planar X-Y Equations (Standard SCARA)
        x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        
        # 3D Z Equation (Decoupled Prismatic Joint)
        z = self.h_base - d3  # Assumption: d3=0 is top, d3=max is bottom
        
        return x, y, z
    
    def inverse_kinematics(self, x, y, z, elbow='up'):
        """
        --- 3D INVERSE KINEMATICS EQUATION ---
        Input: Target Destination (x, y, z)
        Output: Required Joint Values (theta1, theta2, d3)
        """
        # 1. Solve for Prismatic Joint d3
        d3 = self.h_base - z
        
        # 2. Solve for Planar joints theta1, theta2 using Law of Cosines
        d_sq = x**2 + y**2
        d = np.sqrt(d_sq)
        
        if d > (self.l1 + self.l2) or d < abs(self.l1 - self.l2):
            return None, None, None
        
        cos_theta2 = (d_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        
        if elbow == 'up':
            theta2 = np.arccos(cos_theta2)
        else:
            theta2 = -np.arccos(cos_theta2)
        
        alpha = np.arctan2(y, x)
        beta = np.arctan2(self.l2 * np.sin(theta2), 
                         self.l1 + self.l2 * np.cos(theta2))
        
        theta1 = alpha - beta
        
        return theta1, theta2, d3

    def calculate_dynamics(self, theta1, theta2, q_ddot):
        """
        --- ROBOT DYNAMICS (Lagrangian Mechanics) ---
        Formula: Torque = M(q) * q_ddot + G(q)
        """
        # --- A. Inertia / Mass Matrix M(q) ---
        # M11 is the Moment of Inertia of the entire arm around the base
        # It depends on theta2 (how 'folded' the arm is)
        M11 = (self.m1 + self.m2 + self.m_p) * self.l1**2 + \
              (self.m2 + self.m_p) * (self.l2**2 + 2 * self.l1 * self.l2 * np.cos(theta2))
        M22 = (self.m2 + self.m_p) * self.l2**2
        M33 = self.m1 + self.m2 + self.m_p # Fixed mass for vertical motion
        
        # --- B. Gravity Vector G(q) ---
        G1 = 0.0 # No gravity in X-Y plane for horizontal SCARA
        G2 = 0.0 # No gravity in X-Y plane for horizontal SCARA
        G3 = (self.m2 + self.m_p) * self.g # Vertical gravity force
        
        # --- C. Final Torque/Force Calculation ---
        torque1 = M11 * q_ddot[0] + G1
        torque2 = M22 * q_ddot[1] + G2
        force3  = M33 * q_ddot[2] + G3
        
        return [torque1, torque2, force3]
    
    def jacobian(self, theta1, theta2):
        """2D Jacobian for horizontal motion"""
        c1 = np.cos(theta1)
        s1 = np.sin(theta1)
        c12 = np.cos(theta1 + theta2)
        s12 = np.sin(theta1 + theta2)
        
        J = np.array([
            [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12],
            [self.l1 * c1 + self.l2 * c12, self.l2 * c12]
        ])
        return J

    def generate_straight_line_path(self, start_joints, end_coords, duration=2.0, dt=0.05):
        """
        Generates 3D trajectory minimizing joint velocity
        """
        curr_theta1, curr_theta2, curr_d3 = start_joints
        target_x, target_y, target_z = end_coords
        
        # Current Cartesian
        start_x, start_y, start_z = self.forward_kinematics(curr_theta1, curr_theta2, curr_d3)
        
        # Velocity vector
        vx = (target_x - start_x) / duration
        vy = (target_y - start_y) / duration
        vz = (target_z - start_z) / duration
        
        trajectory_points = []
        steps = int(duration / dt)
        
        q = np.array([curr_theta1, curr_theta2])
        d3 = curr_d3
        
        for i in range(steps + 1):
            t = i * dt
            
            # Store point
            trajectory_points.append((t, q[0], q[1], d3))
            
            # Differential Kinematics for X-Y
            J = self.jacobian(q[0], q[1])
            try:
                J_inv = np.linalg.pinv(J)
                q_dot = np.dot(J_inv, np.array([vx, vy]))
            except:
                q_dot = np.zeros(2)
            
            # Integration
            q = q + q_dot * dt
            d3 = d3 - vz * dt # d3 decreases to increase Z height
            
        return trajectory_points

    
    def is_singular(self, theta1, theta2, threshold=0.01):
        """
        Check if robot is near singular configuration
        
        Args:
            theta1: Angle of joint 1
            theta2: Angle of joint 2
            threshold: Determinant threshold
            
        Returns:
            bool: True if singular
        """
        J = self.jacobian(theta1, theta2)
        det = np.linalg.det(J)
        
        return abs(det) < threshold


class CollisionDetector:
    """
    Collision detection between SCARA robots
    """
    
    def __init__(self, l1=0.5, l2=0.5):
        """Initialize collision detector"""
        self.l1 = l1
        self.l2 = l2
        self.robot_radius = 0.05  # Approximate robot link radius
    
    def get_link_positions(self, theta1, theta2, base_x=0, base_y=0):
        """
        Get positions of all points along robot links
        
        Args:
            theta1, theta2: Joint angles
            base_x, base_y: Base position
            
        Returns:
            List of (x, y) positions along the arm
        """
        positions = [(base_x, base_y)]
        
        # Joint 1 position
        j1_x = base_x + self.l1 * np.cos(theta1)
        j1_y = base_y + self.l1 * np.sin(theta1)
        positions.append((j1_x, j1_y))
        
        # End effector position
        ee_x = j1_x + self.l2 * np.cos(theta1 + theta2)
        ee_y = j1_y + self.l2 * np.sin(theta1 + theta2)
        positions.append((ee_x, ee_y))
        
        return positions
    
    def check_collision(self, theta1_left, theta2_left, theta1_right, theta2_right,
                       base_left=(0.3, 0), base_right=(-0.3, 0)):
        """
        Check if two SCARA robots collide
        
        Args:
            Joint angles for both robots
            Base positions of both robots
            
        Returns:
            bool: True if collision detected
        """
        positions_left = self.get_link_positions(theta1_left, theta2_left, *base_left)
        positions_right = self.get_link_positions(theta1_right, theta2_right, *base_right)
        
        # Check distance between all pairs of segments
        min_distance = float('inf')
        
        for i in range(len(positions_left) - 1):
            for j in range(len(positions_right) - 1):
                dist = self._segment_distance(
                    positions_left[i], positions_left[i+1],
                    positions_right[j], positions_right[j+1]
                )
                min_distance = min(min_distance, dist)
        
        # Collision if distance is less than sum of radii
        return min_distance < (2 * self.robot_radius)
    
    def _segment_distance(self, p1, p2, p3, p4):
        """
        Calculate minimum distance between two line segments
        """
        def point_to_segment(p, a, b):
            ab = np.array(b) - np.array(a)
            ap = np.array(p) - np.array(a)
            ab_ab = np.dot(ab, ab)
            
            if ab_ab < 1e-10:
                return np.linalg.norm(ap)
            
            t = np.dot(ap, ab) / ab_ab
            t = np.clip(t, 0, 1)
            
            closest = np.array(a) + t * ab
            return np.linalg.norm(np.array(p) - closest)
        
        d1 = point_to_segment(p1, p3, p4)
        d2 = point_to_segment(p2, p3, p4)
        d3 = point_to_segment(p3, p1, p2)
        d4 = point_to_segment(p4, p1, p2)
        
        return min(d1, d2, d3, d4)
