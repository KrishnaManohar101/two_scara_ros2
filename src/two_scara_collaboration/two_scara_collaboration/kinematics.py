#!/usr/bin/env python3
"""
Kinematics module for SCARA robots - Forward and Inverse Kinematics
"""

import numpy as np
import math

class SCARAKinematics:
    """
    Forward and Inverse Kinematics for SCARA robot
    """
    
    def __init__(self, l1=0.5, l2=0.5):
        """
        Initialize SCARA kinematics
        
        Args:
            l1: Length of first link (m)
            l2: Length of second link (m)
        """
        self.l1 = l1
        self.l2 = l2
    
    def forward_kinematics(self, theta1, theta2):
        """
        Calculate end effector position from joint angles
        
        Args:
            theta1: Angle of joint 1 (radians)
            theta2: Angle of joint 2 (radians)
            
        Returns:
            x, y: End effector position
        """
        x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        
        return x, y
    
    def inverse_kinematics(self, x, y, elbow='up'):
        """
        Calculate joint angles from end effector position
        
        Args:
            x: X position of end effector
            y: Y position of end effector
            elbow: 'up' or 'down' - which solution to use
            
        Returns:
            theta1, theta2: Joint angles (radians)
            
        Returns None if position is unreachable
        """
        # Calculate distance from origin to target
        d = np.sqrt(x**2 + y**2)
        
        # Check if target is reachable
        if d > (self.l1 + self.l2) or d < abs(self.l1 - self.l2):
            return None, None
        
        # Law of cosines to find theta2
        cos_theta2 = (d**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        
        # Clamp to valid range to avoid numerical errors
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        
        if elbow == 'up':
            theta2 = np.arccos(cos_theta2)
        else:
            theta2 = -np.arccos(cos_theta2)
        
        # Find theta1 using atan2
        alpha = np.arctan2(y, x)
        beta = np.arctan2(self.l2 * np.sin(theta2), 
                         self.l1 + self.l2 * np.cos(theta2))
        
        theta1 = alpha - beta
        
        return theta1, theta2
    
    def jacobian(self, theta1, theta2):
        """
        Calculate Jacobian matrix for velocity mapping
        
        Args:
            theta1: Angle of joint 1
            theta2: Angle of joint 2
            
        Returns:
            J: 2x2 Jacobian matrix
        """
        c1 = np.cos(theta1)
        s1 = np.sin(theta1)
        c12 = np.cos(theta1 + theta2)
        s12 = np.sin(theta1 + theta2)
        
        J = np.array([
            [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12],
            [self.l1 * c1 + self.l2 * c12, self.l2 * c12]
        ])
        
        return J
    
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
