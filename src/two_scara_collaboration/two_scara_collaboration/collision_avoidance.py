#!/usr/bin/env python3
"""
Collision Avoidance module for dual SCARA robots
Handles dynamic workspace allocation
"""

import numpy as np
from enum import Enum

class WorkspaceZone(Enum):
    """Workspace allocation zones"""
    LEFT_DOMINANT = 1
    RIGHT_DOMINANT = 2
    SHARED = 3
    NEUTRAL = 4

class CollisionAvoidanceController:
    """
    Manages collision avoidance and workspace allocation for two robots
    """
    
    def __init__(self, left_base=(0.3, 0), right_base=(-0.3, 0),
                 reach_left=1.0, reach_right=1.0):
        """
        Initialize collision avoidance controller
        
        Args:
            left_base: Base position of left robot
            right_base: Base position of right robot
            reach_left: Maximum reach of left robot
            reach_right: Maximum reach of right robot
        """
        self.left_base = np.array(left_base)
        self.right_base = np.array(right_base)
        self.reach_left = reach_left
        self.reach_right = reach_right
        
        # Safety parameters
        self.min_safe_distance = 0.15  # 15 cm minimum distance
        self.conflict_threshold = 0.5  # Distance threshold for conflict
        
        # Workspace allocation parameters
        self.left_allocated_region = None
        self.right_allocated_region = None
    
    def allocate_workspaces(self):
        """
        Dynamically allocate workspaces to each robot
        Based on their positions and task priority
        
        Returns:
            dict: Workspace allocation for each robot
        """
        # Calculate distance between robot bases
        distance = np.linalg.norm(self.right_base - self.left_base)
        
        # Overlapping distance
        overlap = (self.reach_left + self.reach_right) - distance
        
        if overlap <= 0:
            # No overlap - full workspace for each
            return {
                'left': WorkspaceZone.LEFT_DOMINANT,
                'right': WorkspaceZone.RIGHT_DOMINANT,
                'overlap_margin': 0
            }
        else:
            # Overlap exists - allocate shared region
            # Calculate shared region boundaries
            midpoint = (self.left_base + self.right_base) / 2
            
            # Allocate based on task queue (FCFS - First Come First Served)
            return {
                'left': WorkspaceZone.LEFT_DOMINANT,
                'right': WorkspaceZone.RIGHT_DOMINANT,
                'shared_point': midpoint,
                'overlap_distance': overlap
            }
    
    def check_collision_trajectory(self, traj_left, traj_right, 
                                  safety_margin=0.1):
        """
        Check for collision along planned trajectories
        
        Args:
            traj_left: List of joint states for left robot
            traj_right: List of joint states for right robot
            safety_margin: Additional safety margin
            
        Returns:
            bool: True if collision detected
        """
        for i in range(min(len(traj_left), len(traj_right))):
            joints_left = traj_left[i]
            joints_right = traj_right[i]
            
            if self._check_instant_collision(joints_left, joints_right, 
                                            safety_margin):
                return True
        
        return False
    
    def _check_instant_collision(self, joints_left, joints_right, 
                                safety_margin=0.1):
        """
        Check collision at a single instant
        """
        # Calculate end effector positions
        ee_left = self._forward_kinematics(joints_left, self.left_base)
        ee_right = self._forward_kinematics(joints_right, self.right_base)
        
        # Calculate distance
        distance = np.linalg.norm(ee_right - ee_left)
        
        # Collision if too close
        return distance < (self.min_safe_distance + safety_margin)
    
    def _forward_kinematics(self, joints, base):
        """
        Calculate end effector position from joint angles
        Simple SCARA kinematics approximation
        """
        theta1, theta2 = joints[0], joints[1]
        l1, l2 = 0.5, 0.5  # Link lengths
        
        x = base[0] + l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
        y = base[1] + l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
        
        return np.array([x, y])
    
    def get_safety_constraint(self, target_pos_left, target_pos_right):
        """
        Get safety constraints for reaching target positions
        
        Args:
            target_pos_left: Target position for left robot
            target_pos_right: Target position for right robot
            
        Returns:
            dict: Safety constraints and adjustments
        """
        distance = np.linalg.norm(
            np.array(target_pos_right) - np.array(target_pos_left)
        )
        
        constraints = {
            'distance': distance,
            'is_safe': distance >= self.min_safe_distance,
            'margin': distance - self.min_safe_distance
        }
        
        if not constraints['is_safe']:
            # Calculate adjustment needed
            direction = (np.array(target_pos_right) - 
                        np.array(target_pos_left))
            
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
                adjustment = (self.min_safe_distance - distance) / 2
                
                constraints['left_adjustment'] = -direction * adjustment
                constraints['right_adjustment'] = direction * adjustment
        
        return constraints
    
    def compute_avoidance_vector(self, current_pos, obstacles):
        """
        Compute velocity adjustment to avoid obstacles
        
        Args:
            current_pos: Current end effector position
            obstacles: List of obstacle positions
            
        Returns:
            np.array: Avoidance velocity vector
        """
        avoidance_vector = np.zeros(2)
        
        for obstacle in obstacles:
            direction = current_pos - obstacle
            distance = np.linalg.norm(direction)
            
            if distance < self.conflict_threshold and distance > 0:
                # Repulsive force (inverse distance weighting)
                repulsion = direction / (distance ** 2)
                avoidance_vector += repulsion
        
        return avoidance_vector
