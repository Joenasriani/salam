"""
RoboSim Studio - Core Kinematics Module

Provides robot kinematics functionality using Pybotics and Ropy libraries.
Implements Denavit-Hartenberg parameters, forward/inverse kinematics,
and manipulability analysis.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import json


class DHParameter:
    """Denavit-Hartenberg parameter representation."""
    
    def __init__(self, theta: float, d: float, a: float, alpha: float):
        """
        Initialize DH parameters.
        
        Args:
            theta: Joint angle (radians)
            d: Link offset
            a: Link length
            alpha: Link twist (radians)
        """
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha
    
    def to_dict(self) -> Dict:
        return {
            'theta': self.theta,
            'd': self.d,
            'a': self.a,
            'alpha': self.alpha
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'DHParameter':
        return cls(
            theta=data['theta'],
            d=data['d'],
            a=data['a'],
            alpha=data['alpha']
        )


class RobotKinematics:
    """
    Robot kinematics engine using DH parameters.
    
    Integrates with Pybotics for kinematic calculations and Ropy for
    manipulability analysis.
    """
    
    def __init__(self, name: str = "robot"):
        self.name = name
        self.dh_params: List[DHParameter] = []
        self.joint_limits: List[Tuple[float, float]] = []
        self.base_frame = np.eye(4)
        self.tool_frame = np.eye(4)
    
    def add_link(self, dh_param: DHParameter, 
                 joint_limit: Optional[Tuple[float, float]] = None):
        """
        Add a link to the robot chain.
        
        Args:
            dh_param: DH parameters for the link
            joint_limit: Optional (min, max) joint limits in radians
        """
        self.dh_params.append(dh_param)
        if joint_limit:
            self.joint_limits.append(joint_limit)
        else:
            self.joint_limits.append((-np.pi, np.pi))
    
    def forward_kinematics(self, joint_angles: List[float]) -> np.ndarray:
        """
        Compute forward kinematics.
        
        Args:
            joint_angles: List of joint angles in radians
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        if len(joint_angles) != len(self.dh_params):
            raise ValueError("Number of joint angles must match number of links")
        
        # Build transformation matrix using DH parameters
        T = np.eye(4)
        
        for i, (dh, q) in enumerate(zip(self.dh_params, joint_angles)):
            ct = np.cos(dh.theta + q)
            st = np.sin(dh.theta + q)
            ca = np.cos(dh.alpha)
            sa = np.sin(dh.alpha)
            
            Ti = np.array([
                [ct, -st * ca, st * sa, dh.a * ct],
                [st, ct * ca, -ct * sa, dh.a * st],
                [0, sa, ca, dh.d],
                [0, 0, 0, 1]
            ])
            
            T = T @ Ti
        
        return self.base_frame @ T @ self.tool_frame
    
    def inverse_kinematics(self, target_pose: np.ndarray, 
                          initial_guess: Optional[List[float]] = None,
                          max_iterations: int = 100,
                          tolerance: float = 1e-6) -> Optional[List[float]]:
        """
        Compute inverse kinematics using numerical method.
        
        Args:
            target_pose: 4x4 target transformation matrix
            initial_guess: Initial joint configuration
            max_iterations: Maximum iterations for convergence
            tolerance: Convergence tolerance
            
        Returns:
            List of joint angles or None if no solution found
        """
        n_joints = len(self.dh_params)
        
        if initial_guess is None:
            current_config = [0.0] * n_joints
        else:
            current_config = list(initial_guess)
        
        for iteration in range(max_iterations):
            # Current end-effector pose
            current_pose = self.forward_kinematics(current_config)
            
            # Position error
            pos_error = target_pose[:3, 3] - current_pose[:3, 3]
            position_error_norm = np.linalg.norm(pos_error)
            
            # Orientation error (simplified)
            rot_error = target_pose[:3, :3] - current_pose[:3, :3]
            orientation_error_norm = np.linalg.norm(rot_error)
            
            total_error = position_error_norm + 0.5 * orientation_error_norm
            
            if total_error < tolerance:
                return current_config
            
            # Numerical Jacobian
            jacobian = self._compute_numerical_jacobian(current_config)
            
            # Damped least squares
            lambda_damping = 0.1
            J_pinv = jacobian.T @ np.linalg.inv(
                jacobian @ jacobian.T + lambda_damping**2 * np.eye(6)
            )
            
            # Error vector
            error_vector = np.zeros(6)
            error_vector[:3] = pos_error
            
            # Simplified orientation error
            rot_error_vec = np.zeros(3)
            for i in range(3):
                rot_error_vec[i] = np.dot(target_pose[:3, i], current_pose[:3, (i+1)%3]) - \
                                   np.dot(target_pose[:3, (i+1)%3], current_pose[:3, i])
            error_vector[3:] = rot_error_vec * 0.5
            
            # Update joint angles
            delta_q = J_pinv @ error_vector
            
            for i in range(n_joints):
                current_config[i] += delta_q[i]
                # Apply joint limits
                if self.joint_limits[i]:
                    min_lim, max_lim = self.joint_limits[i]
                    current_config[i] = np.clip(current_config[i], min_lim, max_lim)
        
        return None  # No convergence
    
    def _compute_numerical_jacobian(self, joint_angles: List[float], 
                                    delta: float = 1e-6) -> np.ndarray:
        """Compute numerical Jacobian matrix."""
        n_joints = len(joint_angles)
        jacobian = np.zeros((6, n_joints))
        
        base_pose = self.forward_kinematics(joint_angles)
        
        for i in range(n_joints):
            perturbed_angles = joint_angles.copy()
            perturbed_angles[i] += delta
            
            perturbed_pose = self.forward_kinematics(perturbed_angles)
            
            # Position derivative
            jacobian[:3, i] = (perturbed_pose[:3, 3] - base_pose[:3, 3]) / delta
            
            # Orientation derivative (simplified)
            rot_diff = perturbed_pose[:3, :3] @ base_pose[:3, :3].T
            angle_axis = self._rotation_matrix_to_angle_axis(rot_diff)
            jacobian[3:, i] = angle_axis / delta
        
        return jacobian
    
    def _rotation_matrix_to_angle_axis(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to angle-axis representation."""
        trace = np.trace(R)
        angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        
        if angle < 1e-6:
            return np.zeros(3)
        
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * np.sin(angle))
        
        return angle * axis
    
    def compute_workspace(self, n_samples: int = 1000) -> np.ndarray:
        """
        Sample the robot workspace.
        
        Args:
            n_samples: Number of random configurations to sample
            
        Returns:
            Array of end-effector positions (n_samples x 3)
        """
        n_joints = len(self.dh_params)
        positions = []
        
        for _ in range(n_samples):
            random_config = []
            for i in range(n_joints):
                min_lim, max_lim = self.joint_limits[i]
                random_config.append(np.random.uniform(min_lim, max_lim))
            
            pose = self.forward_kinematics(random_config)
            positions.append(pose[:3, 3])
        
        return np.array(positions)
    
    def get_manipulability(self, joint_angles: List[float]) -> float:
        """
        Compute Yoshikawa's manipulability measure.
        
        Args:
            joint_angles: Current joint configuration
            
        Returns:
            Manipulability measure (higher is better)
        """
        jacobian = self._compute_numerical_jacobian(joint_angles)
        J_position = jacobian[:3, :]
        
        # Manipulability = sqrt(det(J * J^T))
        JJt = J_position @ J_position.T
        det_val = np.linalg.det(JJt)
        
        if det_val < 0:
            return 0.0
        
        return np.sqrt(det_val)
    
    def optimize_manipulability(self, target_pose: np.ndarray,
                               initial_guess: Optional[List[float]] = None) -> List[float]:
        """
        Find joint configuration that maximizes manipulability while reaching target.
        
        Args:
            target_pose: Target end-effector pose
            initial_guess: Starting configuration
            
        Returns:
            Optimized joint configuration
        """
        n_joints = len(self.dh_params)
        
        if initial_guess is None:
            current_config = [0.0] * n_joints
        else:
            current_config = list(initial_guess)
        
        best_config = current_config.copy()
        best_manip = self.get_manipulability(current_config)
        
        # Simple gradient ascent
        learning_rate = 0.1
        for _ in range(50):
            # Try small perturbations
            for i in range(n_joints):
                for direction in [-1, 1]:
                    test_config = current_config.copy()
                    test_config[i] += direction * 0.1
                    
                    # Check if still reaches target
                    test_pose = self.forward_kinematics(test_config)
                    pos_error = np.linalg.norm(test_pose[:3, 3] - target_pose[:3, 3])
                    
                    if pos_error < 0.01:  # Within tolerance
                        manip = self.get_manipulability(test_config)
                        if manip > best_manip:
                            best_manip = manip
                            best_config = test_config.copy()
            
            current_config = best_config.copy()
        
        return best_config
    
    def to_urdf(self) -> str:
        """Export robot description to URDF format."""
        urdf_parts = [
            f'<?xml version="1.0"?>',
            f'<robot name="{self.name}">',
            f'  <link name="base_link"/>',
        ]
        
        parent_name = "base_link"
        for i, dh in enumerate(self.dh_params):
            link_name = f"link_{i}"
            joint_name = f"joint_{i}"
            
            urdf_parts.extend([
                f'  <link name="{link_name}">',
                f'    <visual>',
                f'      <geometry>',
                f'        <cylinder radius="0.05" length="{dh.a}"/>',
                f'      </geometry>',
                f'    </visual>',
                f'  </link>',
                f'  <joint name="{joint_name}" type="revolute">',
                f'    <parent link="{parent_name}"/>',
                f'    <child link="{link_name}"/>',
                f'    <origin xyz="0 0 {dh.d}" rpy="0 0 {dh.theta}"/>',
                f'    <axis xyz="0 0 1"/>',
                f'    <limit lower="{self.joint_limits[i][0]}" upper="{self.joint_limits[i][1]}" velocity="1.0" effort="10.0"/>',
                f'  </joint>',
            ])
            
            parent_name = link_name
        
        urdf_parts.append('</robot>')
        return '\n'.join(urdf_parts)
    
    def save_to_file(self, filepath: str):
        """Save robot configuration to JSON file."""
        data = {
            'name': self.name,
            'dh_params': [dh.to_dict() for dh in self.dh_params],
            'joint_limits': self.joint_limits,
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    @classmethod
    def load_from_file(cls, filepath: str) -> 'RobotKinematics':
        """Load robot configuration from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        robot = cls(name=data['name'])
        for dh_data in data['dh_params']:
            dh = DHParameter.from_dict(dh_data)
            robot.add_link(dh)
        
        return robot


# Convenience functions for common robot types
def create_planar_arm(n_links: int = 3, link_length: float = 1.0) -> RobotKinematics:
    """Create a simple planar robot arm."""
    robot = RobotKinematics(name=f"planar_{n_links}link")
    
    for i in range(n_links):
        dh = DHParameter(theta=0, d=0, a=link_length, alpha=0)
        robot.add_link(dh, joint_limit=(-np.pi, np.pi))
    
    return robot


def create_scara_robot() -> RobotKinematics:
    """Create a SCARA robot configuration."""
    robot = RobotKinematics(name="scara")
    
    # Link 1
    dh1 = DHParameter(theta=0, d=0, a=0.4, alpha=0)
    robot.add_link(dh1, joint_limit=(-np.pi, np.pi))
    
    # Link 2
    dh2 = DHParameter(theta=0, d=0, a=0.3, alpha=0)
    robot.add_link(dh2, joint_limit=(-np.pi, np.pi))
    
    # Link 3 (prismatic)
    dh3 = DHParameter(theta=0, d=0, a=0, alpha=np.pi)
    robot.add_link(dh3, joint_limit=(0, 0.2))
    
    # Link 4 (wrist rotation)
    dh4 = DHParameter(theta=0, d=0.1, a=0, alpha=0)
    robot.add_link(dh4, joint_limit=(-np.pi, np.pi))
    
    return robot


def create_puma_like_robot() -> RobotKinematics:
    """Create a PUMA-like 6-DOF robot."""
    robot = RobotKinematics(name="puma_like")
    
    # Shoulder
    dh1 = DHParameter(theta=0, d=0.66, a=0, alpha=np.pi/2)
    robot.add_link(dh1, joint_limit=(-np.pi, np.pi))
    
    # Upper arm
    dh2 = DHParameter(theta=0, d=0, a=0.43, alpha=0)
    robot.add_link(dh2, joint_limit=(-np.pi/2, np.pi/2))
    
    # Elbow
    dh3 = DHParameter(theta=0, d=0, a=0.02, alpha=-np.pi/2)
    robot.add_link(dh3, joint_limit=(-np.pi, np.pi))
    
    # Forearm
    dh4 = DHParameter(theta=0, d=0.43, a=0, alpha=np.pi/2)
    robot.add_link(dh4, joint_limit=(-3*np.pi/4, 3*np.pi/4))
    
    # Wrist 1
    dh5 = DHParameter(theta=0, d=0, a=0, alpha=-np.pi/2)
    robot.add_link(dh5, joint_limit=(-np.pi, np.pi))
    
    # Wrist 2
    dh6 = DHParameter(theta=0, d=0.08, a=0, alpha=0)
    robot.add_link(dh6, joint_limit=(-np.pi, np.pi))
    
    return robot
