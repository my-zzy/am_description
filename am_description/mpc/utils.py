"""
Utility functions for quaternion math and coordinate transformations
"""

import numpy as np
import math


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions q1 * q2
    Quaternion format: [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def quaternion_inverse(q):
    """
    Compute inverse (conjugate for unit quaternions) of quaternion
    Quaternion format: [x, y, z, w]
    """
    x, y, z, w = q
    norm_sq = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w]) / norm_sq


def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    Quaternion format: [x, y, z, w]
    Returns: (roll, pitch, yaw) in radians
    """
    x, y, z, w = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    Args: roll, pitch, yaw in radians
    Returns: quaternion [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    return np.array([
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy   # w
    ])


def rotate_vector_by_quaternion(v, q):
    """
    Rotate vector v by quaternion q
    v: 3D vector
    q: quaternion [x, y, z, w]
    Returns: rotated 3D vector
    """
    qx, qy, qz, qw = q
    
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    
    return R @ v


def skew_symmetric(v):
    """
    Create skew-symmetric matrix from 3D vector
    Used for cross product: skew(a) @ b = a x b
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def normalize_quaternion(q):
    """
    Normalize quaternion to unit length
    """
    norm = np.linalg.norm(q)
    if norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / norm
