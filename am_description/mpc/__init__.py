"""
MPC Module for Aerial Manipulator Control

This module provides Model Predictive Control for the aerial manipulator,
handling coupled quadrotor-arm dynamics with constraint satisfaction.
"""

from .dynamics import AerialManipulatorDynamics
from .mpc_solver import MPCSolver
from .state_estimator import StateEstimator
from .utils import quaternion_multiply, quaternion_inverse, quaternion_to_euler, euler_to_quaternion

__all__ = [
    'AerialManipulatorDynamics',
    'MPCSolver',
    'StateEstimator',
    'quaternion_multiply',
    'quaternion_inverse',
    'quaternion_to_euler',
    'euler_to_quaternion'
]
