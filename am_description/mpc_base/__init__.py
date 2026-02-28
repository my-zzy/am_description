"""
MPC Base Module for Quadrotor Control

Simplified MPC controller for quadrotor base only (no arm).
13 states: position (3), velocity (3), quaternion (4), angular velocity (3)
4 controls: thrust (1), torques (3)
"""

from .acados_base_model import export_quadrotor_model, get_state_bounds, get_control_bounds
from .acados_base_solver import AcadosBaseMPCSolver

__all__ = [
    'export_quadrotor_model',
    'get_state_bounds',
    'get_control_bounds',
    'AcadosBaseMPCSolver',
]
