"""
MPC Acados Module for Aerial Manipulator Whole-Body Control

Uses acados for real-time optimal control of the aerial manipulator.
The arm joints are kept fixed (zero velocity commands) while the quadrotor
base is controlled using MPC.

This module uses the same simplified dynamics model as mpc_base which
ignores arm dynamics, but provides whole-body control interface including
arm joint command publishing.

State vector (13 states for base):
    - position (3): x, y, z
    - velocity (3): vx, vy, vz
    - quaternion (4): qx, qy, qz, qw
    - angular velocity (3): wx, wy, wz

Control vector (4 controls for base):
    - thrust: total thrust force (N)
    - tau_x, tau_y, tau_z: body torques (Nm)

Arm joints are commanded separately with zero velocity to keep them fixed.
"""

from .acados_model import (
    export_quadrotor_model,
    get_state_bounds,
    get_control_bounds,
    MASS,
    GRAVITY,
    HOVER_THRUST
)
from .acados_solver import AcadosMPCSolver

__all__ = [
    'export_quadrotor_model',
    'get_state_bounds',
    'get_control_bounds',
    'AcadosMPCSolver',
    'MASS',
    'GRAVITY',
    'HOVER_THRUST',
]
