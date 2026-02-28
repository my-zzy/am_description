"""
MPC Acados Module for Aerial Manipulator Whole-Body Control

Uses acados for real-time optimal control of the aerial manipulator.
Extended state and control vectors include arm joints, but arm controls
are set to zero after solving to keep the arm fixed.

State vector (17 states):
    - position (3): x, y, z
    - velocity (3): vx, vy, vz
    - quaternion (4): qx, qy, qz, qw
    - angular velocity (3): wx, wy, wz
    - arm joint positions (2): q1, q2
    - arm joint velocities (2): dq1, dq2

Control vector (6 controls):
    - thrust: total thrust force (N)
    - tau_x, tau_y, tau_z: body torques (Nm)
    - ddq1, ddq2: arm joint accelerations (rad/s^2) - set to zero

Arm joint accelerations are set to zero after MPC solve to keep arm fixed.
"""

from .acados_model import (
    export_quadrotor_model,
    get_state_bounds,
    get_control_bounds,
    MASS,
    GRAVITY,
    HOVER_THRUST,
    N_STATES,
    N_CONTROLS,
    N_BASE_STATES,
    N_ARM_STATES,
    N_BASE_CONTROLS,
    N_ARM_CONTROLS,
    STATE_POS,
    STATE_VEL,
    STATE_QUAT,
    STATE_OMEGA,
    STATE_ARM_POS,
    STATE_ARM_VEL,
    CTRL_THRUST,
    CTRL_TORQUE,
    CTRL_ARM_ACC,
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
    'N_STATES',
    'N_CONTROLS',
    'N_BASE_STATES',
    'N_ARM_STATES',
    'N_BASE_CONTROLS',
    'N_ARM_CONTROLS',
    'STATE_POS',
    'STATE_VEL',
    'STATE_QUAT',
    'STATE_OMEGA',
    'STATE_ARM_POS',
    'STATE_ARM_VEL',
    'CTRL_THRUST',
    'CTRL_TORQUE',
    'CTRL_ARM_ACC',
]
