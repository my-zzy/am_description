"""
MPC End-Effector Control Package for Aerial Manipulator

This package provides MPC-based control for tracking end-effector trajectories.
Unlike mpc_acados which keeps the arm fixed, this package actively controls
the arm joints to make the end-effector follow a desired trajectory.

Components:
    - acados_model: Dynamics model with end-effector kinematics
    - acados_solver: MPC solver for end-effector tracking
    
State vector (17 states):
    - position (3): base x, y, z
    - velocity (3): base vx, vy, vz
    - quaternion (4): base orientation qx, qy, qz, qw
    - angular velocity (3): base wx, wy, wz
    - arm joint positions (2): q1, q2
    - arm joint velocities (2): dq1, dq2

Control vector (6 controls):
    - thrust: total thrust force (N)
    - tau_x, tau_y, tau_z: body torques (Nm)
    - ddq1, ddq2: arm joint accelerations (rad/s^2)
"""

from .acados_model import (
    export_quadrotor_model,
    get_state_bounds,
    get_control_bounds,
    forward_kinematics,
    jacobian_ee,
    MASS,
    GRAVITY,
    HOVER_THRUST,
    N_STATES,
    N_CONTROLS,
    N_BASE_STATES,
    N_BASE_CONTROLS,
    N_ARM_STATES,
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
    L1, L2,  # Arm link lengths
)

from .acados_solver import AcadosMPCSolver

__all__ = [
    # Model
    'export_quadrotor_model',
    'get_state_bounds',
    'get_control_bounds',
    'forward_kinematics',
    'jacobian_ee',
    # Solver
    'AcadosMPCSolver',
    # Constants
    'MASS',
    'GRAVITY',
    'HOVER_THRUST',
    'N_STATES',
    'N_CONTROLS',
    'N_BASE_STATES',
    'N_BASE_CONTROLS',
    'N_ARM_STATES',
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
    'L1', 'L2',
]
