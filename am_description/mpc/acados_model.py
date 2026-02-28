"""
Acados Model for Aerial Manipulator

Defines the symbolic dynamics model using CasADi for acados code generation.
"""

import numpy as np
from casadi import SX, vertcat, sin, cos, norm_2


def export_aerial_manipulator_model():
    """
    Export aerial manipulator model for acados
    
    Returns:
        model: acados model structure
    """
    from acados_template import AcadosModel
    
    model = AcadosModel()
    model.name = 'aerial_manipulator'
    
    # Physical parameters
    m_base = 1.5        # Base quadrotor mass (kg)
    m_arm = 0.3         # Arm mass (kg)
    m_total = m_base + m_arm
    g = 9.81            # Gravity (m/s^2)
    
    # Inertia (quadrotor body frame)
    Ixx = 0.03
    Iyy = 0.03
    Izz = 0.05
    
    # Arm parameters
    l_arm = 0.3         # Arm link length (m)
    m_link = 0.15       # Mass per arm link (kg)
    I_link = 0.01       # Link inertia (kg⋅m^2)
    
    # --- States ---
    # Position (world frame)
    px = SX.sym('px')
    py = SX.sym('py')
    pz = SX.sym('pz')
    pos = vertcat(px, py, pz)
    
    # Velocity (world frame)
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')
    vel = vertcat(vx, vy, vz)
    
    # Quaternion (world to body)
    q0 = SX.sym('q0')  # x
    q1 = SX.sym('q1')  # y
    q2 = SX.sym('q2')  # z
    q3 = SX.sym('q3')  # w
    quat = vertcat(q0, q1, q2, q3)
    
    # Angular velocity (body frame)
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    omega = vertcat(wx, wy, wz)
    
    # Arm joint angles
    q1_arm = SX.sym('q1_arm')
    q2_arm = SX.sym('q2_arm')
    q_arm = vertcat(q1_arm, q2_arm)
    
    # Full state vector (15 states)
    x = vertcat(pos, vel, quat, omega, q_arm)
    
    # --- Controls ---
    thrust = SX.sym('thrust')      # Total thrust (N)
    tau_x = SX.sym('tau_x')        # Roll torque (Nm)
    tau_y = SX.sym('tau_y')        # Pitch torque (Nm)
    tau_z = SX.sym('tau_z')        # Yaw torque (Nm)
    dq1_arm = SX.sym('dq1_arm')    # Arm joint 1 velocity (rad/s)
    dq2_arm = SX.sym('dq2_arm')    # Arm joint 2 velocity (rad/s)
    
    u = vertcat(thrust, tau_x, tau_y, tau_z, dq1_arm, dq2_arm)
    
    # --- Dynamics ---
    
    # Rotation matrix from quaternion (body to world)
    R = vertcat(
        vertcat(
            1 - 2*(q1**2 + q2**2),
            2*(q0*q1 + q2*q3),
            2*(q0*q2 - q1*q3)
        ).T,
        vertcat(
            2*(q0*q1 - q2*q3),
            1 - 2*(q0**2 + q2**2),
            2*(q1*q2 + q0*q3)
        ).T,
        vertcat(
            2*(q0*q2 + q1*q3),
            2*(q1*q2 - q0*q3),
            1 - 2*(q0**2 + q1**2)
        ).T
    )
    
    # Thrust force in world frame
    thrust_world = R @ vertcat(0, 0, thrust)
    
    # Gravitational force
    f_gravity = vertcat(0, 0, -m_total * g)
    
    # Arm coupling effect (simplified)
    # Arm configuration affects inertia and CoM
    arm_effect_pos = vertcat(0, 0, 0)  # Simplified: no direct force coupling
    
    # Linear acceleration
    accel = (thrust_world + f_gravity + arm_effect_pos) / m_total
    
    # Quaternion derivative (from angular velocity)
    # dq/dt = 0.5 * Omega(omega) * q
    omega_quat = vertcat(
        0.5 * (-q0*wx - q1*wy - q2*wz),
        0.5 * ( q3*wx - q2*wy + q1*wz),
        0.5 * ( q2*wx + q3*wy - q0*wz),
        0.5 * (-q1*wx + q0*wy + q3*wz)
    )
    
    # Angular acceleration (simplified, arm effects ignored)
    # Inertia matrix
    J = vertcat(
        vertcat(Ixx, 0, 0).T,
        vertcat(0, Iyy, 0).T,
        vertcat(0, 0, Izz).T
    )
    
    # Torques
    torques = vertcat(tau_x, tau_y, tau_z)
    
    # Gyroscopic effect
    gyro = vertcat(
        (Iyy - Izz) * wy * wz,
        (Izz - Ixx) * wx * wz,
        (Ixx - Iyy) * wx * wy
    )
    
    # Angular acceleration
    omega_dot = vertcat(
        (torques[0] - gyro[0]) / Ixx,
        (torques[1] - gyro[1]) / Iyy,
        (torques[2] - gyro[2]) / Izz
    )
    
    # Arm joint velocities (direct control)
    q_arm_dot = vertcat(dq1_arm, dq2_arm)
    
    # Full state derivative
    x_dot = vertcat(
        vel,           # position derivative
        accel,         # velocity derivative
        omega_quat,    # quaternion derivative
        omega_dot,     # angular velocity derivative
        q_arm_dot      # arm joint derivative
    )
    
    # Explicit dynamics
    f_expl = x_dot
    
    # Implicit dynamics (for implicit integrators): x_dot - f(x,u) = 0
    x_dot_sym = SX.sym('x_dot', x.shape)
    f_impl = x_dot_sym - f_expl
    
    # Assign to model
    model.x = x
    model.xdot = x_dot_sym
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    # Model parameters (can be changed at runtime)
    model.p = []
    
    return model


def get_state_bounds():
    """
    Get reasonable bounds for state variables
    
    Returns:
        x_min, x_max: state bounds
    """
    # Position bounds (large workspace)
    pos_min = np.array([-10.0, -10.0, 0.0])
    pos_max = np.array([10.0, 10.0, 10.0])
    
    # Velocity bounds
    vel_min = np.array([-5.0, -5.0, -5.0])
    vel_max = np.array([5.0, 5.0, 5.0])
    
    # Quaternion bounds (unit sphere)
    quat_min = np.array([-1.0, -1.0, -1.0, -1.0])
    quat_max = np.array([1.0, 1.0, 1.0, 1.0])
    
    # Angular velocity bounds
    omega_min = np.array([-5.0, -5.0, -5.0])
    omega_max = np.array([5.0, 5.0, 5.0])
    
    # Arm joint bounds
    q_arm_min = np.array([-np.pi, -np.pi])
    q_arm_max = np.array([np.pi, np.pi])
    
    x_min = np.concatenate([pos_min, vel_min, quat_min, omega_min, q_arm_min])
    x_max = np.concatenate([pos_max, vel_max, quat_max, omega_max, q_arm_max])
    
    return x_min, x_max


def get_control_bounds():
    """
    Get control input bounds
    
    Returns:
        u_min, u_max: control bounds
    """
    # Thrust bounds
    thrust_min = 0.0
    thrust_max = 35.0
    
    # Torque bounds
    torque_limit = 2.0
    
    # Arm velocity bounds
    arm_vel_limit = 2.0
    
    u_min = np.array([thrust_min, -torque_limit, -torque_limit, -torque_limit,
                      -arm_vel_limit, -arm_vel_limit])
    u_max = np.array([thrust_max, torque_limit, torque_limit, torque_limit,
                      arm_vel_limit, arm_vel_limit])
    
    return u_min, u_max
