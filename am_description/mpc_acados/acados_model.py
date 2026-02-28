"""
Acados Model for Aerial Manipulator Whole-Body Control

Uses simplified quadrotor dynamics model (ignoring arm dynamics) for
whole-body MPC control. The arm joints are assumed to be fixed.

13 states: position (3), velocity (3), quaternion (4), angular velocity (3)
4 controls: thrust (1), torques (3)

This is essentially the same model as mpc_base but packaged for whole-body
control where arm commands are also published (as zero velocities).
"""

import numpy as np
from casadi import SX, vertcat, horzcat


def export_quadrotor_model():
    """
    Export simplified quadrotor model for acados (ignoring arm dynamics)
    
    This model treats the aerial manipulator as a rigid body with the arm
    fixed. The mass and inertia include the arm contribution.
    
    State vector (13 states):
        - pos [3]: position in world frame (x, y, z)
        - vel [3]: velocity in world frame (vx, vy, vz)
        - quat [4]: quaternion (qx, qy, qz, qw) - body to world
        - omega [3]: angular velocity in body frame (wx, wy, wz)
    
    Control vector (4 controls):
        - thrust: total thrust force (N)
        - tau_x, tau_y, tau_z: body torques (Nm)
    
    Returns:
        model: acados model structure
    """
    from acados_template import AcadosModel
    
    model = AcadosModel()
    model.name = 'aerial_manipulator'
    
    # Physical parameters (from URDF)
    # Total mass: base(1.5) + arm_base(0.2) + arm_links(0.2) + rotors(0.2) = 2.1 kg
    m = 2.1             # Total mass (kg)
    g = 9.81            # Gravity (m/s^2)
    
    # Inertia (quadrotor body frame + arm contribution, from URDF)
    # These values assume arm is at neutral position
    Ixx = 0.03
    Iyy = 0.03
    Izz = 0.05
    
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
    
    # Quaternion (body to world): [qx, qy, qz, qw]
    qx = SX.sym('qx')
    qy = SX.sym('qy')
    qz = SX.sym('qz')
    qw = SX.sym('qw')
    quat = vertcat(qx, qy, qz, qw)
    
    # Angular velocity (body frame)
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    omega = vertcat(wx, wy, wz)
    
    # Full state vector (13 states)
    x = vertcat(pos, vel, quat, omega)
    
    # --- Controls ---
    thrust = SX.sym('thrust')      # Total thrust (N)
    tau_x = SX.sym('tau_x')        # Roll torque (Nm)
    tau_y = SX.sym('tau_y')        # Pitch torque (Nm)
    tau_z = SX.sym('tau_z')        # Yaw torque (Nm)
    
    u = vertcat(thrust, tau_x, tau_y, tau_z)
    
    # --- Dynamics ---
    
    # Rotation matrix from quaternion (body to world)
    # Standard formula for q = [qx, qy, qz, qw]
    R = vertcat(
        horzcat(1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)),
        horzcat(2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)),
        horzcat(2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2))
    )
    
    # Thrust force in world frame (thrust along body z-axis)
    thrust_body = vertcat(0, 0, thrust)
    thrust_world = R @ thrust_body
    
    # Gravitational force
    f_gravity = vertcat(0, 0, -m * g)
    
    # Linear acceleration (F = ma)
    accel = (thrust_world + f_gravity) / m
    
    # Quaternion derivative from angular velocity
    # dq/dt = 0.5 * [0; omega] ⊗ q  (Hamilton convention)
    quat_dot = 0.5 * vertcat(
        qw*wx - qz*wy + qy*wz,
        qz*wx + qw*wy - qx*wz,
        -qy*wx + qx*wy + qw*wz,
        -qx*wx - qy*wy - qz*wz
    )
    
    # Torques
    torques = vertcat(tau_x, tau_y, tau_z)
    
    # Gyroscopic effect: omega × (J * omega)
    gyro = vertcat(
        (Iyy - Izz) * wy * wz,
        (Izz - Ixx) * wx * wz,
        (Ixx - Iyy) * wx * wy
    )
    
    # Angular acceleration: J * omega_dot = tau - omega × (J * omega)
    omega_dot = vertcat(
        (torques[0] - gyro[0]) / Ixx,
        (torques[1] - gyro[1]) / Iyy,
        (torques[2] - gyro[2]) / Izz
    )
    
    # Full state derivative
    x_dot = vertcat(
        vel,           # position derivative = velocity
        accel,         # velocity derivative = acceleration
        quat_dot,      # quaternion derivative
        omega_dot      # angular velocity derivative
    )
    
    # Explicit dynamics: x_dot = f(x, u)
    f_expl = x_dot
    
    # Implicit dynamics: 0 = f_impl(x, x_dot, u)
    x_dot_sym = SX.sym('x_dot', x.shape)
    f_impl = x_dot_sym - f_expl
    
    # Assign to model
    model.x = x
    model.xdot = x_dot_sym
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    # No runtime parameters
    model.p = []
    
    return model


def get_state_bounds():
    """
    Get reasonable bounds for state variables
    
    Returns:
        x_min, x_max: state bounds [13]
    """
    # Position bounds (workspace)
    pos_min = np.array([-10.0, -10.0, 0.0])
    pos_max = np.array([10.0, 10.0, 10.0])
    
    # Velocity bounds
    vel_min = np.array([-5.0, -5.0, -5.0])
    vel_max = np.array([5.0, 5.0, 5.0])
    
    # Quaternion bounds (unit quaternion, but allow some slack)
    quat_min = np.array([-1.0, -1.0, -1.0, -1.0])
    quat_max = np.array([1.0, 1.0, 1.0, 1.0])
    
    # Angular velocity bounds (rad/s)
    omega_min = np.array([-5.0, -5.0, -5.0])
    omega_max = np.array([5.0, 5.0, 5.0])
    
    x_min = np.concatenate([pos_min, vel_min, quat_min, omega_min])
    x_max = np.concatenate([pos_max, vel_max, quat_max, omega_max])
    
    return x_min, x_max


def get_control_bounds():
    """
    Get control input bounds
    
    Returns:
        u_min, u_max: control bounds [4]
    """
    # Thrust bounds (hover thrust ~20.6N for 2.1kg)
    thrust_min = 0.0
    thrust_max = 40.0   # ~2x hover thrust
    
    # Torque bounds (Nm)
    torque_limit = 2.0
    
    u_min = np.array([thrust_min, -torque_limit, -torque_limit, -torque_limit])
    u_max = np.array([thrust_max, torque_limit, torque_limit, torque_limit])
    
    return u_min, u_max


# Physical constants for external use
MASS = 2.1
GRAVITY = 9.81
HOVER_THRUST = MASS * GRAVITY  # ~20.6 N
