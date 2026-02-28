"""
Acados Model for Aerial Manipulator Whole-Body Control

Extended state and control vectors for whole-body MPC control.
Currently uses simplified quadrotor dynamics (ignoring arm coupling effects),
but includes arm states and controls in the vectors for future extension.

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
    - ddq1, ddq2: arm joint accelerations (rad/s^2)

Note: Arm dynamics are currently decoupled (simple double integrator).
The arm controls can be set to zero after MPC solve to keep arm fixed.
"""

import numpy as np
from casadi import SX, vertcat, horzcat


# State indices for easy access
STATE_POS = slice(0, 3)       # Position [0:3]
STATE_VEL = slice(3, 6)       # Velocity [3:6]
STATE_QUAT = slice(6, 10)     # Quaternion [6:10]
STATE_OMEGA = slice(10, 13)   # Angular velocity [10:13]
STATE_ARM_POS = slice(13, 15) # Arm joint positions [13:15]
STATE_ARM_VEL = slice(15, 17) # Arm joint velocities [15:17]

# Control indices
CTRL_THRUST = 0
CTRL_TORQUE = slice(1, 4)     # Torques [1:4]
CTRL_ARM_ACC = slice(4, 6)    # Arm accelerations [4:6]

# Dimensions
N_STATES = 17
N_CONTROLS = 6
N_BASE_STATES = 13
N_ARM_STATES = 4
N_BASE_CONTROLS = 4
N_ARM_CONTROLS = 2


def export_quadrotor_model():
    """
    Export aerial manipulator model for acados with extended state/control vectors
    
    This model includes arm states and controls but uses simplified dynamics
    where the arm is treated as a decoupled double integrator (no coupling
    effects on the base dynamics).
    
    State vector (17 states):
        - pos [3]: position in world frame (x, y, z)
        - vel [3]: velocity in world frame (vx, vy, vz)
        - quat [4]: quaternion (qx, qy, qz, qw) - body to world
        - omega [3]: angular velocity in body frame (wx, wy, wz)
        - q_arm [2]: arm joint positions (q1, q2)
        - dq_arm [2]: arm joint velocities (dq1, dq2)
    
    Control vector (6 controls):
        - thrust: total thrust force (N)
        - tau_x, tau_y, tau_z: body torques (Nm)
        - ddq1, ddq2: arm joint accelerations (rad/s^2)
    
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
    
    # --- Base States ---
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
    
    # --- Arm States ---
    # Arm joint positions (rad)
    q1 = SX.sym('q1')
    q2 = SX.sym('q2')
    q_arm = vertcat(q1, q2)
    
    # Arm joint velocities (rad/s)
    dq1 = SX.sym('dq1')
    dq2 = SX.sym('dq2')
    dq_arm = vertcat(dq1, dq2)
    
    # Full state vector (17 states)
    x = vertcat(pos, vel, quat, omega, q_arm, dq_arm)
    
    # --- Base Controls ---
    thrust = SX.sym('thrust')      # Total thrust (N)
    tau_x = SX.sym('tau_x')        # Roll torque (Nm)
    tau_y = SX.sym('tau_y')        # Pitch torque (Nm)
    tau_z = SX.sym('tau_z')        # Yaw torque (Nm)
    
    # --- Arm Controls ---
    ddq1 = SX.sym('ddq1')          # Joint 1 acceleration (rad/s^2)
    ddq2 = SX.sym('ddq2')          # Joint 2 acceleration (rad/s^2)
    ddq_arm = vertcat(ddq1, ddq2)
    
    # Full control vector (6 controls)
    u = vertcat(thrust, tau_x, tau_y, tau_z, ddq1, ddq2)
    
    # --- Base Dynamics ---
    
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
    # Note: Currently ignores arm dynamics coupling
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
    # Note: Currently ignores arm dynamics coupling (reaction torques)
    omega_dot = vertcat(
        (torques[0] - gyro[0]) / Ixx,
        (torques[1] - gyro[1]) / Iyy,
        (torques[2] - gyro[2]) / Izz
    )
    
    # --- Arm Dynamics (Simple Double Integrator) ---
    # d(q_arm)/dt = dq_arm
    # d(dq_arm)/dt = ddq_arm (control input)
    q_arm_dot = dq_arm
    dq_arm_dot = ddq_arm
    
    # Full state derivative (17 states)
    x_dot = vertcat(
        vel,           # position derivative = velocity
        accel,         # velocity derivative = acceleration
        quat_dot,      # quaternion derivative
        omega_dot,     # angular velocity derivative
        q_arm_dot,     # arm position derivative = arm velocity
        dq_arm_dot     # arm velocity derivative = arm acceleration (control)
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
        x_min, x_max: state bounds [17]
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
    
    # Arm joint position bounds (rad) - from URDF: ±1.57 rad
    arm_pos_min = np.array([-1.57, -1.57])
    arm_pos_max = np.array([1.57, 1.57])
    
    # Arm joint velocity bounds (rad/s)
    arm_vel_min = np.array([-2.0, -2.0])
    arm_vel_max = np.array([2.0, 2.0])
    
    x_min = np.concatenate([pos_min, vel_min, quat_min, omega_min, arm_pos_min, arm_vel_min])
    x_max = np.concatenate([pos_max, vel_max, quat_max, omega_max, arm_pos_max, arm_vel_max])
    
    return x_min, x_max


def get_control_bounds():
    """
    Get control input bounds
    
    Returns:
        u_min, u_max: control bounds [6]
    """
    # Thrust bounds (hover thrust ~20.6N for 2.1kg)
    thrust_min = 0.0
    thrust_max = 40.0   # ~2x hover thrust
    
    # Torque bounds (Nm)
    torque_limit = 2.0
    
    # Arm joint acceleration bounds (rad/s^2)
    arm_acc_limit = 5.0
    
    u_min = np.array([thrust_min, -torque_limit, -torque_limit, -torque_limit,
                      -arm_acc_limit, -arm_acc_limit])
    u_max = np.array([thrust_max, torque_limit, torque_limit, torque_limit,
                      arm_acc_limit, arm_acc_limit])
    
    return u_min, u_max


# Physical constants for external use
MASS = 2.1
GRAVITY = 9.81
HOVER_THRUST = MASS * GRAVITY  # ~20.6 N
