"""
Acados Model for Aerial Manipulator End-Effector Control

Extended state and control vectors for whole-body MPC control with
end-effector trajectory tracking. This model includes forward kinematics
to compute end-effector position in world frame.

State vector (17 states):
    - position (3): x, y, z (base in world frame)
    - velocity (3): vx, vy, vz
    - quaternion (4): qx, qy, qz, qw
    - angular velocity (3): wx, wy, wz
    - arm joint positions (2): q1, q2
    - arm joint velocities (2): dq1, dq2

Control vector (6 controls):
    - thrust: total thrust force (N)
    - tau_x, tau_y, tau_z: body torques (Nm)
    - ddq1, ddq2: arm joint accelerations (rad/s^2)

End-effector kinematics (from URDF):
    - Arm mounted at (0, 0, -0.05) below base_link
    - Joint 1: revolute about Y-axis
    - Link 1: length L1 = 0.2m
    - Joint 2: revolute about Y-axis (with -π initial offset)
    - Link 2: length L2 = 0.2m
    - End-effector at tip of link 2
    
Note: Arm dynamics are decoupled (simple double integrator) for simplicity.
"""

import numpy as np
from casadi import SX, vertcat, horzcat, sin, cos


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

# Arm link lengths (from URDF)
L1 = 0.2  # First arm link length (m)
L2 = 0.2  # Second arm link length (m)
ARM_MOUNT_Z = -0.05  # Arm mount offset from base_link (m)


def forward_kinematics_body(q1, q2, symbolic=False):
    """
    Compute end-effector position in body frame
    
    From URDF analysis:
    - Joint 1 rotates about Y-axis
    - Joint 2 has -π initial rotation about Y, then rotates about Y
    - Both links point along -Z when joints are at 0
    - Due to -π offset, arm is FOLDED when q1=0, q2=0
    
    Args:
        q1: joint 1 angle (rad)
        q2: joint 2 angle (rad)
        symbolic: if True, use CasADi symbolic types
    
    Returns:
        p_ee: end-effector position in body frame [3]
    """
    if symbolic:
        s1 = sin(q1)
        c1 = cos(q1)
        s12 = sin(q1 + q2)
        c12 = cos(q1 + q2)
    else:
        s1 = np.sin(q1)
        c1 = np.cos(q1)
        s12 = np.sin(q1 + q2)
        c12 = np.cos(q1 + q2)
    
    # End-effector position in body frame
    # Derived from URDF kinematic chain with -π offset at joint 2
    x_ee = L1 * s1 - L2 * s12
    y_ee = 0.0 if not symbolic else SX(0.0)
    z_ee = ARM_MOUNT_Z - L1 * c1 + L2 * c12
    
    if symbolic:
        return vertcat(x_ee, y_ee, z_ee)
    else:
        return np.array([x_ee, y_ee, z_ee])


def forward_kinematics(base_pos, base_quat, q1, q2, symbolic=False):
    """
    Compute end-effector position in world frame
    
    Args:
        base_pos: base position in world frame [3]
        base_quat: base orientation quaternion [qx, qy, qz, qw]
        q1: joint 1 angle (rad)
        q2: joint 2 angle (rad)
        symbolic: if True, use CasADi symbolic types
    
    Returns:
        p_ee_world: end-effector position in world frame [3]
    """
    # End-effector in body frame
    p_ee_body = forward_kinematics_body(q1, q2, symbolic)
    
    # Rotation matrix from body to world
    if symbolic:
        qx, qy, qz, qw = base_quat[0], base_quat[1], base_quat[2], base_quat[3]
        R = vertcat(
            horzcat(1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)),
            horzcat(2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)),
            horzcat(2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2))
        )
        p_ee_world = base_pos + R @ p_ee_body
    else:
        qx, qy, qz, qw = base_quat
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        p_ee_world = base_pos + R @ p_ee_body
    
    return p_ee_world


def jacobian_ee(q1, q2, symbolic=False):
    """
    Compute end-effector Jacobian (partial derivative of EE position w.r.t. joint angles)
    in body frame
    
    Args:
        q1: joint 1 angle (rad)
        q2: joint 2 angle (rad)
        symbolic: if True, use CasADi symbolic types
    
    Returns:
        J: Jacobian matrix [3 x 2]
    """
    if symbolic:
        c1 = cos(q1)
        s1 = sin(q1)
        c12 = cos(q1 + q2)
        s12 = sin(q1 + q2)
    else:
        c1 = np.cos(q1)
        s1 = np.sin(q1)
        c12 = np.cos(q1 + q2)
        s12 = np.sin(q1 + q2)
    
    # Jacobian: dp_ee / d[q1, q2]
    # x_ee = L1*sin(q1) - L2*sin(q1+q2)
    # z_ee = ARM_MOUNT_Z - L1*cos(q1) + L2*cos(q1+q2)
    
    dx_dq1 = L1 * c1 - L2 * c12
    dx_dq2 = -L2 * c12
    
    dy_dq1 = 0.0 if not symbolic else SX(0.0)
    dy_dq2 = 0.0 if not symbolic else SX(0.0)
    
    dz_dq1 = L1 * s1 - L2 * s12
    dz_dq2 = -L2 * s12
    
    if symbolic:
        J = vertcat(
            horzcat(dx_dq1, dx_dq2),
            horzcat(dy_dq1, dy_dq2),
            horzcat(dz_dq1, dz_dq2)
        )
    else:
        J = np.array([
            [dx_dq1, dx_dq2],
            [dy_dq1, dy_dq2],
            [dz_dq1, dz_dq2]
        ])
    
    return J


def export_quadrotor_model():
    """
    Export aerial manipulator model for acados with end-effector output
    
    This model includes:
    - Full 17-state dynamics
    - End-effector position computation for tracking
    - Active arm control (no zeroing)
    
    State vector (17 states):
        - pos [3]: base position in world frame
        - vel [3]: base velocity in world frame
        - quat [4]: base orientation quaternion
        - omega [3]: base angular velocity in body frame
        - q_arm [2]: arm joint positions
        - dq_arm [2]: arm joint velocities
    
    Control vector (6 controls):
        - thrust: total thrust force (N)
        - tau_x, tau_y, tau_z: body torques (Nm)
        - ddq1, ddq2: arm joint accelerations (rad/s^2)
    
    Returns:
        model: acados model structure
    """
    from acados_template import AcadosModel
    
    model = AcadosModel()
    model.name = 'aerial_manipulator_ef'
    
    # Physical parameters (from URDF)
    m = 2.1             # Total mass (kg)
    g = 9.81            # Gravity (m/s^2)
    
    # Inertia (quadrotor body frame, from URDF)
    Ixx = 0.03
    Iyy = 0.03
    Izz = 0.05
    
    # --- Base States ---
    px = SX.sym('px')
    py = SX.sym('py')
    pz = SX.sym('pz')
    pos = vertcat(px, py, pz)
    
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')
    vel = vertcat(vx, vy, vz)
    
    qx = SX.sym('qx')
    qy = SX.sym('qy')
    qz = SX.sym('qz')
    qw = SX.sym('qw')
    quat = vertcat(qx, qy, qz, qw)
    
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    omega = vertcat(wx, wy, wz)
    
    # --- Arm States ---
    q1 = SX.sym('q1')
    q2 = SX.sym('q2')
    q_arm = vertcat(q1, q2)
    
    dq1 = SX.sym('dq1')
    dq2 = SX.sym('dq2')
    dq_arm = vertcat(dq1, dq2)
    
    # Full state vector (17 states)
    x = vertcat(pos, vel, quat, omega, q_arm, dq_arm)
    
    # --- Controls ---
    thrust = SX.sym('thrust')
    tau_x = SX.sym('tau_x')
    tau_y = SX.sym('tau_y')
    tau_z = SX.sym('tau_z')
    ddq1 = SX.sym('ddq1')
    ddq2 = SX.sym('ddq2')
    ddq_arm = vertcat(ddq1, ddq2)
    
    u = vertcat(thrust, tau_x, tau_y, tau_z, ddq1, ddq2)
    
    # --- Rotation matrix from quaternion ---
    R = vertcat(
        horzcat(1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)),
        horzcat(2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)),
        horzcat(2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2))
    )
    
    # --- Base Dynamics ---
    thrust_body = vertcat(0, 0, thrust)
    thrust_world = R @ thrust_body
    f_gravity = vertcat(0, 0, -m * g)
    accel = (thrust_world + f_gravity) / m
    
    # Quaternion derivative
    quat_dot = 0.5 * vertcat(
        qw*wx - qz*wy + qy*wz,
        qz*wx + qw*wy - qx*wz,
        -qy*wx + qx*wy + qw*wz,
        -qx*wx - qy*wy - qz*wz
    )
    
    # Angular velocity derivative (Euler's equation)
    torques = vertcat(tau_x, tau_y, tau_z)
    gyro = vertcat(
        (Iyy - Izz) * wy * wz,
        (Izz - Ixx) * wx * wz,
        (Ixx - Iyy) * wx * wy
    )
    omega_dot = vertcat(
        (torques[0] - gyro[0]) / Ixx,
        (torques[1] - gyro[1]) / Iyy,
        (torques[2] - gyro[2]) / Izz
    )
    
    # --- Arm Dynamics (Simple Double Integrator) ---
    q_arm_dot = dq_arm
    dq_arm_dot = ddq_arm
    
    # --- Full state derivative ---
    x_dot = vertcat(
        vel,
        accel,
        quat_dot,
        omega_dot,
        q_arm_dot,
        dq_arm_dot
    )
    
    # Explicit dynamics
    f_expl = x_dot
    
    # Implicit dynamics
    x_dot_sym = SX.sym('x_dot', x.shape)
    f_impl = x_dot_sym - f_expl
    
    # Assign to model
    model.x = x
    model.xdot = x_dot_sym
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    model.p = []
    
    return model


def get_state_bounds():
    """
    Get reasonable bounds for state variables
    
    Returns:
        x_min, x_max: state bounds [17]
    """
    pos_min = np.array([-10.0, -10.0, 0.0])
    pos_max = np.array([10.0, 10.0, 10.0])
    
    vel_min = np.array([-5.0, -5.0, -5.0])
    vel_max = np.array([5.0, 5.0, 5.0])
    
    quat_min = np.array([-1.0, -1.0, -1.0, -1.0])
    quat_max = np.array([1.0, 1.0, 1.0, 1.0])
    
    omega_min = np.array([-5.0, -5.0, -5.0])
    omega_max = np.array([5.0, 5.0, 5.0])
    
    # Arm joint limits from URDF: ±1.57 rad
    arm_pos_min = np.array([-1.57, -1.57])
    arm_pos_max = np.array([1.57, 1.57])
    
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
    thrust_min = 0.0
    thrust_max = 40.0
    torque_limit = 2.0
    arm_acc_limit = 10.0  # Higher limit for active arm control
    
    u_min = np.array([thrust_min, -torque_limit, -torque_limit, -torque_limit,
                      -arm_acc_limit, -arm_acc_limit])
    u_max = np.array([thrust_max, torque_limit, torque_limit, torque_limit,
                      arm_acc_limit, arm_acc_limit])
    
    return u_min, u_max


# Physical constants
MASS = 2.1
GRAVITY = 9.81
HOVER_THRUST = MASS * GRAVITY


def compute_ee_from_state(state):
    """
    Compute end-effector world position from full state vector
    
    Args:
        state: full state vector [17]
    
    Returns:
        p_ee_world: end-effector position in world frame [3]
    """
    base_pos = state[STATE_POS]
    base_quat = state[STATE_QUAT]
    q1, q2 = state[STATE_ARM_POS]
    
    return forward_kinematics(base_pos, base_quat, q1, q2, symbolic=False)
