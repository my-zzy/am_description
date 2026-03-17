"""
Acados-based MPC Solver for Aerial Manipulator End-Effector Control

MPC solver that tracks end-effector trajectories by simultaneously controlling
the base position and arm joint angles. Uses inverse kinematics to convert
desired EE positions to joint angle references.

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
"""

import numpy as np
import time

from casadi import vertcat

from acados_template import AcadosOcp, AcadosOcpSolver
from .acados_model import (
    export_quadrotor_model, get_control_bounds, HOVER_THRUST,
    N_STATES, N_CONTROLS, N_BASE_CONTROLS,
    L1, L2, ARM_MOUNT_Z, forward_kinematics_body, forward_kinematics
)


def inverse_kinematics_body(p_ee_body, elbow_up=True):
    """
    Compute inverse kinematics for arm end-effector position in body frame
    
    The arm is a 2-DOF planar manipulator in the XZ plane.
    Due to -π offset at joint 2, when q1=q2=0, the arm is folded.
    
    Forward kinematics:
        x_ee = L1*sin(q1) - L2*sin(q1+q2)
        z_ee = ARM_MOUNT_Z - L1*cos(q1) + L2*cos(q1+q2)
    
    Args:
        p_ee_body: desired end-effector position in body frame [3]
        elbow_up: if True, prefer elbow-up configuration
    
    Returns:
        q1, q2: joint angles (rad)
        success: True if solution found within joint limits
    """
    x_d = p_ee_body[0]
    z_d = p_ee_body[2] - ARM_MOUNT_Z  # Relative to arm mount
    
    # Distance from arm mount to desired EE
    r = np.sqrt(x_d**2 + z_d**2)
    
    # Check reachability
    if r > L1 + L2:
        # Stretch towards target
        scale = (L1 + L2 - 0.01) / r
        x_d *= scale
        z_d *= scale
        r = L1 + L2 - 0.01
    elif r < abs(L1 - L2) + 0.01:
        # Too close, clamp
        r = abs(L1 - L2) + 0.01
    
    # For a 2-link arm:
    # x_ee = L1*s1 - L2*s12
    # -z_d = L1*c1 - L2*c12  (note: z_d defined relative to mount, negative is down)
    
    # Using law of cosines to find q2
    cos_q2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)
    
    if elbow_up:
        q2 = -np.arccos(cos_q2)  # Negative for elbow up
    else:
        q2 = np.arccos(cos_q2)
    
    # Due to the -π offset in URDF, internal q2 relates to URDF q2 as: q2_urdf = q2 + π
    # But we derived FK with the offset already included, so:
    # x_ee = L1*sin(q1) - L2*sin(q1+q2)
    # z_d = -L1*cos(q1) + L2*cos(q1+q2)
    
    # Solve for q1 using atan2
    # Let's define: A = L1 - L2*cos(q2), B = L2*sin(q2)
    # x_d = A*sin(q1) - B*cos(q1)  ... this is wrong, let me redo
    
    # Actually, let's use geometric approach:
    # phi = atan2(-z_d, x_d)  # angle from mount to target
    # alpha = angle adjustment for elbow
    
    # Using law of cosines for triangle:
    cos_alpha = (L1**2 + r**2 - L2**2) / (2 * L1 * r)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)
    
    # Angle to target (in XZ plane, measured from -Z axis)
    phi = np.arctan2(x_d, -z_d)
    
    if elbow_up:
        q1 = phi - alpha
    else:
        q1 = phi + alpha
    
    # Clamp to joint limits
    q1 = np.clip(q1, -1.57, 1.57)
    q2 = np.clip(q2, -1.57, 1.57)
    
    # Verify solution
    p_check = forward_kinematics_body(q1, q2, symbolic=False)
    error = np.linalg.norm(p_check - p_ee_body)
    success = error < 0.05  # 5cm tolerance
    
    return q1, q2, success


def ee_body_to_joint_ref(ee_body_pos, current_joints=None):
    """
    Convert desired end-effector body position to joint angle reference
    
    Args:
        ee_body_pos: desired EE position in body frame [3]
        current_joints: current joint angles [2] for continuity (optional)
    
    Returns:
        q_ref: reference joint angles [2]
    """
    # Try both elbow configurations and pick the one closer to current
    q1_up, q2_up, success_up = inverse_kinematics_body(ee_body_pos, elbow_up=True)
    q1_down, q2_down, success_down = inverse_kinematics_body(ee_body_pos, elbow_up=False)
    
    if current_joints is not None:
        dist_up = np.linalg.norm(np.array([q1_up, q2_up]) - current_joints)
        dist_down = np.linalg.norm(np.array([q1_down, q2_down]) - current_joints)
        
        if dist_up <= dist_down and success_up:
            return np.array([q1_up, q2_up])
        elif success_down:
            return np.array([q1_down, q2_down])
        else:
            return np.array([q1_up, q2_up])  # Default to elbow up
    else:
        return np.array([q1_up, q2_up]) if success_up else np.array([q1_down, q2_down])


class AcadosMPCSolver:
    """
    MPC solver for aerial manipulator end-effector control using acados
    
    Tracks end-effector trajectories by:
    1. Computing desired arm joint angles from desired EE position (IK)
    2. Tracking base position + joint angles simultaneously
    """
    
    def __init__(self, params=None):
        """
        Initialize acados MPC solver
        
        Args:
            params: dictionary of MPC parameters
        """
        self.params = {
            'N_horizon': 20,
            'dt': 0.05,

            # Cost mode:
            #  - 'ik': existing linear LS cost on (x,u) tracking; EE tracking via IK->joint refs.
            #  - 'ee': nonlinear LS cost that directly penalizes EE world position error (no IK).
            'cost_mode': 'ik',
            
            # Base state cost weights
            'Q_pos': 5.0,                 # Base position (lower than mpc_acados)
            'Q_vel': 1.0,
            'Q_att': 5.0,
            'Q_omega': 0.5,
            
            # Arm state cost weights (higher for active tracking)
            'Q_arm_pos': 20.0,            # Arm joint position (high for EE tracking)
            'Q_arm_vel': 1.0,

            # Direct EE tracking weight (used in cost_mode='ee')
            'Q_ee_pos': 50.0,

            # Leveling weights used in cost_mode='ee' (penalize qx,qy only; yaw remains free)
            'Q_level': 5.0,

            # Optional arm regularization weights used in cost_mode='ee'
            'Q_arm_pos_ee': 0.5,
            'Q_arm_vel_ee': 0.2,
            
            # Control cost weights
            'R_thrust': 0.01,
            'R_torque': 0.1,
            'R_arm_acc': 0.01,            # Low to allow fast arm motion
            
            # Terminal cost factor
            'Q_terminal_factor': 2.0,
            
            # Solver options
            'qp_solver': 'PARTIAL_CONDENSING_HPIPM',
            'hessian_approx': 'GAUSS_NEWTON',
            'integrator_type': 'ERK',
            'nlp_solver_type': 'SQP_RTI',
            'qp_solver_iter_max': 50,
            'nlp_solver_tol_stat': 1e-3,
            'nlp_solver_tol_eq': 1e-3,
            'nlp_solver_tol_ineq': 1e-3,
            'nlp_solver_tol_comp': 1e-3,
        }
        
        if params is not None:
            self.params.update(params)
        
        self.ocp = self._build_ocp()

        json_file = 'acados_ocp_ef_ik.json' if self.params['cost_mode'] == 'ik' else 'acados_ocp_ef_ee.json'
        self.solver = AcadosOcpSolver(self.ocp, json_file=json_file)
        
        self.n_states = N_STATES
        self.n_controls = N_CONTROLS
        self.N = self.params['N_horizon']
        
        self.x_prev = None
        self.u_prev = None
        
        print(f"Acados MPC solver for end-effector control initialized")
        print(f"  Horizon: N={self.N}, dt={self.params['dt']}s")
        print(f"  Solver: {self.params['nlp_solver_type']}")
    
    def _build_ocp(self):
        """Build acados Optimal Control Problem"""
        ocp = AcadosOcp()
        
        model = export_quadrotor_model()
        ocp.model = model
        
        N = self.params['N_horizon']
        nx = N_STATES
        nu = N_CONTROLS
        
        ocp.dims.N = N
        ocp.solver_options.tf = N * self.params['dt']
        
        cost_mode = self.params.get('cost_mode', 'ik')
        if cost_mode not in ('ik', 'ee'):
            raise ValueError(f"Unknown cost_mode '{cost_mode}'. Expected 'ik' or 'ee'.")

        if cost_mode == 'ik':
            # --- Existing IK-based tracking: LINEAR_LS on (x,u) ---
            Q_pos = self.params['Q_pos']
            Q_vel = self.params['Q_vel']
            Q_att = self.params['Q_att']
            Q_omega = self.params['Q_omega']
            Q_arm_pos = self.params['Q_arm_pos']
            Q_arm_vel = self.params['Q_arm_vel']

            Q = np.diag([
                Q_pos, Q_pos, Q_pos,
                Q_vel, Q_vel, Q_vel,
                Q_att, Q_att, Q_att, Q_att,
                Q_omega, Q_omega, Q_omega,
                Q_arm_pos, Q_arm_pos,
                Q_arm_vel, Q_arm_vel
            ])

            R_thrust = self.params['R_thrust']
            R_torque = self.params['R_torque']
            R_arm_acc = self.params['R_arm_acc']

            R = np.diag([
                R_thrust,
                R_torque, R_torque, R_torque,
                R_arm_acc, R_arm_acc
            ])

            ocp.cost.cost_type = 'LINEAR_LS'
            ocp.cost.cost_type_e = 'LINEAR_LS'

            ny = nx + nu

            ocp.cost.Vx = np.zeros((ny, nx))
            ocp.cost.Vx[:nx, :nx] = np.eye(nx)

            ocp.cost.Vu = np.zeros((ny, nu))
            ocp.cost.Vu[nx:, :] = np.eye(nu)

            W = np.zeros((ny, ny))
            W[:nx, :nx] = Q
            W[nx:, nx:] = R
            ocp.cost.W = W

            ocp.cost.yref = np.zeros((ny,))

            Q_terminal = self.params['Q_terminal_factor'] * Q
            ocp.cost.Vx_e = np.eye(nx)
            ocp.cost.W_e = Q_terminal
            ocp.cost.yref_e = np.zeros((nx,))

        else:
            # --- Direct EE world-position tracking: NONLINEAR_LS ---
            # y = [pos(3), vel(3), qx,qy, omega(3), q_arm(2), dq_arm(2), p_ee_world(3), u(6)]
            # Penalizing qx,qy keeps the vehicle level while leaving yaw free.
            x_sym = ocp.model.x
            u_sym = ocp.model.u

            pos = x_sym[0:3]
            vel = x_sym[3:6]
            quat = x_sym[6:10]
            omega = x_sym[10:13]
            q_arm = x_sym[13:15]
            dq_arm = x_sym[15:17]

            p_ee_world = forward_kinematics(pos, quat, q_arm[0], q_arm[1], symbolic=True)

            # Stage output
            y_expr = vertcat(
                pos,
                vel,
                quat[0], quat[1],
                omega,
                q_arm,
                dq_arm,
                p_ee_world,
                u_sym,
            )

            # Terminal output (no control regularization)
            y_expr_e = vertcat(
                pos,
                vel,
                quat[0], quat[1],
                omega,
                q_arm,
                dq_arm,
                p_ee_world,
            )

            ocp.model.cost_y_expr = y_expr
            ocp.model.cost_y_expr_e = y_expr_e

            ocp.cost.cost_type = 'NONLINEAR_LS'
            ocp.cost.cost_type_e = 'NONLINEAR_LS'

            ny = int(y_expr.shape[0])
            ny_e = int(y_expr_e.shape[0])
            ocp.dims.ny = ny
            ocp.dims.ny_e = ny_e

            Q_pos = self.params['Q_pos']
            Q_vel = self.params['Q_vel']
            Q_level = self.params.get('Q_level', self.params.get('Q_att', 5.0))
            Q_omega = self.params['Q_omega']
            Q_arm_pos = self.params.get('Q_arm_pos_ee', 0.5)
            Q_arm_vel = self.params.get('Q_arm_vel_ee', 0.2)
            Q_ee_pos = self.params.get('Q_ee_pos', 50.0)

            R_thrust = self.params['R_thrust']
            R_torque = self.params['R_torque']
            R_arm_acc = self.params['R_arm_acc']

            W_diag = np.array(
                [
                    # pos
                    Q_pos, Q_pos, Q_pos,
                    # vel
                    Q_vel, Q_vel, Q_vel,
                    # qx, qy (level)
                    Q_level, Q_level,
                    # omega
                    Q_omega, Q_omega, Q_omega,
                    # q_arm
                    Q_arm_pos, Q_arm_pos,
                    # dq_arm
                    Q_arm_vel, Q_arm_vel,
                    # p_ee_world
                    Q_ee_pos, Q_ee_pos, Q_ee_pos,
                    # u
                    R_thrust,
                    R_torque, R_torque, R_torque,
                    R_arm_acc, R_arm_acc,
                ],
                dtype=float,
            )
            if W_diag.shape[0] != ny:
                raise RuntimeError(f"Internal error: W_diag has {W_diag.shape[0]} entries, expected ny={ny}.")
            ocp.cost.W = np.diag(W_diag)
            ocp.cost.yref = np.zeros((ny,))

            W_diag_e = W_diag[:ny_e]
            ocp.cost.W_e = self.params['Q_terminal_factor'] * np.diag(W_diag_e)
            ocp.cost.yref_e = np.zeros((ny_e,))
        
        # Control bounds
        u_min, u_max = get_control_bounds()
        ocp.constraints.lbu = u_min
        ocp.constraints.ubu = u_max
        ocp.constraints.idxbu = np.arange(nu)
        
        # Initial state
        ocp.constraints.x0 = np.zeros(nx)
        
        # Solver options
        ocp.solver_options.qp_solver = self.params['qp_solver']
        ocp.solver_options.hessian_approx = self.params['hessian_approx']
        ocp.solver_options.integrator_type = self.params['integrator_type']
        ocp.solver_options.nlp_solver_type = self.params['nlp_solver_type']
        ocp.solver_options.qp_solver_iter_max = self.params['qp_solver_iter_max']
        ocp.solver_options.nlp_solver_tol_stat = self.params['nlp_solver_tol_stat']
        ocp.solver_options.nlp_solver_tol_eq = self.params['nlp_solver_tol_eq']
        ocp.solver_options.nlp_solver_tol_ineq = self.params['nlp_solver_tol_ineq']
        ocp.solver_options.nlp_solver_tol_comp = self.params['nlp_solver_tol_comp']
        
        if self.params['nlp_solver_type'] == 'SQP_RTI':
            ocp.solver_options.nlp_solver_max_iter = 1
        else:
            ocp.solver_options.nlp_solver_max_iter = 100
        
        return ocp
    
    def solve(self, x0, x_ref_trajectory, u_ref=None, ee_ref_trajectory=None):
        """
        Solve MPC optimization problem
        
        Args:
            x0: initial state [17]
            x_ref_trajectory: reference trajectory [N+1 x 17]
                              Should include arm joint references from IK
            u_ref: reference control (optional) [N x 6]
        
        Returns:
            u_opt: optimal control sequence [N x 6]
            x_pred: predicted state trajectory [N+1 x 17]
            solve_info: dict with solver information
        """
        t_start = time.time()
        
        N = self.params['N_horizon']
        
        # Set initial state
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)
        
        # Default u_ref
        if u_ref is None:
            u_ref_default = np.array([HOVER_THRUST, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            u_ref_default = u_ref[0] if len(u_ref.shape) > 1 else u_ref
        
        cost_mode = self.params.get('cost_mode', 'ik')
        if cost_mode == 'ik':
            # Set reference trajectory (yref = [x_ref, u_ref])
            for k in range(N):
                x_ref_k = x_ref_trajectory[k] if k < len(x_ref_trajectory) else x_ref_trajectory[-1]

                if u_ref is not None and k < len(u_ref):
                    u_ref_k = u_ref[k]
                else:
                    u_ref_k = u_ref_default

                y_ref = np.concatenate([x_ref_k, u_ref_k])
                self.solver.set(k, 'yref', y_ref)

            # Terminal reference (yref_e = x_ref)
            x_ref_e = x_ref_trajectory[-1] if len(x_ref_trajectory) > N else x_ref_trajectory[-1]
            self.solver.set(N, 'yref', x_ref_e)
        else:
            if ee_ref_trajectory is None:
                raise ValueError("ee_ref_trajectory is required when cost_mode='ee'.")

            for k in range(N):
                x_ref_k = x_ref_trajectory[k] if k < len(x_ref_trajectory) else x_ref_trajectory[-1]
                ee_ref_k = ee_ref_trajectory[k] if k < len(ee_ref_trajectory) else ee_ref_trajectory[-1]

                if u_ref is not None and k < len(u_ref):
                    u_ref_k = u_ref[k]
                else:
                    u_ref_k = u_ref_default

                # Must match y_expr in _build_ocp():
                # [pos(3), vel(3), qx,qy, omega(3), q_arm(2), dq_arm(2), p_ee_world(3), u(6)]
                y_ref = np.concatenate(
                    [
                        x_ref_k[0:3],
                        x_ref_k[3:6],
                        x_ref_k[6:8],
                        x_ref_k[10:13],
                        x_ref_k[13:15],
                        x_ref_k[15:17],
                        ee_ref_k,
                        u_ref_k,
                    ]
                )
                self.solver.set(k, 'yref', y_ref)

            x_ref_e = x_ref_trajectory[-1] if len(x_ref_trajectory) > N else x_ref_trajectory[-1]
            ee_ref_e = ee_ref_trajectory[-1] if len(ee_ref_trajectory) > N else ee_ref_trajectory[-1]
            y_ref_e = np.concatenate(
                [
                    x_ref_e[0:3],
                    x_ref_e[3:6],
                    x_ref_e[6:8],
                    x_ref_e[10:13],
                    x_ref_e[13:15],
                    x_ref_e[15:17],
                    ee_ref_e,
                ]
            )
            self.solver.set(N, 'yref', y_ref_e)
        
        # Warm start
        if self.x_prev is not None and self.u_prev is not None:
            for k in range(N):
                self.solver.set(k, 'x', self.x_prev[min(k+1, N)])
                self.solver.set(k, 'u', self.u_prev[min(k, N-1)])
            self.solver.set(N, 'x', self.x_prev[N])
        
        # Solve
        status = self.solver.solve()
        
        # Get solution
        u_opt = np.zeros((N, self.n_controls))
        x_pred = np.zeros((N + 1, self.n_states))
        
        for k in range(N):
            x_pred[k] = self.solver.get(k, 'x')
            u_opt[k] = self.solver.get(k, 'u')
        x_pred[N] = self.solver.get(N, 'x')
        
        # Store for warm starting
        self.x_prev = x_pred.copy()
        self.u_prev = u_opt.copy()
        
        cost = self.solver.get_cost()
        t_solve = time.time() - t_start
        
        solve_info = {
            'success': status == 0,
            'status': status,
            'cost': cost,
            'solve_time': t_solve,
        }
        
        return u_opt, x_pred, solve_info
    
    def reset(self):
        """Reset solver state"""
        self.x_prev = None
        self.u_prev = None
    
    def compute_ee_reference(self, base_pos_ref, ee_world_ref, base_quat_ref=None, current_joints=None):
        """
        Compute full state reference from base position and EE world position
        
        Args:
            base_pos_ref: desired base position in world frame [3]
            ee_world_ref: desired end-effector position in world frame [3]
            base_quat_ref: desired base orientation [4] (default: identity)
            current_joints: current joint angles for IK continuity [2]
        
        Returns:
            x_ref: full state reference [17]
        """
        if base_quat_ref is None:
            base_quat_ref = np.array([0, 0, 0, 1])  # Identity
        
        # Compute EE position in body frame
        qx, qy, qz, qw = base_quat_ref
        R_T = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy + qz*qw), 2*(qx*qz - qy*qw)],
            [2*(qx*qy - qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz + qx*qw)],
            [2*(qx*qz + qy*qw), 2*(qy*qz - qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])  # R^T
        
        ee_body_ref = R_T @ (ee_world_ref - base_pos_ref)
        
        # Compute joint angles via IK
        q_arm_ref = ee_body_to_joint_ref(ee_body_ref, current_joints)
        
        # Build full state reference
        x_ref = np.zeros(N_STATES)
        x_ref[0:3] = base_pos_ref
        x_ref[3:6] = 0.0  # Zero velocity
        x_ref[6:10] = base_quat_ref
        x_ref[10:13] = 0.0  # Zero angular velocity
        x_ref[13:15] = q_arm_ref
        x_ref[15:17] = 0.0  # Zero arm velocity
        
        return x_ref
    
    def __del__(self):
        pass


def main():
    """Test end-effector tracking solver"""
    print("=" * 60)
    print("Acados MPC Solver for End-Effector Control - Test")
    print("=" * 60)
    
    from .acados_model import forward_kinematics, compute_ee_from_state
    
    mpc_params = {
        'N_horizon': 20,
        'dt': 0.05,
        'Q_pos': 5.0,
        'Q_arm_pos': 20.0,
        'nlp_solver_type': 'SQP_RTI',
    }
    
    print("\nInitializing solver...")
    solver = AcadosMPCSolver(mpc_params)
    
    # Initial state: hovering at 2m, arm extended down
    x0 = np.array([
        0.0, 0.0, 2.0,       # position
        0.0, 0.0, 0.0,       # velocity
        0.0, 0.0, 0.0, 1.0,  # quaternion
        0.0, 0.0, 0.0,       # angular velocity
        0.0, np.pi,          # arm joints (extended down)
        0.0, 0.0             # arm velocities
    ])
    
    # Get initial EE position
    ee_init = compute_ee_from_state(x0)
    print(f"\nInitial EE position: {ee_init}")
    
    # Target: move EE 0.3m forward
    ee_target = ee_init + np.array([0.3, 0.0, 0.0])
    print(f"Target EE position: {ee_target}")
    
    # Generate reference trajectory
    N = mpc_params['N_horizon']
    x_ref = np.zeros((N + 1, N_STATES))
    
    for k in range(N + 1):
        # Interpolate EE position
        t = k / N
        ee_ref = ee_init + t * (ee_target - ee_init)
        base_ref = np.array([0, 0, 2.0])  # Keep base stationary
        
        x_ref[k] = solver.compute_ee_reference(base_ref, ee_ref, current_joints=x0[13:15])
    
    print(f"\nReference arm joints at t=0: {x_ref[0, 13:15]}")
    print(f"Reference arm joints at t=N: {x_ref[N, 13:15]}")
    
    # Solve
    print("\nSolving MPC...")
    u_opt, x_pred, info = solver.solve(x0, x_ref)
    
    print(f"  Solve time: {info['solve_time']*1000:.1f} ms")
    print(f"  Success: {info['success']}")
    print(f"  Cost: {info['cost']:.2f}")
    print(f"  First control: thrust={u_opt[0, 0]:.2f}N, arm_acc={u_opt[0, 4:6]}")
    
    # Check final EE position
    ee_final = compute_ee_from_state(x_pred[-1])
    print(f"\nPredicted final EE position: {ee_final}")
    print(f"EE tracking error: {np.linalg.norm(ee_final - ee_target):.4f} m")
    
    # Benchmark
    n_trials = 50
    print(f"\nRunning {n_trials} benchmark solves...")
    
    solve_times = []
    for _ in range(n_trials):
        x0_perturbed = x0 + 0.01 * np.random.randn(N_STATES)
        x0_perturbed[6:10] /= np.linalg.norm(x0_perturbed[6:10])
        _, _, info = solver.solve(x0_perturbed, x_ref)
        solve_times.append(info['solve_time'] * 1000)
    
    solve_times = np.array(solve_times)
    print(f"\nSolve time statistics:")
    print(f"  Mean: {np.mean(solve_times):.2f} ms")
    print(f"  Max:  {np.max(solve_times):.2f} ms")
    print("=" * 60)


if __name__ == '__main__':
    main()
