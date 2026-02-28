"""
Acados-based MPC Solver for Aerial Manipulator Whole-Body Control

MPC solver for aerial manipulator using acados for real-time optimization.
Includes extended state and control vectors with arm joints.

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

Note: Arm controls are set to zero after solving to keep arm fixed.
"""

import numpy as np
import time
import os
from acados_template import AcadosOcp, AcadosOcpSolver
from .acados_model import (export_quadrotor_model, get_control_bounds, HOVER_THRUST,
                           N_STATES, N_CONTROLS, N_BASE_CONTROLS)


class AcadosMPCSolver:
    """
    MPC solver for aerial manipulator whole-body control using acados
    
    Features:
    - Real-time iteration (RTI) scheme for fast solving
    - Structure-exploiting QP solver (HPIPM)
    - 17 states (base + arm), 6 controls (base + arm)
    - Arm controls set to zero after solving (keeping arm fixed)
    """
    
    def __init__(self, params=None):
        """
        Initialize acados MPC solver
        
        Args:
            params: dictionary of MPC parameters
        """
        # Default parameters
        self.params = {
            'N_horizon': 20,              # Prediction horizon
            'dt': 0.05,                   # Time step (50ms)
            
            # Base state cost weights
            'Q_pos': 10.0,                # Position tracking
            'Q_vel': 1.0,                 # Velocity tracking
            'Q_att': 5.0,                 # Attitude tracking
            'Q_omega': 0.5,               # Angular velocity tracking
            
            # Arm state cost weights (keep small since we want arm fixed)
            'Q_arm_pos': 1.0,             # Arm joint position tracking
            'Q_arm_vel': 0.1,             # Arm joint velocity tracking
            
            # Base control cost weights
            'R_thrust': 0.01,             # Thrust cost
            'R_torque': 0.1,              # Torque cost
            
            # Arm control cost weights
            'R_arm_acc': 0.01,            # Arm acceleration cost
            
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
        
        # Update with provided parameters
        if params is not None:
            self.params.update(params)
        
        # Build OCP
        self.ocp = self._build_ocp()
        
        # Create solver (generates C code on first run)
        # Use unique JSON file name to avoid conflicts with other acados solvers
        self.solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp_am.json')
        
        # Dimensions
        self.n_states = N_STATES       # 17 states
        self.n_controls = N_CONTROLS   # 6 controls
        self.n_base_controls = N_BASE_CONTROLS  # 4 base controls
        self.N = self.params['N_horizon']
        
        # Previous solution for warm starting
        self.x_prev = None
        self.u_prev = None
        
        print(f"Acados MPC solver for aerial manipulator initialized")
        print(f"  Horizon: N={self.N}, dt={self.params['dt']}s")
        print(f"  Solver: {self.params['nlp_solver_type']}")
    
    def _build_ocp(self):
        """
        Build acados Optimal Control Problem
        
        Returns:
            ocp: AcadosOcp object
        """
        ocp = AcadosOcp()
        
        # Get model
        model = export_quadrotor_model()
        ocp.model = model
        
        # Dimensions
        N = self.params['N_horizon']
        nx = N_STATES   # 17 states: pos(3) + vel(3) + quat(4) + omega(3) + arm_pos(2) + arm_vel(2)
        nu = N_CONTROLS # 6 controls: thrust(1) + torques(3) + arm_acc(2)
        
        ocp.dims.N = N
        
        # Time horizon
        ocp.solver_options.tf = N * self.params['dt']
        
        # --- Cost function (Linear Least Squares) ---
        
        # Base state cost weights
        Q_pos = self.params['Q_pos']
        Q_vel = self.params['Q_vel']
        Q_att = self.params['Q_att']
        Q_omega = self.params['Q_omega']
        
        # Arm state cost weights
        Q_arm_pos = self.params['Q_arm_pos']
        Q_arm_vel = self.params['Q_arm_vel']
        
        # State cost matrix Q (17x17)
        Q = np.diag([
            Q_pos, Q_pos, Q_pos,           # Position (x, y, z)
            Q_vel, Q_vel, Q_vel,           # Velocity
            Q_att, Q_att, Q_att, Q_att,    # Quaternion
            Q_omega, Q_omega, Q_omega,     # Angular velocity
            Q_arm_pos, Q_arm_pos,          # Arm joint positions
            Q_arm_vel, Q_arm_vel           # Arm joint velocities
        ])
        
        # Control cost weights
        R_thrust = self.params['R_thrust']
        R_torque = self.params['R_torque']
        R_arm_acc = self.params['R_arm_acc']
        
        # Control cost matrix R (6x6)
        R = np.diag([
            R_thrust,                       # Thrust
            R_torque, R_torque, R_torque,  # Torques
            R_arm_acc, R_arm_acc           # Arm accelerations
        ])
        
        # Cost type
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        
        # Output dimension (state + control)
        ny = nx + nu
        
        # Output matrices
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[nx:, :] = np.eye(nu)
        
        # Combined cost matrix
        W = np.zeros((ny, ny))
        W[:nx, :nx] = Q
        W[nx:, nx:] = R
        ocp.cost.W = W
        
        # Reference (set at runtime)
        ocp.cost.yref = np.zeros((ny,))
        
        # Terminal cost
        Q_terminal = self.params['Q_terminal_factor'] * Q
        ocp.cost.Vx_e = np.eye(nx)
        ocp.cost.W_e = Q_terminal
        ocp.cost.yref_e = np.zeros((nx,))
        
        # --- Constraints ---
        
        # Control bounds
        u_min, u_max = get_control_bounds()
        ocp.constraints.lbu = u_min
        ocp.constraints.ubu = u_max
        ocp.constraints.idxbu = np.arange(nu)
        
        # Initial state constraint (set at runtime)
        ocp.constraints.x0 = np.zeros(nx)
        
        # --- Solver options ---
        
        ocp.solver_options.qp_solver = self.params['qp_solver']
        ocp.solver_options.hessian_approx = self.params['hessian_approx']
        ocp.solver_options.integrator_type = self.params['integrator_type']
        ocp.solver_options.nlp_solver_type = self.params['nlp_solver_type']
        ocp.solver_options.qp_solver_iter_max = self.params['qp_solver_iter_max']
        ocp.solver_options.nlp_solver_tol_stat = self.params['nlp_solver_tol_stat']
        ocp.solver_options.nlp_solver_tol_eq = self.params['nlp_solver_tol_eq']
        ocp.solver_options.nlp_solver_tol_ineq = self.params['nlp_solver_tol_ineq']
        ocp.solver_options.nlp_solver_tol_comp = self.params['nlp_solver_tol_comp']
        
        # RTI options
        if self.params['nlp_solver_type'] == 'SQP_RTI':
            ocp.solver_options.nlp_solver_max_iter = 1
        else:
            ocp.solver_options.nlp_solver_max_iter = 100
        
        return ocp
    
    def solve(self, x0, x_ref_trajectory, u_ref=None, zero_arm_controls=True):
        """
        Solve MPC optimization problem
        
        Args:
            x0: initial state [17] (full state with arm)
            x_ref_trajectory: reference trajectory [N+1 x 17]
            u_ref: reference control (optional) [N x 6]
            zero_arm_controls: if True, set arm accelerations to zero after solving
        
        Returns:
            u_opt: optimal control sequence [N x 6]
            x_pred: predicted state trajectory [N+1 x 17]
            solve_info: dict with solver information
        """
        t_start = time.time()
        
        N = self.params['N_horizon']
        
        # Set initial state constraint
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)
        
        # Default u_ref (hover thrust, zero torques, zero arm accelerations)
        if u_ref is None:
            u_ref_default = np.array([HOVER_THRUST, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            u_ref_default = u_ref[0] if len(u_ref.shape) > 1 else u_ref
        
        # Set reference trajectory for each stage
        for k in range(N):
            # Get reference state at stage k
            x_ref_k = x_ref_trajectory[k] if k < len(x_ref_trajectory) else x_ref_trajectory[-1]
            
            # Get reference control
            if u_ref is not None and k < len(u_ref):
                u_ref_k = u_ref[k]
            else:
                u_ref_k = u_ref_default
            
            # Combined reference [x; u]
            y_ref = np.concatenate([x_ref_k, u_ref_k])
            self.solver.set(k, 'yref', y_ref)
        
        # Terminal reference
        x_ref_e = x_ref_trajectory[-1] if len(x_ref_trajectory) > N else x_ref_trajectory[-1]
        self.solver.set(N, 'yref', x_ref_e)
        
        # Warm start with previous solution
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
        
        # Zero out arm controls if requested (keep arm fixed)
        if zero_arm_controls:
            u_opt[:, 4:6] = 0.0  # Set arm accelerations to zero
        
        # Get cost
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
        """Reset solver state (warm start)"""
        self.x_prev = None
        self.u_prev = None
    
    def get_arm_command(self):
        """
        Get arm command (zero acceleration to keep arm fixed)
        
        Returns:
            arm_acc: arm joint accelerations [2] (zeros)
        """
        return np.array([0.0, 0.0])
    
    def get_base_control(self, u):
        """
        Extract base controls from full control vector
        
        Args:
            u: full control vector [6]
        
        Returns:
            base_control: [thrust, tau_x, tau_y, tau_z]
        """
        return u[:4]
    
    def get_arm_control(self, u):
        """
        Extract arm controls from full control vector
        
        Args:
            u: full control vector [6]
        
        Returns:
            arm_control: [ddq1, ddq2] (joint accelerations)
        """
        return u[4:6]
    
    def __del__(self):
        """Cleanup"""
        pass


def main():
    """
    Test acados MPC solver performance
    """
    print("=" * 60)
    print("Acados MPC Solver for Aerial Manipulator - Performance Test")
    print("=" * 60)
    
    # Create solver
    mpc_params = {
        'N_horizon': 20,
        'dt': 0.05,
        'Q_pos': 10.0,
        'Q_vel': 1.0,
        'Q_att': 5.0,
        'Q_omega': 0.5,
        'R_thrust': 0.01,
        'R_torque': 0.1,
        'nlp_solver_type': 'SQP_RTI',
    }
    
    print("\nInitializing acados solver...")
    solver = AcadosMPCSolver(mpc_params)
    
    print(f"\nMPC Configuration:")
    print(f"  States: {solver.n_states}")
    print(f"  Controls: {solver.n_controls}")
    print(f"  Prediction horizon: {mpc_params['N_horizon']} steps")
    print(f"  Time step: {mpc_params['dt']} s")
    print(f"  Solver type: {mpc_params['nlp_solver_type']}")
    
    # Initial state (hovering at 2m, arm at neutral)
    x0 = np.array([
        0.0, 0.0, 2.0,       # position
        0.0, 0.0, 0.0,       # velocity
        0.0, 0.0, 0.0, 1.0,  # quaternion [x,y,z,w]
        0.0, 0.0, 0.0,       # angular velocity
        0.0, 0.0,            # arm joint positions
        0.0, 0.0             # arm joint velocities
    ])
    
    # Circular reference trajectory
    N = mpc_params['N_horizon']
    x_ref = np.zeros((N + 1, N_STATES))  # 17 states
    radius = 1.5
    speed = 0.3
    
    for k in range(N + 1):
        t = k * mpc_params['dt']
        theta = speed * t
        x_ref[k, 0] = radius * np.cos(theta)      # x
        x_ref[k, 1] = radius * np.sin(theta)      # y
        x_ref[k, 2] = 2.0                          # z
        x_ref[k, 3] = -radius * speed * np.sin(theta)  # vx
        x_ref[k, 4] = radius * speed * np.cos(theta)   # vy
        x_ref[k, 9] = 1.0                          # qw (identity quaternion)
        # Arm states remain at zero (neutral position)
    
    print(f"\nTest Scenario: Circular trajectory (r={radius}m)")
    
    # Warm-up solve
    print(f"\nWarm-up solve...")
    u_opt, x_pred, info = solver.solve(x0, x_ref)
    print(f"  Time: {info['solve_time']*1000:.1f} ms")
    print(f"  Success: {info['success']}")
    print(f"  Cost: {info['cost']:.2f}")
    print(f"  First control: thrust={u_opt[0, 0]:.2f}N, torques={u_opt[0, 1:4]}, arm_acc={u_opt[0, 4:6]}")
    print(f"  Arm command (zero): {solver.get_arm_command()}")
    
    # Benchmark
    n_trials = 50
    print(f"\nRunning {n_trials} benchmark solves...")
    
    solve_times = []
    for i in range(n_trials):
        # Slightly perturb initial state
        x0_perturbed = x0 + 0.01 * np.random.randn(N_STATES)
        x0_perturbed[6:10] /= np.linalg.norm(x0_perturbed[6:10])  # normalize quaternion
        
        _, _, info = solver.solve(x0_perturbed, x_ref)
        solve_times.append(info['solve_time'] * 1000)
    
    solve_times = np.array(solve_times)
    
    print(f"\n{'=' * 60}")
    print("Performance Statistics:")
    print(f"{'=' * 60}")
    print(f"Solve Time (ms):")
    print(f"  Mean:   {np.mean(solve_times):7.2f}")
    print(f"  Median: {np.median(solve_times):7.2f}")
    print(f"  Min:    {np.min(solve_times):7.2f}")
    print(f"  Max:    {np.max(solve_times):7.2f}")
    print(f"  Std:    {np.std(solve_times):7.2f}")
    
    control_period = mpc_params['dt'] * 1000
    print(f"\n{'=' * 60}")
    print("Real-time Feasibility:")
    print(f"{'=' * 60}")
    print(f"Control period: {control_period:.1f} ms")
    print(f"Mean solve time: {np.mean(solve_times):.2f} ms")
    print(f"Max solve time: {np.max(solve_times):.2f} ms")
    
    if np.max(solve_times) < control_period:
        print(f"\n✓ REAL-TIME FEASIBLE")
    else:
        print(f"\n✗ NOT real-time feasible (max solve > control period)")
    
    print(f"\n{'=' * 60}\n")


if __name__ == '__main__':
    main()
