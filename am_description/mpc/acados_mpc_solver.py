"""
Acados-based MPC Solver for Aerial Manipulator

High-performance MPC solver using acados with real-time iteration scheme.
Provides 10-100x speedup compared to scipy-based solver.
"""

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from .acados_model import export_aerial_manipulator_model, get_state_bounds, get_control_bounds
from .utils import quaternion_to_euler


class AcadosMPCSolver:
    """
    High-performance MPC solver using acados
    
    Features:
    - Structure-exploiting QP solver (HPIPM)
    - Real-time iteration (RTI) scheme
    - Automatic C code generation
    - 10-100x faster than scipy
    """
    
    def __init__(self, params=None):
        """
        Initialize acados MPC solver
        
        Args:
            params: dictionary of MPC parameters
        """
        # Default parameters
        self.params = {
            'N_horizon': 10,         # Prediction horizon
            'dt': 0.05,              # Time step (50ms)
            
            # Cost weights
            'Q_pos': 10.0,
            'Q_vel': 1.0,
            'Q_att': 5.0,
            'Q_omega': 0.5,
            'Q_arm': 2.0,
            'R_thrust': 0.01,
            'R_torque': 0.1,
            'R_arm': 0.05,
            'Rd_rate': 0.5,
            
            # Terminal cost multiplier
            'Q_terminal_factor': 2.0,
            
            # Solver options
            'qp_solver': 'PARTIAL_CONDENSING_HPIPM',  # Fast QP solver
            'hessian_approx': 'GAUSS_NEWTON',          # or 'EXACT'
            'integrator_type': 'ERK',                   # Explicit RK4
            'nlp_solver_type': 'SQP_RTI',              # Real-time iteration
            'qp_solver_iter_max': 50,
            'nlp_solver_tol_stat': 1e-3,
            'nlp_solver_tol_eq': 1e-3,
            'nlp_solver_tol_ineq': 1e-3,
            'nlp_solver_tol_comp': 1e-3,
        }
        
        # Update with provided parameters
        if params is not None:
            self.params.update(params)
        
        # Build acados OCP
        self.ocp = self._build_ocp()
        
        # Create solver
        self.solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp.json')
        
        # State dimensions
        self.n_states = 15
        self.n_controls = 6
        self.N = self.params['N_horizon']
        
        # Previous solution for warm starting
        self.x_prev = None
        self.u_prev = None
        
        print(f"Acados MPC solver initialized (N={self.N}, dt={self.params['dt']}s)")
    
    def _build_ocp(self):
        """
        Build acados Optimal Control Problem
        
        Returns:
            ocp: AcadosOcp object
        """
        # Create OCP
        ocp = AcadosOcp()
        
        # Get model
        model = export_aerial_manipulator_model()
        ocp.model = model
        
        # Dimensions
        N = self.params['N_horizon']
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        
        ocp.dims.N = N
        
        # Time horizon
        ocp.solver_options.tf = N * self.params['dt']
        
        # --- Cost function ---
        
        # Stage cost (running cost)
        Q_pos = self.params['Q_pos']
        Q_vel = self.params['Q_vel']
        Q_att = self.params['Q_att']
        Q_omega = self.params['Q_omega']
        Q_arm = self.params['Q_arm']
        R_thrust = self.params['R_thrust']
        R_torque = self.params['R_torque']
        R_arm = self.params['R_arm']
        Rd_rate = self.params['Rd_rate']
        
        # State cost matrix
        Q = np.zeros((nx, nx))
        Q[0:3, 0:3] = Q_pos * np.eye(3)      # Position
        Q[3:6, 3:6] = Q_vel * np.eye(3)      # Velocity
        Q[6:10, 6:10] = Q_att * np.eye(4)    # Quaternion
        Q[10:13, 10:13] = Q_omega * np.eye(3)  # Angular velocity
        Q[13:15, 13:15] = Q_arm * np.eye(2)  # Arm joints
        
        # Control cost matrix
        R = np.zeros((nu, nu))
        R[0, 0] = R_thrust                   # Thrust
        R[1:4, 1:4] = R_torque * np.eye(3)   # Torques
        R[4:6, 4:6] = R_arm * np.eye(2)      # Arm velocities
        
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        
        # Reference tracking: minimize ||Vx*x + Vu*u - y_ref||^2
        ny = nx + nu  # Output dimension (state + control)
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[nx:, :] = np.eye(nu)
        
        # Cost matrix
        W = np.zeros((ny, ny))
        W[:nx, :nx] = Q
        W[nx:, nx:] = R
        ocp.cost.W = W
        
        # Reference (will be set at runtime)
        ocp.cost.yref = np.zeros((ny,))
        
        # Terminal cost
        Q_terminal = self.params['Q_terminal_factor'] * Q
        ocp.cost.Vx_e = np.eye(nx)
        ocp.cost.W_e = Q_terminal
        ocp.cost.yref_e = np.zeros((nx,))
        
        # --- Constraints ---
        
        # Control constraints
        u_min, u_max = get_control_bounds()
        ocp.constraints.lbu = u_min
        ocp.constraints.ubu = u_max
        ocp.constraints.idxbu = np.arange(nu)
        
        # State constraints (optional)
        # x_min, x_max = get_state_bounds()
        # ocp.constraints.lbx = x_min
        # ocp.constraints.ubx = x_max
        # ocp.constraints.idxbx = np.arange(nx)
        
        # Initial state constraint (will be set at runtime)
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
        
        # Real-time iteration options
        if self.params['nlp_solver_type'] == 'SQP_RTI':
            ocp.solver_options.nlp_solver_max_iter = 1  # RTI: exactly 1 iteration
        else:
            ocp.solver_options.nlp_solver_max_iter = 20
        
        return ocp
    
    def solve(self, x0, x_ref_trajectory, u_ref=None):
        """
        Solve MPC optimization problem
        
        Args:
            x0: initial state [15,]
            x_ref_trajectory: reference trajectory [N+1 x 15]
            u_ref: reference control (optional) [N x 6]
        
        Returns:
            u_opt: optimal control sequence [N x 6]
            x_pred: predicted state trajectory [N+1 x 15]
            solve_info: dict with solver information
        """
        import time
        
        N = self.params['N_horizon']
        
        # Set initial state
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)
        
        # Set reference trajectory
        u_ref_default = np.zeros(self.n_controls)
        
        for k in range(N):
            # State reference
            x_ref_k = x_ref_trajectory[min(k, len(x_ref_trajectory)-1)]
            
            # Control reference (zero or provided)
            if u_ref is not None and k < len(u_ref):
                u_ref_k = u_ref[k]
            else:
                u_ref_k = u_ref_default
            
            # Combined reference
            y_ref = np.concatenate([x_ref_k, u_ref_k])
            self.solver.set(k, 'yref', y_ref)
        
        # Terminal reference
        x_ref_N = x_ref_trajectory[min(N, len(x_ref_trajectory)-1)]
        self.solver.set(N, 'yref', x_ref_N)
        
        # Warm start with previous solution
        if self.x_prev is not None and self.u_prev is not None:
            for k in range(N):
                if k < len(self.x_prev) - 1:
                    self.solver.set(k, 'x', self.x_prev[k+1])
                    self.solver.set(k, 'u', self.u_prev[min(k+1, len(self.u_prev)-1)])
        
        # Solve
        t_start = time.time()
        status = self.solver.solve()
        solve_time = time.time() - t_start
        
        # Extract solution
        u_opt = np.zeros((N, self.n_controls))
        x_pred = np.zeros((N+1, self.n_states))
        
        for k in range(N):
            u_opt[k] = self.solver.get(k, 'u')
            x_pred[k] = self.solver.get(k, 'x')
        x_pred[N] = self.solver.get(N, 'x')
        
        # Store for warm starting
        self.x_prev = x_pred
        self.u_prev = u_opt
        
        # Get solver statistics
        stats = self.solver.get_stats('statistics')
        
        # Solve info
        solve_info = {
            'success': status == 0,
            'status': status,
            'iterations': stats[0] if len(stats) > 0 else 0,
            'cost': self.solver.get_cost(),
            'computation_time': solve_time,
            'message': f'Acados status: {status}',
            'cost_breakdown': {}  # Not available in acados
        }
        
        return u_opt, x_pred, solve_info
    
    def reset(self):
        """Reset solver (clear warm start)"""
        self.x_prev = None
        self.u_prev = None
    
    def __del__(self):
        """Cleanup"""
        try:
            if hasattr(self, 'solver'):
                del self.solver
        except:
            pass


def main():
    """
    Test acados MPC solver performance
    """
    import time
    from .utils import euler_to_quaternion
    
    print("=" * 60)
    print("Acados MPC Solver Performance Test")
    print("=" * 60)
    
    # Create solver
    mpc_params = {
        'N_horizon': 10,
        'dt': 0.05,
        'Q_pos': 10.0,
        'Q_vel': 1.0,
        'Q_att': 5.0,
        'Q_omega': 0.5,
        'Q_arm': 2.0,
        'R_thrust': 0.01,
        'R_torque': 0.1,
        'R_arm': 0.05,
        'nlp_solver_type': 'SQP_RTI',
    }
    
    print("\nInitializing acados solver...")
    solver = AcadosMPCSolver(mpc_params)
    
    print(f"\nMPC Configuration:")
    print(f"  Prediction horizon: {mpc_params['N_horizon']} steps")
    print(f"  Time step: {mpc_params['dt']} s")
    print(f"  Solver type: {mpc_params['nlp_solver_type']}")
    
    # Define initial state
    x0 = np.array([0, 0, 2.0,  # pos
                   0, 0, 0,    # vel
                   0, 0, 0, 1, # quat
                   0, 0, 0,    # omega
                   0, 0])      # q_arm
    
    # Define circular reference trajectory
    N = mpc_params['N_horizon']
    x_ref = np.zeros((N + 1, 15))
    radius = 1.5
    speed = 0.3
    
    for k in range(N + 1):
        t = k * mpc_params['dt']
        ang = t * speed
        
        x_ref[k, 0:3] = [radius * np.cos(ang), radius * np.sin(ang), 2.0]
        x_ref[k, 3:6] = [-radius * speed * np.sin(ang), radius * speed * np.cos(ang), 0.0]
        x_ref[k, 6:10] = euler_to_quaternion(0, 0, ang)
    
    print(f"\nTest Scenario:")
    print(f"  Circular trajectory (r={radius}m, v={speed}m/s)")
    
    # Warm-up
    print(f"\nWarm-up solve...")
    t_start = time.time()
    u_opt, x_pred, info = solver.solve(x0, x_ref)
    t_warmup = time.time() - t_start
    print(f"  Time: {t_warmup*1000:.1f} ms")
    print(f"  Success: {info['success']}")
    print(f"  Cost: {info['cost']:.2f}")
    
    # Benchmark
    n_trials = 20
    print(f"\nRunning {n_trials} benchmark solves...")
    
    solve_times = []
    for i in range(n_trials):
        x_test = x0 + np.random.randn(15) * 0.05
        
        t_start = time.time()
        u_opt, x_pred, info = solver.solve(x_test, x_ref)
        t_solve = time.time() - t_start
        
        solve_times.append(t_solve * 1000)
        if i < 5:
            print(f"  Trial {i+1}: {t_solve*1000:.1f} ms")
    
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
        slack = control_period - np.max(solve_times)
        print(f"✓ REAL-TIME FEASIBLE")
        print(f"  Time slack: {slack:.2f} ms ({slack/control_period*100:.1f}%)")
    else:
        print(f"✗ NOT REAL-TIME")
    
    print(f"\n{'=' * 60}\n")


if __name__ == '__main__':
    main()
