"""
MPC Solver for Aerial Manipulator

Formulates and solves the optimization problem for Model Predictive Control.
Uses scipy.optimize for nonlinear optimization (can be upgraded to CasADi for better performance).
"""

import numpy as np
from scipy.optimize import minimize
from .dynamics import AerialManipulatorDynamics
from .utils import quaternion_to_euler


class MPCSolver:
    """
    Model Predictive Controller for aerial manipulator
    
    Solves the optimization problem:
        min sum_{k=0}^{N-1} [ (x_k - x_ref_k)^T Q (x_k - x_ref_k) + u_k^T R u_k + du_k^T Rd du_k ]
        s.t. x_{k+1} = f(x_k, u_k)
             u_min <= u_k <= u_max
             x_min <= x_k <= x_max
    """
    
    def __init__(self, params=None):
        """
        Initialize MPC solver
        
        Args:
            params: dictionary of MPC parameters
        """
        # Default parameters
        self.params = {
            'N_prediction': 20,      # Prediction horizon (1 second at 50Hz)
            'N_control': 20,         # Control horizon
            'dt': 0.05,              # Time step (50ms)
            
            # Cost weights
            'Q_pos': 10.0,           # Position error weight
            'Q_vel': 1.0,            # Velocity error weight
            'Q_att': 5.0,            # Attitude error weight (using Euler angles)
            'Q_omega': 0.5,          # Angular velocity weight
            'Q_arm': 2.0,            # Arm position weight
            'R_thrust': 0.01,        # Thrust effort weight
            'R_torque': 0.1,         # Torque effort weight
            'R_arm': 0.05,           # Arm velocity weight
            'Rd_rate': 0.5,          # Control rate penalty
            
            # Constraints
            'thrust_min': 0.0,       # Minimum thrust (N)
            'thrust_max': 35.0,      # Maximum thrust (N)
            'torque_limit': 2.0,     # Torque limits (Nm)
            'arm_vel_limit': 2.0,    # Arm velocity limits (rad/s)
            'tilt_limit': 0.52,      # Max tilt angle (30 degrees)
            
            # Solver options
            'max_iter': 100,
            'tolerance': 1e-4,
            'solver': 'SLSQP'        # 'SLSQP', 'trust-constr'
        }
        
        # Update with provided parameters
        if params is not None:
            self.params.update(params)
        
        # Initialize dynamics model
        self.dynamics = AerialManipulatorDynamics()
        self.dynamics.dt = self.params['dt']
        
        # State and control dimensions
        self.n_states = 15
        self.n_controls = 6
        
        # Previous solution for warm starting
        self.u_prev = None
        
        # Cost function components storage (for analysis)
        self.last_cost_breakdown = {}
        
    def solve(self, x0, x_ref_trajectory, u_ref=None):
        """
        Solve MPC optimization problem
        
        Args:
            x0: initial state [pos(3), vel(3), quat(4), omega(3), q_arm(2)]
            x_ref_trajectory: reference trajectory [N x n_states]
            u_ref: reference control (optional) [N x n_controls]
        
        Returns:
            u_opt: optimal control sequence [N x n_controls]
            x_pred: predicted state trajectory [N+1 x n_states]
            solve_info: dict with solver information
        """
        N = self.params['N_prediction']
        
        # Initialize control sequence (warm start if available)
        if self.u_prev is not None and len(self.u_prev) == N:
            # Shift previous solution
            u_init = np.vstack([self.u_prev[1:], self.u_prev[-1:]])
        else:
            # Hover control as initial guess
            u_init = np.zeros((N, self.n_controls))
            u_init[:, 0] = self.dynamics.total_mass * self.dynamics.g  # Hover thrust
        
        # Flatten for optimizer
        u_init_flat = u_init.flatten()
        
        # Define bounds for control inputs
        bounds = []
        for k in range(N):
            bounds.append((self.params['thrust_min'], self.params['thrust_max']))  # thrust
            for _ in range(3):  # torques
                bounds.append((-self.params['torque_limit'], self.params['torque_limit']))
            for _ in range(2):  # arm velocities
                bounds.append((-self.params['arm_vel_limit'], self.params['arm_vel_limit']))
        
        # Solve optimization
        result = minimize(
            fun=lambda u: self._cost_function(u, x0, x_ref_trajectory, u_ref),
            x0=u_init_flat,
            method=self.params['solver'],
            bounds=bounds,
            options={'maxiter': self.params['max_iter'], 'ftol': self.params['tolerance']}
        )
        
        # Extract optimal control sequence
        u_opt = result.x.reshape((N, self.n_controls))
        
        # Compute predicted trajectory
        x_pred = self._simulate_trajectory(x0, u_opt)
        
        # Store for warm starting
        self.u_prev = u_opt
        
        # Prepare solver info
        solve_info = {
            'success': result.success,
            'iterations': result.nit,
            'cost': result.fun,
            'message': result.message,
            'computation_time': 0.0,  # Would need timing instrumentation
            'cost_breakdown': self.last_cost_breakdown
        }
        
        return u_opt, x_pred, solve_info
    
    def _cost_function(self, u_flat, x0, x_ref_trajectory, u_ref):
        """
        Compute total cost for optimization
        
        Args:
            u_flat: flattened control sequence
            x0: initial state
            x_ref_trajectory: reference trajectory
            u_ref: reference control
        
        Returns:
            total_cost: scalar cost value
        """
        N = self.params['N_prediction']
        u = u_flat.reshape((N, self.n_controls))
        
        # Simulate trajectory
        x_pred = self._simulate_trajectory(x0, u)
        
        # Initialize cost components
        cost_pos = 0.0
        cost_vel = 0.0
        cost_att = 0.0
        cost_omega = 0.0
        cost_arm = 0.0
        cost_thrust = 0.0
        cost_torque = 0.0
        cost_arm_vel = 0.0
        cost_rate = 0.0
        
        # Stage costs
        for k in range(N):
            x_k = x_pred[k]
            x_ref_k = x_ref_trajectory[min(k, len(x_ref_trajectory)-1)]
            u_k = u[k]
            
            # Position error
            pos_error = x_k[0:3] - x_ref_k[0:3]
            cost_pos += self.params['Q_pos'] * np.dot(pos_error, pos_error)
            
            # Velocity error
            vel_error = x_k[3:6] - x_ref_k[3:6]
            cost_vel += self.params['Q_vel'] * np.dot(vel_error, vel_error)
            
            # Attitude error (using Euler angles for interpretability)
            roll, pitch, yaw = quaternion_to_euler(x_k[6:10])
            roll_ref, pitch_ref, yaw_ref = quaternion_to_euler(x_ref_k[6:10])
            att_error = np.array([roll - roll_ref, pitch - pitch_ref, yaw - yaw_ref])
            cost_att += self.params['Q_att'] * np.dot(att_error, att_error)
            
            # Angular velocity error
            omega_error = x_k[10:13] - x_ref_k[10:13]
            cost_omega += self.params['Q_omega'] * np.dot(omega_error, omega_error)
            
            # Arm position error
            arm_error = x_k[13:15] - x_ref_k[13:15]
            cost_arm += self.params['Q_arm'] * np.dot(arm_error, arm_error)
            
            # Control effort
            cost_thrust += self.params['R_thrust'] * u_k[0]**2
            cost_torque += self.params['R_torque'] * np.dot(u_k[1:4], u_k[1:4])
            cost_arm_vel += self.params['R_arm'] * np.dot(u_k[4:6], u_k[4:6])
            
            # Control rate (smoothness)
            if k > 0:
                du = u[k] - u[k-1]
                cost_rate += self.params['Rd_rate'] * np.dot(du, du)
        
        # Terminal cost (emphasize final state)
        x_N = x_pred[N]
        x_ref_N = x_ref_trajectory[min(N, len(x_ref_trajectory)-1)]
        
        pos_error_N = x_N[0:3] - x_ref_N[0:3]
        cost_pos += 2.0 * self.params['Q_pos'] * np.dot(pos_error_N, pos_error_N)
        
        vel_error_N = x_N[3:6] - x_ref_N[3:6]
        cost_vel += 2.0 * self.params['Q_vel'] * np.dot(vel_error_N, vel_error_N)
        
        # Total cost
        total_cost = (cost_pos + cost_vel + cost_att + cost_omega + cost_arm +
                     cost_thrust + cost_torque + cost_arm_vel + cost_rate)
        
        # Store breakdown for analysis
        self.last_cost_breakdown = {
            'position': cost_pos,
            'velocity': cost_vel,
            'attitude': cost_att,
            'omega': cost_omega,
            'arm': cost_arm,
            'thrust': cost_thrust,
            'torque': cost_torque,
            'arm_vel': cost_arm_vel,
            'rate': cost_rate,
            'total': total_cost
        }
        
        return total_cost
    
    def _simulate_trajectory(self, x0, u_sequence):
        """
        Simulate state trajectory given control sequence
        
        Args:
            x0: initial state
            u_sequence: control sequence [N x n_controls]
        
        Returns:
            x_trajectory: state trajectory [N+1 x n_states]
        """
        N = len(u_sequence)
        x_trajectory = np.zeros((N+1, self.n_states))
        x_trajectory[0] = x0
        
        for k in range(N):
            x_trajectory[k+1] = self.dynamics.discretize(x_trajectory[k], u_sequence[k])
        
        return x_trajectory
    
    def check_constraints(self, x_pred, u_opt):
        """
        Check if solution satisfies constraints
        
        Args:
            x_pred: predicted state trajectory
            u_opt: optimal control sequence
        
        Returns:
            violations: dict of constraint violations
        """
        violations = {
            'thrust': [],
            'torque': [],
            'arm_vel': [],
            'tilt': []
        }
        
        N = len(u_opt)
        
        for k in range(N):
            # Thrust constraints
            if u_opt[k, 0] < self.params['thrust_min'] or u_opt[k, 0] > self.params['thrust_max']:
                violations['thrust'].append((k, u_opt[k, 0]))
            
            # Torque constraints
            for i in range(3):
                if abs(u_opt[k, 1+i]) > self.params['torque_limit']:
                    violations['torque'].append((k, i, u_opt[k, 1+i]))
            
            # Arm velocity constraints
            for i in range(2):
                if abs(u_opt[k, 4+i]) > self.params['arm_vel_limit']:
                    violations['arm_vel'].append((k, i, u_opt[k, 4+i]))
            
            # Tilt constraint
            roll, pitch, yaw = quaternion_to_euler(x_pred[k, 6:10])
            if abs(roll) > self.params['tilt_limit'] or abs(pitch) > self.params['tilt_limit']:
                violations['tilt'].append((k, roll, pitch))
        
        return violations
    
    def update_params(self, new_params):
        """
        Update MPC parameters
        
        Args:
            new_params: dict of parameters to update
        """
        self.params.update(new_params)
        self.dynamics.dt = self.params['dt']
