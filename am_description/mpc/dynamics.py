"""
Aerial Manipulator Dynamics Model

Implements the coupled quadrotor-arm dynamics for MPC prediction.
State: [position(3), velocity(3), quaternion(4), angular_velocity(3), arm_joints(2)]
Total: 15 states
"""

import numpy as np
from .utils import rotate_vector_by_quaternion, skew_symmetric, normalize_quaternion


class AerialManipulatorDynamics:
    """
    Coupled dynamics model for quadrotor with 2-link manipulator arm
    """
    
    def __init__(self):
        # Mass properties (kg)
        self.m_base = 1.5        # Base quadrotor mass
        self.m_arm_base = 0.2    # Arm base link
        self.m_arm1 = 0.1        # First arm link
        self.m_arm2 = 0.1        # Second arm link
        self.m_rotors = 4 * 0.05 # Four rotors
        
        self.total_mass = self.m_base + self.m_arm_base + self.m_arm1 + self.m_arm2 + self.m_rotors
        
        # Base inertia (kg·m²) - approximated as box
        self.I_base = np.diag([0.03, 0.03, 0.05])
        
        # Arm link lengths (m)
        self.L1 = 0.15
        self.L2 = 0.15
        
        # Arm inertias (simplified as point masses for now)
        self.I_arm1 = self.m_arm1 * (self.L1**2) / 12
        self.I_arm2 = self.m_arm2 * (self.L2**2) / 12
        
        # Gravity
        self.g = 9.81
        
        # Aerodynamic drag coefficient
        self.k_drag = 0.1
        
        # Discretization time step
        self.dt = 0.05  # 50ms, matching 20Hz control rate
        
    def continuous_dynamics(self, state, control):
        """
        Compute continuous-time state derivative
        
        Args:
            state: [pos(3), vel(3), quat(4), omega(3), q_arm(2)] - 15 elements
            control: [thrust, torque_x, torque_y, torque_z, q_arm_dot(2)] - 6 elements
        
        Returns:
            state_dot: derivative of state
        """
        # Extract state components
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        q_arm = state[13:15]
        
        # Extract control inputs
        thrust = control[0]
        torques = control[1:4]
        q_arm_dot = control[4:6]
        
        # Normalize quaternion
        quat = normalize_quaternion(quat)
        
        # Position derivative
        pos_dot = vel
        
        # Thrust force in body frame
        thrust_body = np.array([0.0, 0.0, thrust])
        
        # Rotate thrust to world frame
        thrust_world = rotate_vector_by_quaternion(thrust_body, quat)
        
        # Gravity force
        gravity_force = np.array([0.0, 0.0, -self.total_mass * self.g])
        
        # Aerodynamic drag (simplified)
        drag_force = -self.k_drag * vel
        
        # Arm coupling effects (simplified - reaction forces from arm motion)
        # This is a simplified model; full dynamics would include Coriolis terms
        arm_reaction = self._compute_arm_reaction_force(q_arm, q_arm_dot, omega)
        
        # Total force
        total_force = thrust_world + gravity_force + drag_force + arm_reaction
        
        # Velocity derivative (Newton's second law)
        vel_dot = total_force / self.total_mass
        
        # Quaternion derivative (from angular velocity)
        # q_dot = 0.5 * Omega(omega) * q, where Omega is the quaternion matrix
        quat_dot = 0.5 * self._omega_matrix(omega) @ quat
        
        # Compute effective inertia including arm contribution
        I_total = self._compute_total_inertia(q_arm)
        
        # Arm coupling torques
        arm_torque = self._compute_arm_coupling_torque(q_arm, q_arm_dot, omega)
        
        # Angular acceleration (Euler's equation)
        omega_dot = np.linalg.solve(I_total, 
                                     torques + arm_torque - np.cross(omega, I_total @ omega))
        
        # Arm joint acceleration (simplified - assuming direct velocity control)
        q_arm_ddot = np.zeros(2)  # Will be controlled directly via velocity
        
        # Assemble state derivative
        state_dot = np.concatenate([
            pos_dot,      # 3
            vel_dot,      # 3
            quat_dot,     # 4
            omega_dot,    # 3
            q_arm_dot     # 2
        ])
        
        return state_dot
    
    def _omega_matrix(self, omega):
        """
        Create the omega matrix for quaternion derivative
        q_dot = 0.5 * Omega * q
        """
        wx, wy, wz = omega
        return np.array([
            [0,   -wx,  -wy,  -wz],
            [wx,   0,   wz,  -wy],
            [wy,  -wz,   0,   wx],
            [wz,   wy,  -wx,   0]
        ])
    
    def _compute_arm_reaction_force(self, q_arm, q_arm_dot, omega):
        """
        Compute reaction forces on base from arm motion (simplified)
        """
        # Simplified model - centrifugal effects
        q1, q2 = q_arm
        q1_dot, q2_dot = q_arm_dot
        
        # Position of arm COM relative to base (simplified)
        r_arm = np.array([
            self.L1 * np.cos(q1) + 0.5 * self.L2 * np.cos(q1 + q2),
            self.L1 * np.sin(q1) + 0.5 * self.L2 * np.sin(q1 + q2),
            0.0
        ]) * 0.1  # Scale factor for coupling strength
        
        # Centrifugal and Coriolis forces (very simplified)
        force = -(self.m_arm1 + self.m_arm2) * np.cross(omega, np.cross(omega, r_arm))
        
        return force
    
    def _compute_total_inertia(self, q_arm):
        """
        Compute total inertia tensor including arm configuration
        """
        # Base inertia
        I = self.I_base.copy()
        
        # Add arm contribution (simplified - parallel axis theorem)
        q1, q2 = q_arm
        
        # Arm adds to moments of inertia based on configuration
        r1_sq = (self.L1)**2
        r2_sq = (self.L1 + self.L2)**2
        
        # Additional inertia from arm (simplified)
        I[0, 0] += self.m_arm1 * r1_sq * (np.sin(q1)**2) + self.m_arm2 * r2_sq * (np.sin(q1+q2)**2)
        I[1, 1] += self.m_arm1 * r1_sq * (np.cos(q1)**2) + self.m_arm2 * r2_sq * (np.cos(q1+q2)**2)
        I[2, 2] += self.I_arm1 + self.I_arm2
        
        return I
    
    def _compute_arm_coupling_torque(self, q_arm, q_arm_dot, omega):
        """
        Compute coupling torques from arm motion on base
        """
        # Simplified model - gyroscopic effects
        q1, q2 = q_arm
        q1_dot, q2_dot = q_arm_dot
        
        # Gyroscopic torque from arm rotation
        torque = np.zeros(3)
        
        # Simplified coupling - reaction to arm acceleration
        torque[2] = -0.01 * (q1_dot + q2_dot)  # Yaw coupling
        
        return torque
    
    def discretize(self, state, control, method='rk4'):
        """
        Discretize dynamics using RK4 or Euler method
        
        Args:
            state: current state
            control: control input
            method: 'rk4' or 'euler'
        
        Returns:
            next_state: state at next time step
        """
        if method == 'euler':
            state_dot = self.continuous_dynamics(state, control)
            next_state = state + self.dt * state_dot
            
        elif method == 'rk4':
            k1 = self.continuous_dynamics(state, control)
            k2 = self.continuous_dynamics(state + 0.5*self.dt*k1, control)
            k3 = self.continuous_dynamics(state + 0.5*self.dt*k2, control)
            k4 = self.continuous_dynamics(state + self.dt*k3, control)
            
            next_state = state + (self.dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        else:
            raise ValueError(f"Unknown discretization method: {method}")
        
        # Normalize quaternion in next state
        next_state[6:10] = normalize_quaternion(next_state[6:10])
        
        return next_state
    
    def linearize(self, state_eq, control_eq):
        """
        Linearize dynamics around equilibrium point (hover)
        Returns A, B matrices for linear dynamics: x_dot = A*x + B*u
        
        This is a simplified linearization for hover condition.
        For full implementation, use numerical differentiation or symbolic math.
        """
        n_states = 15
        n_controls = 6
        
        # Placeholder - would need proper Jacobian computation
        A = np.zeros((n_states, n_states))
        B = np.zeros((n_states, n_controls))
        
        # Position-velocity coupling
        A[0:3, 3:6] = np.eye(3)
        
        # Simplified linearization around hover
        # Attitude-velocity coupling (small angle approximation)
        A[3:6, 6:9] = np.eye(3) * self.g  # Tilt creates acceleration
        
        # Angular velocity affects quaternion
        A[6:10, 10:13] = np.eye(4, 3) * 0.5
        
        # Control input mapping
        B[3, 0] = 1.0 / self.total_mass  # Thrust affects vertical acceleration
        B[10:13, 1:4] = np.linalg.inv(self.I_base)  # Torques affect angular acceleration
        B[13:15, 4:6] = np.eye(2)  # Direct arm joint velocity control
        
        return A, B
