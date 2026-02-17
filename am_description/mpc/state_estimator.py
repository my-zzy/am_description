"""
State Estimator for Aerial Manipulator

Fuses IMU data (acceleration, angular velocity) with joint encoder data
to estimate the full 15-state vector for MPC.
"""

import numpy as np
from .utils import rotate_vector_by_quaternion, normalize_quaternion


class StateEstimator:
    """
    Extended Kalman Filter-based state estimator
    Fuses IMU and joint state measurements
    """
    
    def __init__(self):
        # State vector: [pos(3), vel(3), quat(4), omega(3), q_arm(2)]
        self.state = np.zeros(15)
        self.state[6:10] = np.array([0, 0, 0, 1])  # Initialize quaternion to identity
        
        # Covariance matrix
        self.P = np.eye(15) * 0.1
        
        # Process noise covariance
        self.Q = np.eye(15) * 0.01
        self.Q[0:3, 0:3] *= 0.001  # Position (low noise)
        self.Q[3:6, 3:6] *= 0.1    # Velocity (higher noise)
        self.Q[6:10, 6:10] *= 0.01  # Quaternion
        self.Q[10:13, 10:13] *= 0.1  # Angular velocity
        self.Q[13:15, 13:15] *= 0.001  # Arm joints (encoder precise)
        
        # Measurement noise covariance
        self.R_imu = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05])  # acc, quat, omega
        self.R_joints = np.diag([0.001, 0.001])  # Joint positions
        
        # Gravity vector
        self.gravity = np.array([0, 0, 9.81])
        
        # Complementary filter parameters
        self.alpha_vel = 0.98  # Trust integration vs acceleration
        
        # Timing
        self.last_update_time = None
        self.dt = 0.05  # Default time step
        
        # Bias estimates
        self.acc_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        
    def predict(self, control, dt=None):
        """
        Prediction step using control inputs
        
        Args:
            control: [thrust, torque_x, torque_y, torque_z, q_arm_dot1, q_arm_dot2]
            dt: time step (uses self.dt if None)
        """
        if dt is None:
            dt = self.dt
        
        # Extract state
        pos = self.state[0:3]
        vel = self.state[3:6]
        quat = self.state[6:10]
        omega = self.state[10:13]
        q_arm = self.state[13:15]
        
        # Predict position from velocity
        pos_new = pos + vel * dt
        
        # Predict velocity (simplified - assumes small changes)
        vel_new = vel  # Will be corrected in update step
        
        # Predict quaternion from angular velocity
        # Small angle approximation: q_new ≈ q + 0.5 * dt * Omega(omega) * q
        omega_quat = np.array([omega[0], omega[1], omega[2], 0])
        quat_dot = 0.5 * np.array([
            -omega[0]*quat[0] - omega[1]*quat[1] - omega[2]*quat[2],
            omega[0]*quat[3] + omega[2]*quat[1] - omega[1]*quat[2],
            omega[1]*quat[3] - omega[2]*quat[0] + omega[0]*quat[2],
            omega[2]*quat[3] + omega[1]*quat[0] - omega[0]*quat[1]
        ])
        quat_new = normalize_quaternion(quat + quat_dot * dt)
        
        # Predict angular velocity (assumed constant between updates)
        omega_new = omega
        
        # Predict arm joints from control
        q_arm_new = q_arm + control[4:6] * dt
        
        # Update state
        self.state = np.concatenate([pos_new, vel_new, quat_new, omega_new, q_arm_new])
        
        # Update covariance (simplified - just add process noise)
        self.P += self.Q * dt
    
    def update_imu(self, linear_acceleration, orientation, angular_velocity):
        """
        Update state using IMU measurements
        
        Args:
            linear_acceleration: 3D acceleration in body frame (m/s²)
            orientation: quaternion [x, y, z, w]
            angular_velocity: 3D angular velocity in body frame (rad/s)
        """
        # Normalize orientation measurement
        orientation = normalize_quaternion(orientation)
        
        # Update quaternion directly (IMU orientation is reliable)
        self.state[6:10] = orientation
        
        # Remove gyro bias and update angular velocity
        angular_velocity_corrected = angular_velocity - self.gyro_bias
        self.state[10:13] = angular_velocity_corrected
        
        # Transform acceleration to world frame
        acc_world = rotate_vector_by_quaternion(linear_acceleration, orientation)
        
        # Remove gravity and bias
        acc_linear = acc_world - self.gravity - self.acc_bias
        
        # Complementary filter for velocity
        # Integrate acceleration
        vel_from_acc = self.state[3:6] + acc_linear * self.dt
        
        # Blend with current velocity estimate (reduces drift)
        self.state[3:6] = self.alpha_vel * vel_from_acc + (1 - self.alpha_vel) * self.state[3:6]
        
        # Update position from velocity
        self.state[0:3] += self.state[3:6] * self.dt
        
        # Ground constraint
        if self.state[2] < 0:
            self.state[2] = 0
            if self.state[5] < 0:
                self.state[5] = 0
        
        # Reduce covariance for measured states
        self.P[6:13, 6:13] *= 0.9  # Reduce uncertainty in attitude and angular velocity
    
    def update_joints(self, joint_positions):
        """
        Update arm joint states from encoder measurements
        
        Args:
            joint_positions: [q1, q2] arm joint angles (rad)
        """
        # Update arm joint positions directly (encoders are accurate)
        self.state[13:15] = joint_positions
        
        # Reduce covariance for joint states
        self.P[13:15, 13:15] = np.eye(2) * 0.0001
    
    def update_velocity_bias_correction(self, velocity_estimate=None):
        """
        Apply bias correction when external velocity estimate is available
        (e.g., from vision/mocap)
        
        Args:
            velocity_estimate: 3D velocity from external source
        """
        if velocity_estimate is not None:
            # Correct accumulated drift
            vel_error = velocity_estimate - self.state[3:6]
            self.state[3:6] += 0.1 * vel_error  # Slow correction
            
            # Update acceleration bias estimate
            self.acc_bias += 0.01 * vel_error / self.dt
    
    def get_state(self):
        """
        Get current state estimate
        
        Returns:
            state: [pos(3), vel(3), quat(4), omega(3), q_arm(2)]
        """
        return self.state.copy()
    
    def reset(self, initial_state=None):
        """
        Reset estimator to initial state
        
        Args:
            initial_state: optional initial state vector
        """
        if initial_state is not None:
            self.state = initial_state.copy()
        else:
            self.state = np.zeros(15)
            self.state[6:10] = np.array([0, 0, 0, 1])  # Identity quaternion
        
        self.P = np.eye(15) * 0.1
        self.acc_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.last_update_time = None
