#!/usr/bin/env python3
"""
MPC Controller for Aerial Manipulator using Acados

Whole-body MPC controller that uses acados for real-time optimization.
The controller tracks reference trajectories for the quadrotor base while
keeping arm joints fixed (arm accelerations zeroed post-solve).

This controller:
- Uses whole-body state representation (base + arm)
- Solves MPC with SQP_RTI for real-time performance
- Controls the base with thrust and torques
- Includes arm states in optimization but zeroes arm controls after solve
- Subscribes to IMU and joint states for full state estimation

State vector (17 states):
    Base states [0:13]:
        - position (3): x, y, z [m]
        - velocity (3): vx, vy, vz [m/s]
        - quaternion (4): qx, qy, qz, qw
        - angular velocity (3): wx, wy, wz [rad/s]
    Arm states [13:17]:
        - joint positions (2): q1, q2 [rad]
        - joint velocities (2): dq1, dq2 [rad/s]

Control vector (6 controls):
    Base controls [0:4]:
        - thrust: total thrust force [N]
        - tau_x, tau_y, tau_z: body torques [Nm]
    Arm controls [4:6]:
        - ddq1, ddq2: joint accelerations [rad/s^2] (zeroed post-solve)

Dynamics:
    - Base: 6-DOF rigid body with thrust along body z-axis
    - Arm: Simple double integrator (decoupled from base for simplicity)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import threading
from collections import deque
import matplotlib.pyplot as plt

from am_description.mpc_acados.acados_solver import AcadosMPCSolver
from am_description.mpc_acados.acados_model import (
    HOVER_THRUST, MASS, GRAVITY,
    N_STATES, N_CONTROLS, N_BASE_CONTROLS
)


def quaternion_to_euler(q):
    """Convert quaternion [x,y,z,w] to euler angles [roll, pitch, yaw]"""
    qx, qy, qz, qw = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])


def euler_to_quaternion(roll, pitch, yaw):
    """Convert euler angles to quaternion [x,y,z,w]"""
    cr, cp, cy = np.cos(roll/2), np.cos(pitch/2), np.cos(yaw/2)
    sr, sp, sy = np.sin(roll/2), np.sin(pitch/2), np.sin(yaw/2)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return np.array([qx, qy, qz, qw])


class WholeBodyStateEstimator:
    """
    State estimator for aerial manipulator (whole-body)
    
    Estimates base state from IMU and tracks arm joint positions/velocities.
    Full state vector: [base(13), arm_pos(2), arm_vel(2)] = 17 states
    """
    
    def __init__(self):
        # Base state: [pos(3), vel(3), quat(4), omega(3)] = 13 states
        self.base_state = np.zeros(13)
        self.base_state[6:10] = np.array([0, 0, 0, 1])  # Identity quaternion
        
        # Arm state: joint positions and velocities
        self.arm_positions = np.array([0.0, 0.0])
        self.arm_velocities = np.array([0.0, 0.0])
        self.last_arm_positions = None
        self.last_arm_time = None
        
        # Velocity integration
        self.last_time = None
        self.acc_world = np.zeros(3)
        
    def update_imu(self, linear_acc, orientation, angular_vel):
        """
        Update base state from IMU data
        
        Args:
            linear_acc: linear acceleration in body frame [3]
            orientation: quaternion [x,y,z,w]
            angular_vel: angular velocity in body frame [3]
        """
        # Update orientation
        self.base_state[6:10] = orientation / np.linalg.norm(orientation)
        
        # Update angular velocity
        self.base_state[10:13] = angular_vel
        
        # Convert acceleration to world frame and integrate velocity
        quat = self.base_state[6:10]
        qx, qy, qz, qw = quat
        
        # Rotation matrix (body to world)
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # World frame acceleration (remove gravity)
        acc_world = R @ linear_acc - np.array([0, 0, GRAVITY])
        self.acc_world = acc_world
        
        # Integrate velocity
        current_time = time.time()
        if self.last_time is not None:
            dt = min(current_time - self.last_time, 0.1)
            self.base_state[3:6] += acc_world * dt
            self.base_state[0:3] += self.base_state[3:6] * dt
        self.last_time = current_time
    
    def update_joints(self, joint_positions, joint_velocities=None):
        """
        Update arm joint positions and velocities
        
        Args:
            joint_positions: joint positions [2]
            joint_velocities: joint velocities [2] (optional, computed if None)
        """
        new_positions = np.array(joint_positions[:2])
        
        if joint_velocities is not None:
            # Use provided velocities
            self.arm_velocities = np.array(joint_velocities[:2])
        else:
            # Estimate velocities from position differences
            current_time = time.time()
            if self.last_arm_positions is not None and self.last_arm_time is not None:
                dt = current_time - self.last_arm_time
                if dt > 0.001:  # Avoid division by small dt
                    self.arm_velocities = (new_positions - self.last_arm_positions) / dt
            self.last_arm_positions = new_positions.copy()
            self.last_arm_time = current_time
        
        self.arm_positions = new_positions
    
    def get_base_state(self):
        """Get current base state estimate [13]"""
        return self.base_state.copy()
    
    def get_arm_positions(self):
        """Get current arm joint positions [2]"""
        return self.arm_positions.copy()
    
    def get_arm_velocities(self):
        """Get current arm joint velocities [2]"""
        return self.arm_velocities.copy()
    
    def get_arm_state(self):
        """Get current arm state [4] = [positions(2), velocities(2)]"""
        return np.concatenate([self.arm_positions, self.arm_velocities])
    
    def get_full_state(self):
        """Get full state [17] = base(13) + arm_pos(2) + arm_vel(2)"""
        return np.concatenate([self.base_state, self.arm_positions, self.arm_velocities])
    
    def reset(self):
        """Reset state estimator"""
        self.base_state = np.zeros(13)
        self.base_state[6:10] = np.array([0, 0, 0, 1])
        self.arm_positions = np.array([0.0, 0.0])
        self.arm_velocities = np.array([0.0, 0.0])
        self.last_arm_positions = None
        self.last_arm_time = None
        self.last_time = None


class MPCControllerAcados(Node):
    """
    ROS2 node for aerial manipulator MPC control using acados
    
    Controls the whole body but keeps arm joints fixed.
    """
    
    def __init__(self):
        super().__init__('mpc_controller_acados')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_rate', 20.0),
                ('trajectory_mode', 'hover'),
                ('trajectory_radius', 1.5),
                ('trajectory_speed', 0.3),
                ('trajectory_height', 2.0),
                ('arm_fixed_position', [0.0, 0.0]),
            ]
        )
        
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        
        # MPC parameters
        mpc_params = {
            'N_horizon': 20,
            'dt': 0.05,
            'Q_pos': 10.0,
            'Q_vel': 1.0,
            'Q_att': 5.0,
            'Q_omega': 0.5,
            'R_thrust': 0.01,
            'R_torque': 0.1,
            'Q_terminal_factor': 2.0,
            'nlp_solver_type': 'SQP_RTI',
        }
        
        # Initialize MPC solver
        self.get_logger().info('Initializing acados MPC solver for aerial manipulator...')
        self.mpc_solver = AcadosMPCSolver(mpc_params)
        
        # State estimator
        self.state_estimator = WholeBodyStateEstimator()
        
        # Trajectory parameters
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_radius = self.get_parameter('trajectory_radius').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        self.trajectory_height = self.get_parameter('trajectory_height').value
        self.trajectory_time = 0.0
        
        # Arm control (keep fixed at this position)
        arm_pos_param = self.get_parameter('arm_fixed_position').value
        self.arm_fixed_position = np.array(arm_pos_param) if arm_pos_param else np.array([0.0, 0.0])
        
        # Control state
        self.control_enabled = False
        self.imu_received = False
        self.joints_received = False
        
        # Data logging
        self.max_log_size = 5000
        self.log_time = deque(maxlen=self.max_log_size)
        self.log_pos = deque(maxlen=self.max_log_size)
        self.log_pos_ref = deque(maxlen=self.max_log_size)
        self.log_att = deque(maxlen=self.max_log_size)
        self.log_att_ref = deque(maxlen=self.max_log_size)
        self.log_control = deque(maxlen=self.max_log_size)
        self.log_arm = deque(maxlen=self.max_log_size)
        self.log_solve_time = deque(maxlen=self.max_log_size)
        self.start_log_time = None
        
        # Performance metrics
        self.solve_times = deque(maxlen=100)
        self.tracking_errors = deque(maxlen=100)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/aerial_manipulator/imu',
            self.imu_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publishers
        self.thrust_pub = self.create_publisher(
            Wrench,
            '/aerial_manipulator/thrust',
            10
        )
        
        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('MPC Controller (Acados) for Aerial Manipulator')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Prediction horizon: {mpc_params["N_horizon"]} steps')
        self.get_logger().info(f'Arm mode: FIXED at {self.arm_fixed_position}')
        self.get_logger().info('Commands: start [mode], stop, plot, stats, quit')
        self.get_logger().info('Modes: hover, circle, square, figure8, takeoff, up')
        self.get_logger().info('=' * 50)
    
    def imu_callback(self, msg: Imu):
        """Update state estimator with IMU data"""
        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.state_estimator.update_imu(linear_acc, orientation, angular_vel)
        self.imu_received = True
    
    def joint_callback(self, msg: JointState):
        """Update state estimator with joint encoder data"""
        if len(msg.position) >= 2:
            joint_positions = np.array(msg.position[0:2])
            # Extract velocities if available, otherwise use zeros
            joint_velocities = np.zeros(2)
            if hasattr(msg, 'velocity') and len(msg.velocity) >= 2:
                joint_velocities = np.array(msg.velocity[0:2])
            self.state_estimator.update_joints(joint_positions, joint_velocities)
            self.joints_received = True
    
    def generate_reference_trajectory(self, current_time, horizon):
        """
        Generate reference trajectory for MPC
        
        Args:
            current_time: current trajectory time
            horizon: number of steps
        
        Returns:
            x_ref: reference trajectory [horizon+1 x 17]
                   Base states [0:13] + arm_pos [13:15] + arm_vel [15:17]
        """
        x_ref = np.zeros((horizon + 1, N_STATES))  # 17 states
        dt = self.mpc_solver.params['dt']
        
        for k in range(horizon + 1):
            t = current_time + k * dt
            
            if self.trajectory_mode == 'hover':
                # Simple hover at target height
                x_ref[k, 2] = self.trajectory_height
                x_ref[k, 6:10] = [0, 0, 0, 1]  # Level attitude
                
            elif self.trajectory_mode == 'takeoff':
                # Smooth takeoff trajectory
                takeoff_duration = 5.0
                if t < takeoff_duration:
                    # 5th-order polynomial for smooth takeoff
                    tau = t / takeoff_duration
                    s = 10*tau**3 - 15*tau**4 + 6*tau**5
                    s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / takeoff_duration
                    x_ref[k, 2] = s * self.trajectory_height
                    x_ref[k, 5] = s_dot * self.trajectory_height
                else:
                    x_ref[k, 2] = self.trajectory_height
                x_ref[k, 6:10] = [0, 0, 0, 1]
                
            elif self.trajectory_mode == 'circle':
                # Circular trajectory
                omega = self.trajectory_speed / self.trajectory_radius
                x_ref[k, 0] = self.trajectory_radius * np.cos(omega * t)
                x_ref[k, 1] = self.trajectory_radius * np.sin(omega * t)
                x_ref[k, 2] = self.trajectory_height
                x_ref[k, 3] = -self.trajectory_radius * omega * np.sin(omega * t)
                x_ref[k, 4] = self.trajectory_radius * omega * np.cos(omega * t)
                
                # Calculate required attitude for trajectory
                acc_x = -self.trajectory_radius * omega**2 * np.cos(omega * t)
                acc_y = -self.trajectory_radius * omega**2 * np.sin(omega * t)
                acc_z = 0
                
                # Thrust direction
                thrust_vec = np.array([acc_x, acc_y, acc_z + GRAVITY])
                thrust_mag = np.linalg.norm(thrust_vec)
                z_body = thrust_vec / thrust_mag
                
                # Compute roll and pitch
                pitch = np.arcsin(np.clip(z_body[0], -1, 1))
                roll = np.arctan2(-z_body[1], z_body[2])
                yaw = 0.0
                
                x_ref[k, 6:10] = euler_to_quaternion(roll, pitch, yaw)
                
            elif self.trajectory_mode == 'square':
                # Square trajectory
                period = 4 * self.trajectory_radius / self.trajectory_speed
                phase = t % period
                segment = int(4 * phase / period)
                segment_time = phase - segment * period / 4
                
                r = self.trajectory_radius
                v = self.trajectory_speed
                
                if segment == 0:
                    x_ref[k, 0] = -r + v * segment_time
                    x_ref[k, 1] = -r
                    x_ref[k, 3] = v
                elif segment == 1:
                    x_ref[k, 0] = r
                    x_ref[k, 1] = -r + v * segment_time
                    x_ref[k, 4] = v
                elif segment == 2:
                    x_ref[k, 0] = r - v * segment_time
                    x_ref[k, 1] = r
                    x_ref[k, 3] = -v
                else:
                    x_ref[k, 0] = -r
                    x_ref[k, 1] = r - v * segment_time
                    x_ref[k, 4] = -v
                
                x_ref[k, 2] = self.trajectory_height
                x_ref[k, 6:10] = [0, 0, 0, 1]
                
            elif self.trajectory_mode == 'figure8':
                # Figure-8 trajectory
                omega = self.trajectory_speed / self.trajectory_radius
                x_ref[k, 0] = self.trajectory_radius * np.sin(omega * t)
                x_ref[k, 1] = self.trajectory_radius * np.sin(2 * omega * t) / 2
                x_ref[k, 2] = self.trajectory_height
                x_ref[k, 3] = self.trajectory_radius * omega * np.cos(omega * t)
                x_ref[k, 4] = self.trajectory_radius * omega * np.cos(2 * omega * t)
                x_ref[k, 6:10] = [0, 0, 0, 1]
            
            elif self.trajectory_mode == 'up':
                # Gradual ascent from ground level
                ascent_speed = 0.5  # m/s vertical speed
                target_height = self.trajectory_height
                current_height = ascent_speed * t
                
                if current_height < target_height:
                    x_ref[k, 2] = current_height
                    x_ref[k, 5] = ascent_speed  # vz
                else:
                    x_ref[k, 2] = target_height
                    x_ref[k, 5] = 0.0  # stop ascending
                
                x_ref[k, 6:10] = [0, 0, 0, 1]  # Level attitude
        
        return x_ref
    
    def control_loop(self):
        """Main MPC control loop"""
        if not self.control_enabled:
            return
        
        if not self.imu_received:
            self.get_logger().warn('Waiting for IMU data...', throttle_duration_sec=1.0)
            return
        
        # Note: We proceed even without joint state since we're keeping arm fixed
        # Arm states will be zeros if not received
        
        if self.start_log_time is None:
            self.start_log_time = time.time()
        
        # Get full state (17 states: base + arm)
        x_current = self.state_estimator.get_full_state()
        
        # Generate reference trajectory
        N = self.mpc_solver.N
        x_ref = self.generate_reference_trajectory(self.trajectory_time, N)
        
        # Solve MPC (with arm controls zeroed post-solve)
        solve_start = time.time()
        try:
            u_opt, x_pred, info = self.mpc_solver.solve(x_current, x_ref, zero_arm_controls=True)
            solve_time = time.time() - solve_start
            self.solve_times.append(solve_time * 1000)
            
            if not info['success']:
                self.get_logger().warn(f'MPC solve failed: status={info["status"]}')
            
            # Apply first control - base controls (u_opt is 6-dim: thrust, tau_x, tau_y, tau_z, ddq1, ddq2)
            thrust = u_opt[0, 0]  # CTRL_THRUST
            tau_x = u_opt[0, 1]   # CTRL_TAU_X
            tau_y = u_opt[0, 2]   # CTRL_TAU_Y
            tau_z = u_opt[0, 3]   # CTRL_TAU_Z
            # u_opt[0, 4:6] are arm accelerations (zeroed by zero_arm_controls=True)
            
            # Publish base wrench
            msg = Wrench()
            msg.force.z = thrust
            msg.torque.x = tau_x
            msg.torque.y = tau_y
            msg.torque.z = tau_z
            self.thrust_pub.publish(msg)
            
            # Publish arm commands (zero velocity to keep fixed)
            arm_cmd = Float64MultiArray()
            arm_cmd.data = [0.0, 0.0]  # Zero velocity for both joints
            self.arm_cmd_pub.publish(arm_cmd)
            
            # Log data
            log_t = time.time() - self.start_log_time
            self.log_time.append(log_t)
            self.log_pos.append(x_current[0:3].copy())
            self.log_pos_ref.append(x_ref[0, 0:3].copy())
            self.log_att.append(quaternion_to_euler(x_current[6:10]))
            self.log_att_ref.append(quaternion_to_euler(x_ref[0, 6:10]))
            self.log_control.append([thrust, tau_x, tau_y, tau_z])
            self.log_arm.append(self.state_estimator.get_arm_state())
            self.log_solve_time.append(solve_time * 1000)
            
            # Track errors
            pos_error = np.linalg.norm(x_current[0:3] - x_ref[0, 0:3])
            self.tracking_errors.append(pos_error)
            
            # Periodic status logging
            if int(self.trajectory_time / self.dt) % 40 == 0:  # Every 2 seconds
                avg_solve_time = np.mean(self.solve_times) if self.solve_times else 0
                self.get_logger().info(
                    f'Mode: {self.trajectory_mode} | '
                    f'Pos: [{x_current[0]:.2f}, {x_current[1]:.2f}, {x_current[2]:.2f}] | '
                    f'Error: {pos_error:.3f}m | '
                    f'Thrust: {thrust:.1f}N | '
                    f'Solve: {solve_time*1000:.1f}ms'
                )
            
        except Exception as e:
            self.get_logger().error(f'MPC error: {e}')
            # Apply hover thrust
            msg = Wrench()
            msg.force.z = HOVER_THRUST
            self.thrust_pub.publish(msg)
            
            # Still send zero arm command
            arm_cmd = Float64MultiArray()
            arm_cmd.data = [0.0, 0.0]
            self.arm_cmd_pub.publish(arm_cmd)
        
        self.trajectory_time += self.dt
    
    def start_trajectory(self, mode='hover'):
        """Start trajectory following"""
        self.trajectory_mode = mode
        self.trajectory_time = 0.0
        self.control_enabled = True
        self.state_estimator.reset()
        self.mpc_solver.reset()
        self.start_log_time = None
        self.get_logger().info(f'Started MPC trajectory: {mode}')
        self.get_logger().info(f'Arm joints: FIXED at {self.arm_fixed_position}')
    
    def stop(self):
        """Stop controller"""
        self.control_enabled = False
        
        # Send zero thrust
        msg = Wrench()
        msg.force.z = 0.0
        self.thrust_pub.publish(msg)
        
        # Send zero arm velocity
        arm_cmd = Float64MultiArray()
        arm_cmd.data = [0.0, 0.0]
        self.arm_cmd_pub.publish(arm_cmd)
        
        self.get_logger().info('MPC controller stopped')
    
    def plot_data(self):
        """Plot logged data"""
        if len(self.log_time) == 0:
            self.get_logger().warn('No data to plot')
            return
        
        t = np.array(self.log_time)
        pos = np.array(self.log_pos)
        pos_ref = np.array(self.log_pos_ref)
        att = np.array(self.log_att)
        att_ref = np.array(self.log_att_ref)
        control = np.array(self.log_control)
        arm = np.array(self.log_arm) if self.log_arm else None
        
        fig, axes = plt.subplots(4, 3, figsize=(14, 12))
        
        # Position plots
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            axes[0, i].plot(t, pos[:, i], 'b-', label='Actual', linewidth=1.5)
            axes[0, i].plot(t, pos_ref[:, i], 'r--', label='Reference', linewidth=1)
            axes[0, i].set_ylabel(f'{labels[i]} Position (m)')
            axes[0, i].grid(True, alpha=0.3)
            if i == 0:
                axes[0, i].legend()
                axes[0, i].set_title('Position Tracking')
        
        # Attitude plots
        att_labels = ['Roll', 'Pitch', 'Yaw']
        for i in range(3):
            axes[1, i].plot(t, np.degrees(att[:, i]), 'b-', label='Actual', linewidth=1.5)
            axes[1, i].plot(t, np.degrees(att_ref[:, i]), 'r--', label='Reference', linewidth=1)
            axes[1, i].set_ylabel(f'{att_labels[i]} (deg)')
            axes[1, i].grid(True, alpha=0.3)
            if i == 0:
                axes[1, i].legend()
                axes[1, i].set_title('Attitude')
        
        # Control plots
        control_labels = ['Thrust (N)', 'Torque X (Nm)', 'Torque Y (Nm)']
        for i in range(3):
            axes[2, i].plot(t, control[:, i], 'g-', linewidth=1)
            axes[2, i].set_ylabel(control_labels[i])
            axes[2, i].grid(True, alpha=0.3)
            if i == 0:
                axes[2, i].axhline(y=HOVER_THRUST, color='k', linestyle='--', alpha=0.5, label=f'Hover: {HOVER_THRUST:.1f}N')
                axes[2, i].legend()
                axes[2, i].set_title('Base Control Inputs')
        
        # Arm and solve time plots
        if arm is not None and len(arm) > 0:
            axes[3, 0].plot(t, arm[:, 0], 'b-', label='Joint 1', linewidth=1.5)
            axes[3, 0].plot(t, arm[:, 1], 'r-', label='Joint 2', linewidth=1.5)
            axes[3, 0].set_ylabel('Joint Position (rad)')
            axes[3, 0].set_xlabel('Time (s)')
            axes[3, 0].legend()
            axes[3, 0].grid(True, alpha=0.3)
            axes[3, 0].set_title('Arm Joints (Fixed)')
        
        axes[3, 1].plot(t, control[:, 3], 'g-', linewidth=1)
        axes[3, 1].set_ylabel('Torque Z (Nm)')
        axes[3, 1].set_xlabel('Time (s)')
        axes[3, 1].grid(True, alpha=0.3)
        
        solve_times = np.array(self.log_solve_time)
        axes[3, 2].plot(t, solve_times, 'm-', linewidth=1)
        axes[3, 2].axhline(y=self.dt * 1000, color='r', linestyle='--', alpha=0.7, label=f'Control period: {self.dt*1000:.1f}ms')
        axes[3, 2].set_ylabel('Solve Time (ms)')
        axes[3, 2].set_xlabel('Time (s)')
        axes[3, 2].legend()
        axes[3, 2].grid(True, alpha=0.3)
        axes[3, 2].set_title('MPC Solve Time')
        
        plt.tight_layout()
        plt.savefig('mpc_acados_results.png', dpi=150)
        plt.show()
        self.get_logger().info('Plot saved to mpc_acados_results.png')
    
    def print_stats(self):
        """Print performance statistics"""
        if len(self.solve_times) == 0:
            self.get_logger().info('No solve time data')
            return
        
        times = np.array(self.solve_times)
        errors = np.array(self.tracking_errors) if self.tracking_errors else np.array([0])
        
        print("\n" + "=" * 60)
        print("MPC Controller (Acados) Performance Statistics")
        print("=" * 60)
        print(f"Solve Time (ms):")
        print(f"  Mean:   {np.mean(times):.2f}")
        print(f"  Median: {np.median(times):.2f}")
        print(f"  Min:    {np.min(times):.2f}")
        print(f"  Max:    {np.max(times):.2f}")
        print(f"  Std:    {np.std(times):.2f}")
        print(f"\nTracking Error (m):")
        print(f"  Mean:   {np.mean(errors):.4f}")
        print(f"  Max:    {np.max(errors):.4f}")
        print(f"  RMS:    {np.sqrt(np.mean(errors**2)):.4f}")
        print(f"\nControl period: {self.dt * 1000:.1f} ms")
        print(f"Arm mode: FIXED")
        
        if np.max(times) < self.dt * 1000:
            print("\nStatus: REAL-TIME FEASIBLE")
        else:
            print("\nStatus: NOT real-time feasible")
        print("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    controller = MPCControllerAcados()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    def command_loop():
        """Interactive command loop"""
        print("\n" + "=" * 60)
        print("MPC Controller (Acados) - Aerial Manipulator")
        print("=" * 60)
        print("Commands:")
        print("  start [mode]  - Start trajectory (hover/circle/square/figure8/takeoff/up)")
        print("  stop          - Stop controller")
        print("  plot          - Plot logged data")
        print("  stats         - Show solver statistics")
        print("  status        - Show current status")
        print("  quit          - Exit")
        print("=" * 60 + "\n")
        
        while rclpy.ok():
            try:
                cmd = input(">>> ").strip().lower()
                
                if cmd.startswith('start'):
                    parts = cmd.split()
                    mode = parts[1] if len(parts) > 1 else 'hover'
                    if mode in ['hover', 'circle', 'square', 'figure8', 'takeoff', 'up']:
                        controller.start_trajectory(mode)
                    else:
                        print(f"Unknown mode: {mode}")
                        print("Available: hover, circle, square, figure8, takeoff, up")
                    
                elif cmd == 'stop':
                    controller.stop()
                    
                elif cmd == 'plot':
                    controller.plot_data()
                    
                elif cmd == 'stats':
                    controller.print_stats()
                    
                elif cmd == 'status':
                    state = controller.state_estimator.get_full_state()
                    print(f"Control enabled: {controller.control_enabled}")
                    print(f"Mode: {controller.trajectory_mode}")
                    print(f"Base position: [{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}]")
                    print(f"Arm joints: [{state[13]:.3f}, {state[14]:.3f}]")
                    print(f"IMU received: {controller.imu_received}")
                    print(f"Joints received: {controller.joints_received}")
                    print(f"Logged samples: {len(controller.log_time)}")
                    
                elif cmd in ['quit', 'exit', 'q']:
                    controller.stop()
                    rclpy.shutdown()
                    break
                    
                elif cmd == '':
                    continue
                    
                else:
                    print(f"Unknown command: {cmd}")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                controller.stop()
                break
    
    cmd_thread = threading.Thread(target=command_loop, daemon=True)
    cmd_thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
