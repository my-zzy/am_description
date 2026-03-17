#!/usr/bin/env python3
"""
MPC Controller for Aerial Manipulator End-Effector Tracking

Whole-body MPC controller that tracks end-effector trajectories.
The controller actively controls both the base and arm joints to make
the end-effector follow desired trajectories in world frame.

This controller:
- Uses whole-body state representation (base + arm)
- Computes inverse kinematics for EE trajectory to joint references
- Actively controls arm joints (no zeroing)
- Tracks both base position and EE position simultaneously

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
        - ddq1, ddq2: joint accelerations [rad/s^2] (ACTIVE)

Dynamics:
    - Base: 6-DOF rigid body with thrust along body z-axis
    - Arm: Simple double integrator (decoupled from base)
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

from am_description.mpc_ef.acados_solver import AcadosMPCSolver, ee_body_to_joint_ref
from am_description.mpc_ef.acados_model import (
    HOVER_THRUST, MASS, GRAVITY,
    N_STATES, N_CONTROLS,
    forward_kinematics, forward_kinematics_body, compute_ee_from_state,
    L1, L2, ARM_MOUNT_Z
)


def quaternion_to_euler(q):
    """Convert quaternion [x,y,z,w] to euler angles [roll, pitch, yaw]"""
    qx, qy, qz, qw = q
    
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    
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
    
    Full state vector: [base(13), arm_pos(2), arm_vel(2)] = 17 states
    """
    
    def __init__(self):
        self.base_state = np.zeros(13)
        self.base_state[6:10] = np.array([0, 0, 0, 1])
        
        self.arm_positions = np.array([0.0, 0.0])
        self.arm_velocities = np.array([0.0, 0.0])
        self.last_arm_positions = None
        self.last_arm_time = None
        
        self.last_time = None
        self.acc_world = np.zeros(3)
        
    def update_imu(self, linear_acc, orientation, angular_vel):
        """Update base state from IMU data"""
        self.base_state[6:10] = orientation / np.linalg.norm(orientation)
        self.base_state[10:13] = angular_vel
        
        quat = self.base_state[6:10]
        qx, qy, qz, qw = quat
        
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        
        acc_world = R @ linear_acc - np.array([0, 0, GRAVITY])
        self.acc_world = acc_world
        
        current_time = time.time()
        if self.last_time is not None:
            dt = min(current_time - self.last_time, 0.1)
            self.base_state[3:6] += acc_world * dt
            self.base_state[0:3] += self.base_state[3:6] * dt
        self.last_time = current_time
    
    def update_joints(self, joint_positions, joint_velocities=None):
        """Update arm joint positions and velocities"""
        new_positions = np.array(joint_positions[:2])
        
        if joint_velocities is not None:
            self.arm_velocities = np.array(joint_velocities[:2])
        else:
            current_time = time.time()
            if self.last_arm_positions is not None and self.last_arm_time is not None:
                dt = current_time - self.last_arm_time
                if dt > 0.001:
                    self.arm_velocities = (new_positions - self.last_arm_positions) / dt
            self.last_arm_positions = new_positions.copy()
            self.last_arm_time = current_time
        
        self.arm_positions = new_positions
    
    def get_base_state(self):
        return self.base_state.copy()
    
    def get_arm_positions(self):
        return self.arm_positions.copy()
    
    def get_arm_velocities(self):
        return self.arm_velocities.copy()
    
    def get_arm_state(self):
        return np.concatenate([self.arm_positions, self.arm_velocities])
    
    def get_full_state(self):
        return np.concatenate([self.base_state, self.arm_positions, self.arm_velocities])
    
    def reset(self):
        self.base_state = np.zeros(13)
        self.base_state[6:10] = np.array([0, 0, 0, 1])
        self.arm_positions = np.array([0.0, 0.0])
        self.arm_velocities = np.array([0.0, 0.0])
        self.last_arm_positions = None
        self.last_arm_time = None
        self.last_time = None


class MPCControllerEF(Node):
    """
    ROS2 node for aerial manipulator MPC end-effector control
    
    Tracks end-effector trajectories by controlling both base and arm.
    """
    
    def __init__(self):
        super().__init__('mpc_controller_ef')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_rate', 20.0),
                ('cost_mode', 'ik'),  # 'ik' (existing) or 'ee' (direct EE penalty)
                ('trajectory_mode', 'hover'),
                ('trajectory_radius', 0.15),  # Smaller for EE motion
                ('trajectory_speed', 0.1),
                ('trajectory_height', 2.0),
                ('ee_offset_z', -0.3),  # EE offset below base (extended arm)
            ]
        )
        
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        
        # MPC parameters
        self.cost_mode = str(self.get_parameter('cost_mode').value).strip().lower()
        mpc_params = {
            'N_horizon': 20,
            'dt': 0.05,
            'cost_mode': self.cost_mode,
            'Q_pos': 5.0,
            'Q_vel': 1.0,
            'Q_att': 5.0,
            'Q_omega': 0.5,
            'Q_arm_pos': 20.0,  # High for EE tracking
            'Q_arm_vel': 1.0,
            # Direct EE penalty weights (used when cost_mode='ee')
            'Q_ee_pos': 50.0,
            'Q_level': 5.0,
            'Q_arm_pos_ee': 0.5,
            'Q_arm_vel_ee': 0.2,
            'R_thrust': 0.01,
            'R_torque': 0.1,
            'R_arm_acc': 0.01,
            'Q_terminal_factor': 2.0,
            'nlp_solver_type': 'SQP_RTI',
        }
        
        self.get_logger().info('Initializing acados MPC solver for end-effector control...')
        self.mpc_solver = AcadosMPCSolver(mpc_params)
        
        self.state_estimator = WholeBodyStateEstimator()
        
        # Trajectory parameters
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_radius = self.get_parameter('trajectory_radius').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        self.trajectory_height = self.get_parameter('trajectory_height').value
        self.ee_offset_z = self.get_parameter('ee_offset_z').value
        self.trajectory_time = 0.0

        # Yaw reference helper (used to rotate the arm plane in world)
        self._yaw_ref_last = 0.0
        
        # Initial arm configuration (extended down: q1=0, q2=π)
        self.arm_default_joints = np.array([0.0, np.pi])
        
        # Control state
        self.control_enabled = False
        self.imu_received = False
        self.joints_received = False
        
        # Data logging
        self.max_log_size = 5000
        self.log_time = deque(maxlen=self.max_log_size)
        self.log_pos = deque(maxlen=self.max_log_size)
        self.log_pos_ref = deque(maxlen=self.max_log_size)
        self.log_ee = deque(maxlen=self.max_log_size)
        self.log_ee_ref = deque(maxlen=self.max_log_size)
        self.log_att = deque(maxlen=self.max_log_size)
        self.log_control = deque(maxlen=self.max_log_size)
        self.log_arm = deque(maxlen=self.max_log_size)
        self.log_solve_time = deque(maxlen=self.max_log_size)
        self.start_log_time = None
        
        # Performance metrics
        self.solve_times = deque(maxlen=100)
        self.ee_errors = deque(maxlen=100)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/aerial_manipulator/imu', self.imu_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Publishers
        self.thrust_pub = self.create_publisher(
            Wrench, '/aerial_manipulator/thrust', 10)
        
        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('MPC Controller for End-Effector Tracking')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Prediction horizon: {mpc_params["N_horizon"]} steps')
        self.get_logger().info(f'Cost mode: {self.cost_mode}')
        self.get_logger().info('Arm mode: ACTIVE (end-effector tracking)')
        self.get_logger().info('Commands: start [mode], stop, plot, stats, quit')
        self.get_logger().info('EE Modes: hover, circle, line, reach')
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
            joint_velocities = np.zeros(2)
            if hasattr(msg, 'velocity') and len(msg.velocity) >= 2:
                joint_velocities = np.array(msg.velocity[0:2])
            self.state_estimator.update_joints(joint_positions, joint_velocities)
            self.joints_received = True
    
    def get_ee_trajectory_point(self, t, current_state):
        """
        Get end-effector trajectory point at time t
        
        Args:
            t: trajectory time
            current_state: current full state [17]
        
        Returns:
            base_pos_ref: reference base position [3]
            ee_world_ref: reference EE position in world frame [3]
            base_quat_ref: reference base orientation [4]
        """
        # Base position reference (stationary or moving)
        base_pos_ref = np.array([0.0, 0.0, self.trajectory_height])
        
        # Default EE position (below base with extended arm)
        ee_default = base_pos_ref + np.array([0.0, 0.0, self.ee_offset_z])
        
        if self.trajectory_mode == 'hover':
            # Keep EE at default position
            ee_world_ref = ee_default
            
        elif self.trajectory_mode == 'circle':
            # EE traces a circle in XY plane
            omega = self.trajectory_speed / self.trajectory_radius
            x_offset = self.trajectory_radius * np.cos(omega * t)
            y_offset = self.trajectory_radius * np.sin(omega * t)
            ee_world_ref = ee_default + np.array([x_offset, y_offset, 0.0])
            
        elif self.trajectory_mode == 'line':
            # EE moves back and forth along X axis
            period = 4.0  # seconds
            phase = (t % period) / period
            if phase < 0.5:
                x_offset = self.trajectory_radius * (4 * phase - 1)
            else:
                x_offset = self.trajectory_radius * (3 - 4 * phase)
            ee_world_ref = ee_default + np.array([x_offset, 0.0, 0.0])
            
        elif self.trajectory_mode == 'vertical':
            # EE moves up and down
            period = 4.0
            phase = (t % period) / period
            z_offset = 0.1 * np.sin(2 * np.pi * phase)
            ee_world_ref = ee_default + np.array([0.0, 0.0, z_offset])
            
        elif self.trajectory_mode == 'reach':
            # Smooth reach motion forward
            reach_duration = 3.0
            if t < reach_duration:
                tau = t / reach_duration
                s = 10*tau**3 - 15*tau**4 + 6*tau**5  # Smooth step
                x_offset = s * 0.2  # Reach 20cm forward
            else:
                x_offset = 0.2
            ee_world_ref = ee_default + np.array([x_offset, 0.0, 0.0])
            
        else:
            ee_world_ref = ee_default

        # Compute a yaw reference so the arm's (body-frame) XZ plane can track
        # world-frame XY motion (e.g., a circle). With two parallel joint axes,
        # the arm cannot generate body-frame Y motion; yawing the base rotates
        # the arm plane in world.
        ee_vec_world = ee_world_ref - base_pos_ref
        xy_norm = float(np.hypot(ee_vec_world[0], ee_vec_world[1]))
        if xy_norm > 1e-6:
            yaw_ref = float(np.arctan2(ee_vec_world[1], ee_vec_world[0]))
            self._yaw_ref_last = yaw_ref
        else:
            yaw_ref = self._yaw_ref_last

        base_quat_ref = euler_to_quaternion(0.0, 0.0, yaw_ref)  # roll, pitch, yaw

        return base_pos_ref, ee_world_ref, base_quat_ref
    
    def generate_reference_trajectory(self, current_time, horizon, current_state):
        """
        Generate reference trajectory for MPC
        
        Args:
            current_time: current trajectory time
            horizon: number of steps
            current_state: current full state [17]
        
        Returns:
            x_ref: reference trajectory [horizon+1 x 17]
            ee_refs: end-effector reference positions [horizon+1 x 3]
        """
        x_ref = np.zeros((horizon + 1, N_STATES))
        ee_refs = np.zeros((horizon + 1, 3))
        dt = self.mpc_solver.params['dt']
        
        current_joints = current_state[13:15]
        current_quat = current_state[6:10]
        current_yaw = float(quaternion_to_euler(current_quat)[2])
        
        for k in range(horizon + 1):
            t = current_time + k * dt
            
            base_pos_ref, ee_world_ref, base_quat_ref = self.get_ee_trajectory_point(t, current_state)
            ee_refs[k] = ee_world_ref

            if self.cost_mode == 'ik':
                # Compute full state reference from EE position via IK -> joint references
                x_ref[k] = self.mpc_solver.compute_ee_reference(
                    base_pos_ref, ee_world_ref, base_quat_ref, current_joints
                )

                # Update current_joints for continuity in IK
                current_joints = x_ref[k, 13:15]
            else:
                # Direct EE penalty mode: do NOT use IK.
                # Provide a gentle regularization reference only.
                x_ref_k = np.zeros(N_STATES)
                x_ref_k[0:3] = base_pos_ref
                x_ref_k[3:6] = 0.0

                # Keep level (roll=pitch=0) but don't constrain yaw:
                # - cost_mode='ee' only penalizes qx,qy, so yaw is free
                # - keeping yaw continuous avoids large quaternion jumps
                x_ref_k[6:10] = euler_to_quaternion(0.0, 0.0, current_yaw)
                x_ref_k[10:13] = 0.0

                # Keep arm near current configuration with low weight (weights set in solver params)
                x_ref_k[13:15] = current_joints
                x_ref_k[15:17] = 0.0

                x_ref[k] = x_ref_k
        
        return x_ref, ee_refs
    
    def control_loop(self):
        """Main MPC control loop"""
        if not self.control_enabled:
            return
        
        if not self.imu_received:
            self.get_logger().warn('Waiting for IMU data...', throttle_duration_sec=1.0)
            return
        
        if self.start_log_time is None:
            self.start_log_time = time.time()
        
        # Get full state
        x_current = self.state_estimator.get_full_state()
        
        # Generate reference trajectory
        N = self.mpc_solver.N
        x_ref, ee_refs = self.generate_reference_trajectory(self.trajectory_time, N, x_current)
        
        # Solve MPC
        solve_start = time.time()
        try:
            if self.cost_mode == 'ik':
                u_opt, x_pred, info = self.mpc_solver.solve(x_current, x_ref)
            else:
                u_opt, x_pred, info = self.mpc_solver.solve(x_current, x_ref, ee_ref_trajectory=ee_refs)
            solve_time = time.time() - solve_start
            self.solve_times.append(solve_time * 1000)
            
            if not info['success']:
                self.get_logger().warn(f'MPC solve failed: status={info["status"]}')
            
            # Apply first control
            thrust = u_opt[0, 0]
            tau_x = u_opt[0, 1]
            tau_y = u_opt[0, 2]
            tau_z = u_opt[0, 3]
            ddq1 = u_opt[0, 4]
            ddq2 = u_opt[0, 5]
            
            # Publish base wrench
            msg = Wrench()
            msg.force.z = thrust
            msg.torque.x = tau_x
            msg.torque.y = tau_y
            msg.torque.z = tau_z
            self.thrust_pub.publish(msg)
            
            # Publish arm commands
            # For position-controlled arm, integrate acceleration to velocity command
            # Or use predicted joint positions directly
            arm_cmd = Float64MultiArray()
            # Use predicted next joint positions as command
            next_joints = x_pred[1, 13:15]
            arm_cmd.data = [float(next_joints[0]), float(next_joints[1])]
            self.arm_cmd_pub.publish(arm_cmd)
            
            # Compute actual EE position
            ee_actual = compute_ee_from_state(x_current)
            ee_ref = ee_refs[0]
            ee_error = np.linalg.norm(ee_actual - ee_ref)
            self.ee_errors.append(ee_error)
            
            # Log data
            log_t = time.time() - self.start_log_time
            self.log_time.append(log_t)
            self.log_pos.append(x_current[0:3].copy())
            self.log_pos_ref.append(x_ref[0, 0:3].copy())
            self.log_ee.append(ee_actual.copy())
            self.log_ee_ref.append(ee_ref.copy())
            self.log_att.append(quaternion_to_euler(x_current[6:10]))
            self.log_control.append([thrust, tau_x, tau_y, tau_z, ddq1, ddq2])
            self.log_arm.append(self.state_estimator.get_arm_state())
            self.log_solve_time.append(solve_time * 1000)
            
            # Periodic status logging
            if int(self.trajectory_time / self.dt) % 40 == 0:
                self.get_logger().info(
                    f'Mode: {self.trajectory_mode} | '
                    f'EE: [{ee_actual[0]:.2f}, {ee_actual[1]:.2f}, {ee_actual[2]:.2f}] | '
                    f'EE Error: {ee_error:.3f}m | '
                    f'Joints: [{x_current[13]:.2f}, {x_current[14]:.2f}] | '
                    f'Solve: {solve_time*1000:.1f}ms'
                )
            
        except Exception as e:
            self.get_logger().error(f'MPC error: {e}')
            import traceback
            traceback.print_exc()
            
            # Apply hover thrust
            msg = Wrench()
            msg.force.z = HOVER_THRUST
            self.thrust_pub.publish(msg)
        
        self.trajectory_time += self.dt
    
    def start_trajectory(self, mode='hover'):
        """Start trajectory following"""
        self.trajectory_mode = mode
        self.trajectory_time = 0.0
        self.control_enabled = True
        self.state_estimator.reset()
        self.mpc_solver.reset()
        self.start_log_time = None
        self.get_logger().info(f'Started EE trajectory: {mode}')
    
    def stop(self):
        """Stop controller"""
        self.control_enabled = False
        
        msg = Wrench()
        msg.force.z = 0.0
        self.thrust_pub.publish(msg)
        
        self.get_logger().info('MPC controller stopped')
    
    def plot_data(self):
        """Plot logged data"""
        if len(self.log_time) == 0:
            self.get_logger().warn('No data to plot')
            return
        
        t = np.array(self.log_time)
        pos = np.array(self.log_pos)
        pos_ref = np.array(self.log_pos_ref)
        ee = np.array(self.log_ee)
        ee_ref = np.array(self.log_ee_ref)
        att = np.array(self.log_att)
        control = np.array(self.log_control)
        arm = np.array(self.log_arm) if self.log_arm else None
        
        fig, axes = plt.subplots(4, 3, figsize=(14, 12))
        
        # Base position plots
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            axes[0, i].plot(t, pos[:, i], 'b-', label='Actual', linewidth=1.5)
            axes[0, i].plot(t, pos_ref[:, i], 'r--', label='Reference', linewidth=1)
            axes[0, i].set_ylabel(f'Base {labels[i]} (m)')
            axes[0, i].grid(True, alpha=0.3)
            if i == 0:
                axes[0, i].legend()
                axes[0, i].set_title('Base Position')
        
        # End-effector position plots
        for i in range(3):
            axes[1, i].plot(t, ee[:, i], 'b-', label='Actual', linewidth=1.5)
            axes[1, i].plot(t, ee_ref[:, i], 'r--', label='Reference', linewidth=1)
            axes[1, i].set_ylabel(f'EE {labels[i]} (m)')
            axes[1, i].grid(True, alpha=0.3)
            if i == 0:
                axes[1, i].legend()
                axes[1, i].set_title('End-Effector Position')
        
        # Control plots
        control_labels = ['Thrust (N)', 'Torque X (Nm)', 'Torque Y (Nm)']
        for i in range(3):
            axes[2, i].plot(t, control[:, i], 'g-', linewidth=1)
            axes[2, i].set_ylabel(control_labels[i])
            axes[2, i].grid(True, alpha=0.3)
            if i == 0:
                axes[2, i].axhline(y=HOVER_THRUST, color='k', linestyle='--', alpha=0.5)
                axes[2, i].set_title('Base Control Inputs')
        
        # Arm plots
        if arm is not None and len(arm) > 0:
            axes[3, 0].plot(t, arm[:, 0], 'b-', label='Joint 1 pos', linewidth=1.5)
            axes[3, 0].plot(t, arm[:, 1], 'r-', label='Joint 2 pos', linewidth=1.5)
            axes[3, 0].set_ylabel('Joint Position (rad)')
            axes[3, 0].set_xlabel('Time (s)')
            axes[3, 0].legend()
            axes[3, 0].grid(True, alpha=0.3)
            axes[3, 0].set_title('Arm Joints')
        
        # Arm accelerations
        axes[3, 1].plot(t, control[:, 4], 'b-', label='ddq1', linewidth=1)
        axes[3, 1].plot(t, control[:, 5], 'r-', label='ddq2', linewidth=1)
        axes[3, 1].set_ylabel('Joint Acc (rad/s²)')
        axes[3, 1].set_xlabel('Time (s)')
        axes[3, 1].legend()
        axes[3, 1].grid(True, alpha=0.3)
        axes[3, 1].set_title('Arm Accelerations')
        
        # Solve time
        solve_times = np.array(self.log_solve_time)
        axes[3, 2].plot(t, solve_times, 'm-', linewidth=1)
        axes[3, 2].axhline(y=self.dt * 1000, color='r', linestyle='--', alpha=0.7)
        axes[3, 2].set_ylabel('Solve Time (ms)')
        axes[3, 2].set_xlabel('Time (s)')
        axes[3, 2].grid(True, alpha=0.3)
        axes[3, 2].set_title('MPC Solve Time')
        
        plt.tight_layout()
        plt.savefig('mpc_ef_results.png', dpi=150)
        plt.show()
        self.get_logger().info('Plot saved to mpc_ef_results.png')
    
    def print_stats(self):
        """Print performance statistics"""
        if len(self.solve_times) == 0:
            self.get_logger().info('No solve time data')
            return
        
        times = np.array(self.solve_times)
        errors = np.array(self.ee_errors) if self.ee_errors else np.array([0])
        
        print("\n" + "=" * 60)
        print("MPC Controller (End-Effector) Performance Statistics")
        print("=" * 60)
        print(f"Solve Time (ms):")
        print(f"  Mean:   {np.mean(times):.2f}")
        print(f"  Median: {np.median(times):.2f}")
        print(f"  Min:    {np.min(times):.2f}")
        print(f"  Max:    {np.max(times):.2f}")
        print(f"  Std:    {np.std(times):.2f}")
        print(f"\nEE Tracking Error (m):")
        print(f"  Mean:   {np.mean(errors):.4f}")
        print(f"  Max:    {np.max(errors):.4f}")
        print(f"  RMS:    {np.sqrt(np.mean(errors**2)):.4f}")
        print(f"\nControl period: {self.dt * 1000:.1f} ms")
        print(f"Arm mode: ACTIVE (EE tracking)")
        
        if np.max(times) < self.dt * 1000:
            print("\nStatus: REAL-TIME FEASIBLE")
        else:
            print("\nStatus: NOT real-time feasible")
        print("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    controller = MPCControllerEF()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    def command_loop():
        """Interactive command loop"""
        print("\n" + "=" * 60)
        print("MPC Controller - End-Effector Tracking")
        print("=" * 60)
        print("Commands:")
        print("  start [mode]  - Start EE trajectory (hover/circle/line/vertical/reach)")
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
                    if mode in ['hover', 'circle', 'line', 'vertical', 'reach']:
                        controller.start_trajectory(mode)
                    else:
                        print(f"Unknown mode: {mode}")
                        print("Available: hover, circle, line, vertical, reach")
                    
                elif cmd == 'stop':
                    controller.stop()
                    
                elif cmd == 'plot':
                    controller.plot_data()
                    
                elif cmd == 'stats':
                    controller.print_stats()
                    
                elif cmd == 'status':
                    state = controller.state_estimator.get_full_state()
                    ee_pos = compute_ee_from_state(state)
                    print(f"Control enabled: {controller.control_enabled}")
                    print(f"Mode: {controller.trajectory_mode}")
                    print(f"Base position: [{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}]")
                    print(f"EE position: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}]")
                    print(f"Arm joints: [{state[13]:.3f}, {state[14]:.3f}]")
                    print(f"IMU received: {controller.imu_received}")
                    print(f"Joints received: {controller.joints_received}")
                    
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
