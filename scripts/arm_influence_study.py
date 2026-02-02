#!/usr/bin/env python3
"""
Arm Influence Study - Studies the effect of arm movement on quadrotor stability
1. Hovers the drone at a stable position
2. Moves the robot arm through predefined motions
3. Logs and plots the drone position and end effector position
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading


class ArmInfluenceStudy(Node):
    def __init__(self):
        super().__init__('arm_influence_study')
        
        # Drone parameters
        self.mass = 2.1  # kg
        self.hover_thrust = self.mass * 9.81  # N
        
        # Arm geometry (from URDF)
        self.arm_base_offset = np.array([0.0, 0.0, -0.05])  # arm_mount offset from base_link
        self.arm_link_1_length = 0.1  # length of arm_link_1
        self.arm_link_2_length = 0.1  # length of arm_link_2
        
        # Current state (estimated from IMU)
        self.position = np.array([0.0, 0.0, 1.0])  # Start at spawn height
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x, y, z, w]
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])
        
        # Joint states
        self.joint_positions = np.array([0.0, 0.0])  # [joint1, joint2]
        self.joint_velocities = np.array([0.0, 0.0])
        
        # End effector position in world frame
        self.end_effector_pos = np.array([0.0, 0.0, 0.0])
        
        # IMU integration
        self.last_imu_time = None
        self.gravity = np.array([0.0, 0.0, 9.81])
        
        # Target state for hover
        self.target_position = np.array([0.0, 0.0, 2.0])
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        
        # PID gains for position control
        self.kp_pos = np.array([2.0, 2.0, 4.0])  # x, y, z
        self.kd_pos = np.array([1.5, 1.5, 2.5])
        self.ki_pos = np.array([0.1, 0.1, 0.2])
        
        # PID gains for attitude control
        self.kp_att = np.array([1.0, 1.0, 0.5])  # roll, pitch, yaw
        self.kd_att = np.array([0.3, 0.3, 0.2])
        
        # Integral error accumulation
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.max_integral = 5.0
        
        # Control state
        self.control_enabled = False
        self.experiment_phase = 'idle'  # 'idle', 'stabilizing', 'arm_motion', 'done'
        self.experiment_time = 0.0
        self.stabilize_duration = 5.0  # seconds to stabilize before arm motion
        self.arm_motion_duration = 20.0  # duration of arm motion experiment
        
        # Arm motion parameters
        self.arm_motion_type = 'sinusoid'  # 'sinusoid', 'step', 'sweep'
        self.arm_frequency = 0.5  # Hz for sinusoidal motion
        self.arm_amplitude = 1.0  # radians
        
        # Data logging
        self.max_log_size = 10000
        self.log_time = deque(maxlen=self.max_log_size)
        
        # Drone position
        self.log_drone_x = deque(maxlen=self.max_log_size)
        self.log_drone_y = deque(maxlen=self.max_log_size)
        self.log_drone_z = deque(maxlen=self.max_log_size)
        
        # Drone attitude
        self.log_roll = deque(maxlen=self.max_log_size)
        self.log_pitch = deque(maxlen=self.max_log_size)
        self.log_yaw = deque(maxlen=self.max_log_size)
        
        # End effector position (world frame)
        self.log_ee_x = deque(maxlen=self.max_log_size)
        self.log_ee_y = deque(maxlen=self.max_log_size)
        self.log_ee_z = deque(maxlen=self.max_log_size)
        
        # Joint positions
        self.log_joint1 = deque(maxlen=self.max_log_size)
        self.log_joint2 = deque(maxlen=self.max_log_size)
        
        # Command positions
        self.log_joint1_cmd = deque(maxlen=self.max_log_size)
        self.log_joint2_cmd = deque(maxlen=self.max_log_size)
        
        self.start_log_time = None
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/aerial_manipulator/imu',
            self.imu_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.thrust_pub = self.create_publisher(
            Wrench,
            '/aerial_manipulator/thrust',
            10
        )
        
        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )
        
        # Control timer (50 Hz)
        self.dt = 0.02
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Current arm command
        self.arm_cmd = [0.0, 0.0]
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Arm Influence Study Controller')
        self.get_logger().info('=' * 50)
        self.get_logger().info('This script studies how arm motion affects drone stability')
        self.get_logger().info('Waiting for IMU and joint state data...')
        
    def imu_callback(self, msg: Imu):
        """Update state from IMU and integrate to estimate position/velocity"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get orientation directly from IMU
        self.orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Get angular velocity
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Get linear acceleration in body frame
        acc_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Transform acceleration to world frame
        acc_world = self.rotate_vector_by_quaternion(acc_body, self.orientation)
        
        # Remove gravity to get linear acceleration
        self.linear_acceleration = acc_world - self.gravity
        
        # Integrate acceleration to get velocity and position
        if self.last_imu_time is not None:
            dt = current_time - self.last_imu_time
            
            if dt > 0 and dt < 0.1:
                # Integrate acceleration to velocity
                self.velocity += self.linear_acceleration * dt
                
                # Apply velocity damping to reduce drift
                drift_correction = 0.01
                self.velocity *= (1.0 - drift_correction * dt)
                
                # Integrate velocity to position
                self.position += self.velocity * dt
                
                # Ground constraint
                if self.position[2] < 0:
                    self.position[2] = 0
                    self.velocity[2] = max(0, self.velocity[2])
        
        self.last_imu_time = current_time
        
        # Update end effector position
        self.compute_end_effector_position()
        
    def joint_state_callback(self, msg: JointState):
        """Update joint states"""
        for i, name in enumerate(msg.name):
            if name == 'arm_joint_1':
                self.joint_positions[0] = msg.position[i]
                if len(msg.velocity) > i:
                    self.joint_velocities[0] = msg.velocity[i]
            elif name == 'arm_joint_2':
                self.joint_positions[1] = msg.position[i]
                if len(msg.velocity) > i:
                    self.joint_velocities[1] = msg.velocity[i]
                    
    def rotate_vector_by_quaternion(self, v, q):
        """Rotate vector v by quaternion q"""
        qx, qy, qz, qw = q
        
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        
        return R @ v
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to roll, pitch, yaw"""
        x, y, z, w = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def compute_end_effector_position(self):
        """Compute end effector position in world frame using forward kinematics"""
        j1 = self.joint_positions[0]  # rotation about Y axis
        j2 = self.joint_positions[1]  # rotation about Z axis
        
        # Forward kinematics in arm base frame
        # arm_joint_1 rotates about Y, arm_joint_2 rotates about Z
        # arm_link_1 extends along Z (length 0.1m with center at 0.05)
        # arm_link_2 extends along Z (length 0.1m with center at 0.05)
        
        # Position of end of link 1 in arm base frame
        # Link 1 rotates about Y axis, extends in Z direction
        L1 = self.arm_link_1_length
        L2 = self.arm_link_2_length
        
        # End of link 1 position (in arm base frame)
        # Rotation about Y by j1, then translate along the rotated Z axis
        p1_arm = np.array([
            -L1 * math.sin(j1),  # X component
            0.0,                  # Y component
            -L1 * math.cos(j1)   # Z component (negative because arm points down)
        ])
        
        # End of link 2 position (relative to end of link 1)
        # After j1 rotation, we rotate about Z by j2, then extend along rotated Z
        # The Z axis after j1 rotation is: [-sin(j1), 0, -cos(j1)]
        # Rotation about this axis by j2... but j2 rotates about the local Z of link1
        
        # Simplified: after j1, the local frame has:
        # - X axis rotated by j1 about Y
        # - Z axis pointing in direction [-sin(j1), 0, -cos(j1)]
        # j2 rotates about this local Z axis
        
        # For simplicity, compute rotation matrices
        # R1: rotation about Y by j1
        R1 = np.array([
            [math.cos(j1), 0, math.sin(j1)],
            [0, 1, 0],
            [-math.sin(j1), 0, math.cos(j1)]
        ])
        
        # R2: rotation about Z by j2 (in the rotated frame after R1)
        R2 = np.array([
            [math.cos(j2), -math.sin(j2), 0],
            [math.sin(j2), math.cos(j2), 0],
            [0, 0, 1]
        ])
        
        # Link 1 end in arm base frame (extends along -Z after rotation)
        link1_end = R1 @ np.array([0, 0, -L1])
        
        # Link 2 end relative to link 1 end (extends along -Z in link1 frame, then rotated by j2)
        link2_local = R2 @ np.array([0, 0, -L2])
        link2_world = R1 @ link2_local
        
        # End effector in arm base frame
        ee_arm_base = link1_end + link2_world
        
        # Transform to body frame (add arm base offset)
        ee_body = ee_arm_base + self.arm_base_offset
        
        # Transform to world frame
        self.end_effector_pos = self.position + self.rotate_vector_by_quaternion(ee_body, self.orientation)
        
    def generate_arm_command(self):
        """Generate arm motion command based on motion type"""
        if self.experiment_phase != 'arm_motion':
            return [0.0, 0.0]
        
        t = self.experiment_time - self.stabilize_duration
        
        if self.arm_motion_type == 'sinusoid':
            # Sinusoidal motion on both joints
            j1 = self.arm_amplitude * math.sin(2 * math.pi * self.arm_frequency * t)
            j2 = self.arm_amplitude * 0.5 * math.sin(2 * math.pi * self.arm_frequency * 0.7 * t)
            return [j1, j2]
            
        elif self.arm_motion_type == 'step':
            # Step changes every 2 seconds
            step_period = 2.0
            step_phase = int(t / step_period) % 4
            positions = [
                [0.5, 0.0],
                [0.0, 0.5],
                [-0.5, 0.0],
                [0.0, -0.5]
            ]
            return positions[step_phase]
            
        elif self.arm_motion_type == 'sweep':
            # Slow sweep through joint space
            j1 = self.arm_amplitude * math.sin(2 * math.pi * 0.1 * t)
            j2 = self.arm_amplitude * math.cos(2 * math.pi * 0.1 * t)
            return [j1, j2]
            
        return [0.0, 0.0]
    
    def control_loop(self):
        """Main control loop - PID position control for hover + arm motion"""
        if not self.control_enabled:
            # Send zero thrust when disabled
            msg = Wrench()
            msg.force.z = 0.0
            self.thrust_pub.publish(msg)
            return
        
        # Update experiment time
        self.experiment_time += self.dt
        
        # Initialize start time for logging
        if self.start_log_time is None:
            self.start_log_time = self.get_clock().now().nanoseconds / 1e9
        
        # State machine for experiment phases
        if self.experiment_phase == 'stabilizing':
            if self.experiment_time >= self.stabilize_duration:
                self.experiment_phase = 'arm_motion'
                self.get_logger().info('Stabilization complete. Starting arm motion...')
                
        elif self.experiment_phase == 'arm_motion':
            if self.experiment_time >= self.stabilize_duration + self.arm_motion_duration:
                self.experiment_phase = 'done'
                self.get_logger().info('Arm motion experiment complete!')
                self.get_logger().info('Type "plot" to see results or "stop" to end.')
        
        # Generate and publish arm commands
        self.arm_cmd = self.generate_arm_command()
        arm_msg = Float64MultiArray()
        arm_msg.data = self.arm_cmd
        self.arm_pub.publish(arm_msg)
        
        # Position error
        pos_error = self.target_position - self.position
        vel_error = self.target_velocity - self.velocity
        
        # Update integral
        self.integral_error += pos_error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        
        # PID control output
        acc_desired = (self.kp_pos * pos_error + 
                      self.kd_pos * vel_error + 
                      self.ki_pos * self.integral_error)
        
        # Total desired acceleration including gravity compensation
        acc_total = acc_desired + np.array([0.0, 0.0, 9.81])
        
        # Compute thrust magnitude
        thrust_magnitude = self.mass * np.linalg.norm(acc_total)
        
        # Compute desired attitude
        acc_norm = np.linalg.norm(acc_total)
        if acc_norm > 0.1:
            thrust_dir = acc_total / acc_norm
            desired_pitch = math.asin(thrust_dir[0])
            desired_roll = math.asin(-thrust_dir[1] / math.cos(desired_pitch)) if abs(math.cos(desired_pitch)) > 0.1 else 0.0
        else:
            desired_roll = 0.0
            desired_pitch = 0.0
        
        desired_yaw = 0.0
        
        # Clamp desired angles
        max_tilt = 0.5
        desired_roll = np.clip(desired_roll, -max_tilt, max_tilt)
        desired_pitch = np.clip(desired_pitch, -max_tilt, max_tilt)
        
        # Get current attitude
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)
        
        # Attitude error
        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch
        yaw_error = desired_yaw - yaw
        
        # Wrap yaw error
        if yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Compute torques
        torque_x = self.kp_att[0] * roll_error - self.kd_att[0] * self.angular_velocity[0]
        torque_y = self.kp_att[1] * pitch_error - self.kd_att[1] * self.angular_velocity[1]
        torque_z = self.kp_att[2] * yaw_error - self.kd_att[2] * self.angular_velocity[2]
        
        # Clamp thrust
        thrust_z = np.clip(thrust_magnitude, 0.0, 40.0)
        
        # Log data
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_log_time
        self.log_time.append(current_time)
        
        # Drone position
        self.log_drone_x.append(self.position[0])
        self.log_drone_y.append(self.position[1])
        self.log_drone_z.append(self.position[2])
        
        # Drone attitude
        self.log_roll.append(roll)
        self.log_pitch.append(pitch)
        self.log_yaw.append(yaw)
        
        # End effector position
        self.log_ee_x.append(self.end_effector_pos[0])
        self.log_ee_y.append(self.end_effector_pos[1])
        self.log_ee_z.append(self.end_effector_pos[2])
        
        # Joint positions
        self.log_joint1.append(self.joint_positions[0])
        self.log_joint2.append(self.joint_positions[1])
        
        # Joint commands
        self.log_joint1_cmd.append(self.arm_cmd[0])
        self.log_joint2_cmd.append(self.arm_cmd[1])
        
        # Publish thrust command
        msg = Wrench()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = thrust_z
        msg.torque.x = torque_x
        msg.torque.y = torque_y
        msg.torque.z = torque_z
        self.thrust_pub.publish(msg)
        
        # Log status periodically
        if int(self.experiment_time * 10) % 50 == 0:  # Every 5 seconds
            self.get_logger().info(
                f'Phase: {self.experiment_phase} | Time: {self.experiment_time:.1f}s | '
                f'Drone: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}] | '
                f'EE: [{self.end_effector_pos[0]:.2f}, {self.end_effector_pos[1]:.2f}, {self.end_effector_pos[2]:.2f}] | '
                f'Joints: [{self.joint_positions[0]:.2f}, {self.joint_positions[1]:.2f}]'
            )
    
    def start_experiment(self, motion_type='sinusoid'):
        """Start the arm influence experiment"""
        self.arm_motion_type = motion_type
        self.experiment_time = 0.0
        self.experiment_phase = 'stabilizing'
        self.control_enabled = True
        self.integral_error = np.array([0.0, 0.0, 0.0])
        
        # Clear logs
        self.log_time.clear()
        self.log_drone_x.clear()
        self.log_drone_y.clear()
        self.log_drone_z.clear()
        self.log_roll.clear()
        self.log_pitch.clear()
        self.log_yaw.clear()
        self.log_ee_x.clear()
        self.log_ee_y.clear()
        self.log_ee_z.clear()
        self.log_joint1.clear()
        self.log_joint2.clear()
        self.log_joint1_cmd.clear()
        self.log_joint2_cmd.clear()
        self.start_log_time = None
        
        self.get_logger().info(f'Starting experiment with motion type: {motion_type}')
        self.get_logger().info(f'Stabilizing for {self.stabilize_duration}s, then arm motion for {self.arm_motion_duration}s')
    
    def stop(self):
        """Stop the experiment"""
        self.control_enabled = False
        self.experiment_phase = 'idle'
        
        # Return arm to home position
        arm_msg = Float64MultiArray()
        arm_msg.data = [0.0, 0.0]
        self.arm_pub.publish(arm_msg)
        
        self.get_logger().info('Experiment stopped')
    
    def plot_data(self):
        """Plot comprehensive data showing arm influence on drone"""
        if len(self.log_time) == 0:
            self.get_logger().warn('No data to plot! Run an experiment first.')
            return
        
        # Convert to numpy arrays
        time = np.array(self.log_time)
        drone_x = np.array(self.log_drone_x)
        drone_y = np.array(self.log_drone_y)
        drone_z = np.array(self.log_drone_z)
        roll = np.rad2deg(np.array(self.log_roll))
        pitch = np.rad2deg(np.array(self.log_pitch))
        yaw = np.rad2deg(np.array(self.log_yaw))
        ee_x = np.array(self.log_ee_x)
        ee_y = np.array(self.log_ee_y)
        ee_z = np.array(self.log_ee_z)
        joint1 = np.rad2deg(np.array(self.log_joint1))
        joint2 = np.rad2deg(np.array(self.log_joint2))
        joint1_cmd = np.rad2deg(np.array(self.log_joint1_cmd))
        joint2_cmd = np.rad2deg(np.array(self.log_joint2_cmd))
        
        # Find where arm motion starts
        motion_start_idx = int(self.stabilize_duration / self.dt)
        
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('Arm Influence on Quadrotor Stability Study', fontsize=16, fontweight='bold')
        
        # Create grid spec for custom layout
        gs = fig.add_gridspec(4, 3, hspace=0.35, wspace=0.3)
        
        # --- Row 1: Drone Position ---
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(time, drone_x, 'b-', linewidth=1.5, label='Actual')
        ax1.axhline(y=0, color='r', linestyle='--', alpha=0.7, label='Target')
        ax1.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7, label='Motion Start')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('X Position (m)')
        ax1.set_title('Drone X Position')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.grid(True, alpha=0.3)
        
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(time, drone_y, 'b-', linewidth=1.5, label='Actual')
        ax2.axhline(y=0, color='r', linestyle='--', alpha=0.7, label='Target')
        ax2.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Drone Y Position')
        ax2.legend(loc='upper right', fontsize=8)
        ax2.grid(True, alpha=0.3)
        
        ax3 = fig.add_subplot(gs[0, 2])
        ax3.plot(time, drone_z, 'b-', linewidth=1.5, label='Actual')
        ax3.axhline(y=2.0, color='r', linestyle='--', alpha=0.7, label='Target')
        ax3.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Z Position (m)')
        ax3.set_title('Drone Z Position (Altitude)')
        ax3.legend(loc='upper right', fontsize=8)
        ax3.grid(True, alpha=0.3)
        
        # --- Row 2: Drone Attitude ---
        ax4 = fig.add_subplot(gs[1, 0])
        ax4.plot(time, roll, 'b-', linewidth=1.5)
        ax4.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Roll (deg)')
        ax4.set_title('Drone Roll')
        ax4.grid(True, alpha=0.3)
        
        ax5 = fig.add_subplot(gs[1, 1])
        ax5.plot(time, pitch, 'b-', linewidth=1.5)
        ax5.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Pitch (deg)')
        ax5.set_title('Drone Pitch')
        ax5.grid(True, alpha=0.3)
        
        ax6 = fig.add_subplot(gs[1, 2])
        ax6.plot(time, yaw, 'b-', linewidth=1.5)
        ax6.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Yaw (deg)')
        ax6.set_title('Drone Yaw')
        ax6.grid(True, alpha=0.3)
        
        # --- Row 3: End Effector Position ---
        ax7 = fig.add_subplot(gs[2, 0])
        ax7.plot(time, ee_x, 'g-', linewidth=1.5, label='End Effector')
        ax7.plot(time, drone_x, 'b--', linewidth=1, alpha=0.7, label='Drone')
        ax7.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax7.set_xlabel('Time (s)')
        ax7.set_ylabel('X Position (m)')
        ax7.set_title('End Effector X Position')
        ax7.legend(loc='upper right', fontsize=8)
        ax7.grid(True, alpha=0.3)
        
        ax8 = fig.add_subplot(gs[2, 1])
        ax8.plot(time, ee_y, 'g-', linewidth=1.5, label='End Effector')
        ax8.plot(time, drone_y, 'b--', linewidth=1, alpha=0.7, label='Drone')
        ax8.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('Y Position (m)')
        ax8.set_title('End Effector Y Position')
        ax8.legend(loc='upper right', fontsize=8)
        ax8.grid(True, alpha=0.3)
        
        ax9 = fig.add_subplot(gs[2, 2])
        ax9.plot(time, ee_z, 'g-', linewidth=1.5, label='End Effector')
        ax9.plot(time, drone_z, 'b--', linewidth=1, alpha=0.7, label='Drone')
        ax9.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('Z Position (m)')
        ax9.set_title('End Effector Z Position')
        ax9.legend(loc='upper right', fontsize=8)
        ax9.grid(True, alpha=0.3)
        
        # --- Row 4: Joint Positions ---
        ax10 = fig.add_subplot(gs[3, 0])
        ax10.plot(time, joint1, 'b-', linewidth=1.5, label='Actual')
        ax10.plot(time, joint1_cmd, 'r--', linewidth=1, alpha=0.7, label='Command')
        ax10.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax10.set_xlabel('Time (s)')
        ax10.set_ylabel('Joint 1 (deg)')
        ax10.set_title('Arm Joint 1 (Pitch)')
        ax10.legend(loc='upper right', fontsize=8)
        ax10.grid(True, alpha=0.3)
        
        ax11 = fig.add_subplot(gs[3, 1])
        ax11.plot(time, joint2, 'b-', linewidth=1.5, label='Actual')
        ax11.plot(time, joint2_cmd, 'r--', linewidth=1, alpha=0.7, label='Command')
        ax11.axvline(x=self.stabilize_duration, color='g', linestyle=':', alpha=0.7)
        ax11.set_xlabel('Time (s)')
        ax11.set_ylabel('Joint 2 (deg)')
        ax11.set_title('Arm Joint 2 (Yaw)')
        ax11.legend(loc='upper right', fontsize=8)
        ax11.grid(True, alpha=0.3)
        
        # 3D trajectory plot
        ax12 = fig.add_subplot(gs[3, 2], projection='3d')
        ax12.plot3D(drone_x, drone_y, drone_z, 'b-', linewidth=1, label='Drone', alpha=0.7)
        ax12.plot3D(ee_x, ee_y, ee_z, 'g-', linewidth=1, label='End Effector', alpha=0.7)
        ax12.scatter([0], [0], [2], color='r', s=100, marker='*', label='Target')
        ax12.set_xlabel('X (m)')
        ax12.set_ylabel('Y (m)')
        ax12.set_zlabel('Z (m)')
        ax12.set_title('3D Trajectory')
        ax12.legend(loc='upper right', fontsize=8)
        
        plt.tight_layout()
        plt.savefig('src/am_description/fig/arm_influence_study_results.png', dpi=300)
        plt.show()
        
        # Print statistics
        self.print_statistics()
        
        
    def print_statistics(self):
        """Print statistical summary of the experiment"""
        if len(self.log_time) == 0:
            return
            
        motion_start_idx = int(self.stabilize_duration / self.dt)
        
        # Get data after motion starts
        drone_x = np.array(self.log_drone_x)[motion_start_idx:]
        drone_y = np.array(self.log_drone_y)[motion_start_idx:]
        drone_z = np.array(self.log_drone_z)[motion_start_idx:]
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('EXPERIMENT STATISTICS (During Arm Motion)')
        self.get_logger().info('=' * 60)
        
        # Position deviations from target
        x_dev = drone_x - 0.0  # target x
        y_dev = drone_y - 0.0  # target y
        z_dev = drone_z - 2.0  # target z
        
        self.get_logger().info(f'X Position - Mean Dev: {np.mean(x_dev):.4f}m, Std: {np.std(x_dev):.4f}m, Max: {np.max(np.abs(x_dev)):.4f}m')
        self.get_logger().info(f'Y Position - Mean Dev: {np.mean(y_dev):.4f}m, Std: {np.std(y_dev):.4f}m, Max: {np.max(np.abs(y_dev)):.4f}m')
        self.get_logger().info(f'Z Position - Mean Dev: {np.mean(z_dev):.4f}m, Std: {np.std(z_dev):.4f}m, Max: {np.max(np.abs(z_dev)):.4f}m')
        
        # Total 3D position error
        pos_error = np.sqrt(x_dev**2 + y_dev**2 + z_dev**2)
        self.get_logger().info(f'3D Position Error - Mean: {np.mean(pos_error):.4f}m, Max: {np.max(pos_error):.4f}m')
        
        # Attitude statistics
        roll = np.rad2deg(np.array(self.log_roll)[motion_start_idx:])
        pitch = np.rad2deg(np.array(self.log_pitch)[motion_start_idx:])
        yaw = np.rad2deg(np.array(self.log_yaw)[motion_start_idx:])
        
        self.get_logger().info(f'Roll  - Mean: {np.mean(roll):.2f}°, Std: {np.std(roll):.2f}°, Max: {np.max(np.abs(roll)):.2f}°')
        self.get_logger().info(f'Pitch - Mean: {np.mean(pitch):.2f}°, Std: {np.std(pitch):.2f}°, Max: {np.max(np.abs(pitch)):.2f}°')
        self.get_logger().info(f'Yaw   - Mean: {np.mean(yaw):.2f}°, Std: {np.std(yaw):.2f}°, Max: {np.max(np.abs(yaw)):.2f}°')
        
        self.get_logger().info('=' * 60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    controller = ArmInfluenceStudy()
    
    def command_loop():
        controller.get_logger().info('\n' + '=' * 50)
        controller.get_logger().info('ARM INFLUENCE STUDY - Commands')
        controller.get_logger().info('=' * 50)
        controller.get_logger().info('  start [motion_type] - Start experiment')
        controller.get_logger().info('      motion_type: sinusoid (default), step, sweep')
        controller.get_logger().info('  stop     - Stop experiment')
        controller.get_logger().info('  status   - Show current status')
        controller.get_logger().info('  plot     - Plot collected data')
        controller.get_logger().info('  params   - Show/set parameters')
        controller.get_logger().info('  quit     - Exit')
        controller.get_logger().info('=' * 50 + '\n')
        
        while rclpy.ok():
            try:
                cmd = input(">> ").strip().lower().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'start':
                    motion_type = cmd[1] if len(cmd) > 1 else 'sinusoid'
                    if motion_type in ['sinusoid', 'step', 'sweep']:
                        controller.start_experiment(motion_type)
                    else:
                        print(f"Unknown motion type: {motion_type}. Use: sinusoid, step, sweep")
                        
                elif cmd[0] == 'stop':
                    controller.stop()
                    
                elif cmd[0] == 'status':
                    print(f"Control enabled: {controller.control_enabled}")
                    print(f"Experiment phase: {controller.experiment_phase}")
                    print(f"Motion type: {controller.arm_motion_type}")
                    print(f"Experiment time: {controller.experiment_time:.1f}s")
                    print(f"Drone position: {controller.position}")
                    print(f"End effector: {controller.end_effector_pos}")
                    print(f"Joint positions: {controller.joint_positions}")
                    print(f"Logged samples: {len(controller.log_time)}")
                    
                elif cmd[0] == 'plot':
                    controller.plot_data()
                    
                elif cmd[0] == 'params':
                    if len(cmd) == 1:
                        print(f"Current parameters:")
                        print(f"  stabilize_duration: {controller.stabilize_duration}s")
                        print(f"  arm_motion_duration: {controller.arm_motion_duration}s")
                        print(f"  arm_frequency: {controller.arm_frequency} Hz")
                        print(f"  arm_amplitude: {controller.arm_amplitude} rad")
                    elif len(cmd) >= 3:
                        param = cmd[1]
                        value = float(cmd[2])
                        if param == 'stabilize':
                            controller.stabilize_duration = value
                        elif param == 'duration':
                            controller.arm_motion_duration = value
                        elif param == 'freq':
                            controller.arm_frequency = value
                        elif param == 'amp':
                            controller.arm_amplitude = value
                        print(f"Set {param} = {value}")
                    
                elif cmd[0] in ['quit', 'exit']:
                    controller.stop()
                    rclpy.shutdown()
                    break
                    
                else:
                    print("Unknown command. Try: start, stop, status, plot, params, quit")
                    
            except Exception as e:
                print(f"Error: {e}")
    
    cmd_thread = threading.Thread(target=command_loop, daemon=True)
    cmd_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
