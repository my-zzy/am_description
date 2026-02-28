#!/usr/bin/env python3
"""
MPC Base Controller for Quadrotor

Simplified MPC controller that only controls the quadrotor base (no arm).
Uses acados for real-time optimization.

State vector (12):
    - position (3): x, y, z
    - velocity (3): vx, vy, vz
    - quaternion (4): qx, qy, qz, qw
    - angular velocity (3): wx, wy, wz

Control vector (4):
    - thrust: total thrust force (N)
    - tau_x, tau_y, tau_z: body torques (Nm)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
import numpy as np
import time
import threading
from collections import deque
import matplotlib.pyplot as plt

from am_description.mpc_base.acados_base_solver import AcadosBaseMPCSolver
from am_description.mpc_base.acados_base_model import HOVER_THRUST, MASS, GRAVITY


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


class BaseStateEstimator:
    """
    Simple state estimator for quadrotor base
    """
    
    def __init__(self):
        # State: [pos(3), vel(3), quat(4), omega(3)]
        self.state = np.zeros(12)
        self.state[6:10] = np.array([0, 0, 0, 1])  # Identity quaternion
        
        # Velocity integration
        self.last_time = None
        self.acc_world = np.zeros(3)
        
    def update_imu(self, linear_acc, orientation, angular_vel):
        """
        Update state from IMU data
        
        Args:
            linear_acc: linear acceleration in body frame [3]
            orientation: quaternion [x,y,z,w]
            angular_vel: angular velocity in body frame [3]
        """
        # Update orientation
        self.state[6:10] = orientation / np.linalg.norm(orientation)
        
        # Update angular velocity
        self.state[10:13] = angular_vel
        
        # Convert acceleration to world frame and integrate velocity
        quat = self.state[6:10]
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
            self.state[3:6] += acc_world * dt
            self.state[0:3] += self.state[3:6] * dt
        self.last_time = current_time
    
    def get_state(self):
        """Get current state estimate [12]"""
        return self.state.copy()
    
    def reset(self):
        """Reset state estimator"""
        self.state = np.zeros(12)
        self.state[6:10] = np.array([0, 0, 0, 1])
        self.last_time = None


class MPCBaseController(Node):
    """
    ROS2 node for quadrotor base MPC control
    """
    
    def __init__(self):
        super().__init__('mpc_base_controller')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_rate', 20.0),
                ('trajectory_mode', 'hover'),
                ('trajectory_radius', 1.5),
                ('trajectory_speed', 0.3),
                ('trajectory_height', 2.0),
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
        self.get_logger().info('Initializing acados MPC solver...')
        self.mpc_solver = AcadosBaseMPCSolver(mpc_params)
        
        # State estimator
        self.state_estimator = BaseStateEstimator()
        
        # Trajectory parameters
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_radius = self.get_parameter('trajectory_radius').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        self.trajectory_height = self.get_parameter('trajectory_height').value
        self.trajectory_time = 0.0
        
        # Control state
        self.control_enabled = False
        self.imu_received = False
        
        # Data logging
        self.max_log_size = 5000
        self.log_time = deque(maxlen=self.max_log_size)
        self.log_pos = deque(maxlen=self.max_log_size)
        self.log_pos_ref = deque(maxlen=self.max_log_size)
        self.log_att = deque(maxlen=self.max_log_size)
        self.log_att_ref = deque(maxlen=self.max_log_size)
        self.log_control = deque(maxlen=self.max_log_size)
        self.log_solve_time = deque(maxlen=self.max_log_size)
        self.start_log_time = None
        
        # Performance metrics
        self.solve_times = deque(maxlen=100)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/aerial_manipulator/imu',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.thrust_pub = self.create_publisher(
            Wrench,
            '/aerial_manipulator/thrust',
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('MPC Base Controller initialized!')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Prediction horizon: {mpc_params["N_horizon"]} steps')
        self.get_logger().info('Commands: start [mode], stop, plot, stats, quit')
        self.get_logger().info('Modes: hover, circle, square, figure8, takeoff')
    
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
    
    def generate_reference_trajectory(self, current_time, horizon):
        """
        Generate reference trajectory for MPC
        
        Args:
            current_time: current trajectory time
            horizon: number of steps
        
        Returns:
            x_ref: reference trajectory [horizon+1 x 12]
        """
        x_ref = np.zeros((horizon + 1, 12))
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
        
        return x_ref
    
    def control_loop(self):
        """Main MPC control loop"""
        if not self.control_enabled:
            return
        
        if not self.imu_received:
            self.get_logger().warn('Waiting for IMU data...', throttle_duration_sec=1.0)
            return
        
        if self.start_log_time is None:
            self.start_log_time = time.time()
        
        # Get current state
        x_current = self.state_estimator.get_state()
        
        # Generate reference trajectory
        N = self.mpc_solver.N
        x_ref = self.generate_reference_trajectory(self.trajectory_time, N)
        
        # Solve MPC
        solve_start = time.time()
        try:
            u_opt, x_pred, info = self.mpc_solver.solve(x_current, x_ref)
            solve_time = time.time() - solve_start
            self.solve_times.append(solve_time * 1000)
            
            if not info['success']:
                self.get_logger().warn(f'MPC solve failed: status={info["status"]}')
            
            # Apply first control
            thrust = u_opt[0, 0]
            tau_x = u_opt[0, 1]
            tau_y = u_opt[0, 2]
            tau_z = u_opt[0, 3]
            
            # Publish wrench
            msg = Wrench()
            msg.force.z = thrust
            msg.torque.x = tau_x
            msg.torque.y = tau_y
            msg.torque.z = tau_z
            self.thrust_pub.publish(msg)
            
            # Log data
            log_t = time.time() - self.start_log_time
            self.log_time.append(log_t)
            self.log_pos.append(x_current[0:3].copy())
            self.log_pos_ref.append(x_ref[0, 0:3].copy())
            self.log_att.append(quaternion_to_euler(x_current[6:10]))
            self.log_att_ref.append(quaternion_to_euler(x_ref[0, 6:10]))
            self.log_control.append([thrust, tau_x, tau_y, tau_z])
            self.log_solve_time.append(solve_time * 1000)
            
        except Exception as e:
            self.get_logger().error(f'MPC error: {e}')
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
        self.get_logger().info(f'Started MPC trajectory: {mode}')
    
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
        att = np.array(self.log_att)
        att_ref = np.array(self.log_att_ref)
        control = np.array(self.log_control)
        
        fig, axes = plt.subplots(3, 3, figsize=(14, 10))
        
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
            axes[2, i].set_xlabel('Time (s)')
            axes[2, i].grid(True, alpha=0.3)
            if i == 0:
                axes[2, i].axhline(y=HOVER_THRUST, color='k', linestyle='--', alpha=0.5, label=f'Hover: {HOVER_THRUST:.1f}N')
                axes[2, i].legend()
                axes[2, i].set_title('Control Inputs')
        
        plt.tight_layout()
        plt.savefig('mpc_base_results.png', dpi=150)
        plt.show()
        self.get_logger().info('Plot saved to mpc_base_results.png')
    
    def print_stats(self):
        """Print performance statistics"""
        if len(self.solve_times) == 0:
            self.get_logger().info('No solve time data')
            return
        
        times = np.array(self.solve_times)
        print("\n" + "=" * 50)
        print("MPC Solver Performance")
        print("=" * 50)
        print(f"Solve Time (ms):")
        print(f"  Mean:   {np.mean(times):.2f}")
        print(f"  Median: {np.median(times):.2f}")
        print(f"  Min:    {np.min(times):.2f}")
        print(f"  Max:    {np.max(times):.2f}")
        print(f"  Std:    {np.std(times):.2f}")
        print(f"\nControl period: {self.dt * 1000:.1f} ms")
        
        if np.max(times) < self.dt * 1000:
            print("Status: REAL-TIME FEASIBLE")
        else:
            print("Status: NOT real-time feasible")
        print("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    controller = MPCBaseController()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    def command_loop():
        """Interactive command loop"""
        print("\n" + "=" * 50)
        print("MPC Base Controller - Command Interface")
        print("=" * 50)
        print("Commands:")
        print("  start [mode]  - Start trajectory (hover/circle/square/figure8/takeoff)")
        print("  stop          - Stop controller")
        print("  plot          - Plot logged data")
        print("  stats         - Show solver statistics")
        print("  quit          - Exit")
        print("=" * 50 + "\n")
        
        while rclpy.ok():
            try:
                cmd = input(">>> ").strip().lower()
                
                if cmd.startswith('start'):
                    parts = cmd.split()
                    mode = parts[1] if len(parts) > 1 else 'hover'
                    controller.start_trajectory(mode)
                    
                elif cmd == 'stop':
                    controller.stop()
                    
                elif cmd == 'plot':
                    controller.plot_data()
                    
                elif cmd == 'stats':
                    controller.print_stats()
                    
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
