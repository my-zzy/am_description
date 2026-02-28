#!/usr/bin/env python3
"""
Acados MPC Controller for Aerial Manipulator

High-performance MPC controller using acados for real-time trajectory tracking.
Provides 10-100x speedup compared to scipy-based controller.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time
from collections import deque
import matplotlib.pyplot as plt

from am_description.mpc import StateEstimator, euler_to_quaternion
from am_description.mpc.acados_mpc_solver import AcadosMPCSolver
from am_description.mpc.utils import quaternion_to_euler


class AcadosMPCController(Node):
    def __init__(self):
        super().__init__('acados_mpc_controller')
        
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_rate', 20.0),  # Hz
                ('trajectory_mode', 'circle'),
                ('trajectory_radius', 1.5),
                ('trajectory_speed', 0.3),
                ('trajectory_height', 2.0),
            ]
        )
        
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        
        # MPC parameters (optimized for acados)
        mpc_params = {
            'N_horizon': 10,
            'dt': 0.05,
            
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
            'Q_terminal_factor': 2.0,
            
            # Acados solver options
            'qp_solver': 'PARTIAL_CONDENSING_HPIPM',
            'hessian_approx': 'GAUSS_NEWTON',
            'integrator_type': 'ERK',
            'nlp_solver_type': 'SQP_RTI',  # Real-time iteration for speed
            'qp_solver_iter_max': 50,
            'nlp_solver_tol_stat': 1e-3,
            'nlp_solver_tol_eq': 1e-3,
            'nlp_solver_tol_ineq': 1e-3,
            'nlp_solver_tol_comp': 1e-3,
        }
        
        # Initialize acados MPC solver
        self.get_logger().info('Initializing acados MPC solver (this may take a few seconds)...')
        self.mpc_solver = AcadosMPCSolver(mpc_params)
        
        # Initialize state estimator
        self.state_estimator = StateEstimator()
        
        # Trajectory parameters
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_radius = self.get_parameter('trajectory_radius').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        self.trajectory_height = self.get_parameter('trajectory_height').value
        self.trajectory_time = 0.0
        
        # Control state
        self.control_enabled = False
        self.imu_received = False
        self.joints_received = False
        
        # Data logging
        self.max_log_size = 5000
        self.log_time = deque(maxlen=self.max_log_size)
        self.log_pos = deque(maxlen=self.max_log_size)
        self.log_pos_ref = deque(maxlen=self.max_log_size)
        self.log_att = deque(maxlen=self.max_log_size)  # Euler angles [roll, pitch, yaw]
        self.log_att_ref = deque(maxlen=self.max_log_size)
        self.log_control = deque(maxlen=self.max_log_size)
        self.log_solve_time = deque(maxlen=self.max_log_size)
        self.log_cost = deque(maxlen=self.max_log_size)
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
        
        self.get_logger().info('Acados MPC Controller initialized!')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Prediction horizon: {mpc_params["N_horizon"]} steps')
        self.get_logger().info(f'Solver type: {mpc_params["nlp_solver_type"]}')
        self.get_logger().info('Waiting for sensor data...')
    
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
            self.state_estimator.update_joints(joint_positions)
            self.joints_received = True
    
    def generate_reference_trajectory(self, current_time, horizon):
        """
        Generate reference trajectory for MPC horizon
        
        Args:
            current_time: current trajectory time
            horizon: number of steps to generate
        
        Returns:
            x_ref: reference state trajectory [horizon+1 x 15]
        """
        x_ref = np.zeros((horizon + 1, 15))
        
        for k in range(horizon + 1):
            t = current_time + k * self.mpc_solver.params['dt']
            g = 9.81
            
            if self.trajectory_mode == 'hover':
                pos = np.array([0.0, 0.0, self.trajectory_height])
                vel = np.array([0.0, 0.0, 0.0])
                acc = np.array([0.0, 0.0, 0.0])
            
            elif self.trajectory_mode == 'takeoff':
                # Smooth takeoff trajectory from ground to target height
                takeoff_duration = 5.0  # seconds
                
                if t < takeoff_duration:
                    # Smooth polynomial profile (5th order for zero vel/acc at endpoints)
                    s = t / takeoff_duration
                    height_profile = s**3 * (10 - 15*s + 6*s**2)  # 0 to 1
                    vel_profile = (30 * s**2 * (1 - 2*s + s**2)) / takeoff_duration
                    acc_profile = (60 * s * (1 - 3*s + 2*s**2)) / (takeoff_duration**2)
                    
                    pos = np.array([0.0, 0.0, self.trajectory_height * height_profile])
                    vel = np.array([0.0, 0.0, self.trajectory_height * vel_profile])
                    acc = np.array([0.0, 0.0, self.trajectory_height * acc_profile])
                else:
                    # Hold at target height
                    pos = np.array([0.0, 0.0, self.trajectory_height])
                    vel = np.array([0.0, 0.0, 0.0])
                    acc = np.array([0.0, 0.0, 0.0])
                
            elif self.trajectory_mode == 'circle':
                ang = t * self.trajectory_speed
                r = self.trajectory_radius
                pos = np.array([
                    r * math.cos(ang),
                    r * math.sin(ang),
                    self.trajectory_height
                ])
                vel = np.array([
                    -r * self.trajectory_speed * math.sin(ang),
                    r * self.trajectory_speed * math.cos(ang),
                    0.0
                ])
                acc = np.array([
                    -r * self.trajectory_speed**2 * math.cos(ang),
                    -r * self.trajectory_speed**2 * math.sin(ang),
                    0.0
                ])
                
            elif self.trajectory_mode == 'square':
                ang = t * self.trajectory_speed
                side_length = 2.0
                period = 4.0
                phase = (ang % (2 * math.pi)) / (2 * math.pi) * period
                
                if phase < 1.0:
                    pos = np.array([side_length * phase, 0.0, self.trajectory_height])
                    vel = np.array([side_length * self.trajectory_speed, 0.0, 0.0])
                elif phase < 2.0:
                    pos = np.array([side_length, side_length * (phase - 1.0), self.trajectory_height])
                    vel = np.array([0.0, side_length * self.trajectory_speed, 0.0])
                elif phase < 3.0:
                    pos = np.array([side_length * (3.0 - phase), side_length, self.trajectory_height])
                    vel = np.array([-side_length * self.trajectory_speed, 0.0, 0.0])
                else:
                    pos = np.array([0.0, side_length * (4.0 - phase), self.trajectory_height])
                    vel = np.array([0.0, -side_length * self.trajectory_speed, 0.0])
                acc = np.array([0.0, 0.0, 0.0])
                    
            elif self.trajectory_mode == 'figure8':
                ang = t * self.trajectory_speed
                r = self.trajectory_radius
                pos = np.array([
                    r * math.sin(ang),
                    r * math.sin(ang) * math.cos(ang),
                    self.trajectory_height
                ])
                vel = np.array([
                    r * self.trajectory_speed * math.cos(ang),
                    r * self.trajectory_speed * math.cos(2*ang),
                    0.0
                ])
                acc = np.array([
                    -r * self.trajectory_speed**2 * math.sin(ang),
                    -2.0 * r * self.trajectory_speed**2 * math.sin(2*ang),
                    0.0
                ])
            
            elif self.trajectory_mode == 'up':
                # Gradual ascent from ground level
                ascent_speed = 0.5  # m/s vertical speed
                target_height = self.trajectory_height
                current_height = ascent_speed * t
                
                if current_height < target_height:
                    pos = np.array([0.0, 0.0, current_height])
                    vel = np.array([0.0, 0.0, ascent_speed])
                else:
                    pos = np.array([0.0, 0.0, target_height])
                    vel = np.array([0.0, 0.0, 0.0])
                acc = np.array([0.0, 0.0, 0.0])
            
            else:
                pos = np.array([0.0, 0.0, self.trajectory_height])
                vel = np.array([0.0, 0.0, 0.0])
                acc = np.array([0.0, 0.0, 0.0])
            
            # Compute desired orientation from required thrust direction
            thrust_vector = acc + np.array([0.0, 0.0, g])
            thrust_mag = np.linalg.norm(thrust_vector)
            
            if thrust_mag > 1e-6:
                z_body = thrust_vector / thrust_mag
                vel_mag = np.linalg.norm(vel[0:2])
                if vel_mag > 0.1:
                    yaw = math.atan2(vel[1], vel[0])
                else:
                    yaw = 0.0
                
                pitch = math.asin(np.clip(z_body[0], -1.0, 1.0))
                roll = math.atan2(-z_body[1], z_body[2])
                quat = euler_to_quaternion(roll, pitch, yaw)
            else:
                quat = np.array([0.0, 0.0, 0.0, 1.0])
            
            omega = np.array([0.0, 0.0, 0.0])
            q_arm = np.array([0.0, 0.0])
            
            x_ref[k] = np.concatenate([pos, vel, quat, omega, q_arm])
        
        return x_ref
    
    def control_loop(self):
        """Main MPC control loop"""
        if not self.control_enabled:
            msg = Wrench()
            msg.force.z = 0.0
            self.thrust_pub.publish(msg)
            return
        
        if not self.imu_received or not self.joints_received:
            return
        
        if self.start_log_time is None:
            self.start_log_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get current state estimate
        x_current = self.state_estimator.get_state()
        
        # Generate reference trajectory
        N = self.mpc_solver.params['N_horizon']
        x_ref = self.generate_reference_trajectory(self.trajectory_time, N)
        
        # Solve MPC optimization
        solve_start = time.time()
        try:
            u_opt, x_pred, solve_info = self.mpc_solver.solve(x_current, x_ref)
            solve_time = time.time() - solve_start
            
            # Apply first control input
            u_apply = u_opt[0]
            
            thrust = u_apply[0]
            torque_x = u_apply[1]
            torque_y = u_apply[2]
            torque_z = u_apply[3]
            arm_vel_1 = u_apply[4]
            arm_vel_2 = u_apply[5]
            
            # Publish base control
            thrust_msg = Wrench()
            thrust_msg.force.x = 0.0
            thrust_msg.force.y = 0.0
            thrust_msg.force.z = thrust
            thrust_msg.torque.x = torque_x
            thrust_msg.torque.y = torque_y
            thrust_msg.torque.z = torque_z
            self.thrust_pub.publish(thrust_msg)
            
            # Publish arm commands
            arm_msg = Float64MultiArray()
            arm_msg.data = [arm_vel_1, arm_vel_2]
            # FOR TEST ONLY!!
            arm_msg.data = [0., 0.]
            self.arm_cmd_pub.publish(arm_msg)
            
            # Update state estimator
            self.state_estimator.predict(u_apply, self.dt)
            
            # Log data
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_log_time
            self.log_time.append(current_time)
            self.log_pos.append(x_current[0:3].copy())
            self.log_pos_ref.append(x_ref[0, 0:3].copy())
            
            # Log attitude (convert quaternion to Euler angles)
            roll, pitch, yaw = quaternion_to_euler(x_current[6:10])
            roll_ref, pitch_ref, yaw_ref = quaternion_to_euler(x_ref[0, 6:10])
            self.log_att.append(np.array([roll, pitch, yaw]))
            self.log_att_ref.append(np.array([roll_ref, pitch_ref, yaw_ref]))
            
            self.log_control.append(u_apply.copy())
            self.log_solve_time.append(solve_time)
            self.log_cost.append(solve_info['cost'])
            
            # Track performance
            self.solve_times.append(solve_time)
            pos_error = np.linalg.norm(x_current[0:3] - x_ref[0, 0:3])
            self.tracking_errors.append(pos_error)
            
            # Log status periodically
            if int(self.trajectory_time / self.dt) % 40 == 0:
                avg_solve_time = np.mean(self.solve_times) * 1000
                avg_error = np.mean(self.tracking_errors)
                self.get_logger().info(
                    f'Mode: {self.trajectory_mode} | '
                    f'Pos: [{x_current[0]:.2f}, {x_current[1]:.2f}, {x_current[2]:.2f}] | '
                    f'Error: {pos_error:.3f}m | '
                    f'Thrust: {thrust:.1f}N | '
                    f'Solve: {solve_time*1000:.2f}ms (avg: {avg_solve_time:.2f}ms) | '
                    f'Cost: {solve_info["cost"]:.2f}'
                )
                
                if solve_time > self.dt:
                    self.get_logger().warn(
                        f'MPC solve time ({solve_time*1000:.1f}ms) exceeds control period ({self.dt*1000:.1f}ms)!'
                    )
            
        except Exception as e:
            self.get_logger().error(f'Acados MPC solve failed: {str(e)}')
            import traceback
            traceback.print_exc()
            # Fallback: hover control
            thrust_msg = Wrench()
            thrust_msg.force.z = 16.0  # Approximate hover thrust
            self.thrust_pub.publish(thrust_msg)
        
        self.trajectory_time += self.dt
    
    def start_trajectory(self, mode='circle'):
        """Start trajectory following with acados MPC"""
        self.trajectory_mode = mode
        self.trajectory_time = 0.0
        self.control_enabled = True
        self.state_estimator.reset()
        self.mpc_solver.reset()
        self.get_logger().info(f'Starting Acados MPC trajectory: {mode}')
    
    def stop(self):
        """Stop MPC controller"""
        self.control_enabled = False
        msg = Wrench()
        msg.force.z = 0.0
        self.thrust_pub.publish(msg)
        self.get_logger().info('Acados MPC Controller stopped')
    
    def plot_data(self):
        """Plot logged trajectory, attitude, and control data"""
        if len(self.log_time) == 0:
            self.get_logger().warn('No data to plot')
            return
        
        # Convert to numpy arrays
        t = np.array(self.log_time)
        pos = np.array(self.log_pos)
        pos_ref = np.array(self.log_pos_ref)
        att = np.array(self.log_att)  # [roll, pitch, yaw]
        att_ref = np.array(self.log_att_ref)
        control = np.array(self.log_control)
        
        # Create figure with subplots
        fig = plt.figure(figsize=(14, 10))
        
        # Position plots
        ax1 = plt.subplot(3, 3, 1)
        ax1.plot(t, pos[:, 0], 'b-', label='Actual', linewidth=1.5)
        ax1.plot(t, pos_ref[:, 0], 'r--', label='Reference', linewidth=1)
        ax1.set_ylabel('X Position (m)')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.set_title('Position Tracking')
        
        ax2 = plt.subplot(3, 3, 2)
        ax2.plot(t, pos[:, 1], 'b-', linewidth=1.5)
        ax2.plot(t, pos_ref[:, 1], 'r--', linewidth=1)
        ax2.set_ylabel('Y Position (m)')
        ax2.grid(True, alpha=0.3)
        
        ax3 = plt.subplot(3, 3, 3)
        ax3.plot(t, pos[:, 2], 'b-', linewidth=1.5)
        ax3.plot(t, pos_ref[:, 2], 'r--', linewidth=1)
        ax3.set_ylabel('Z Position (m)')
        ax3.grid(True, alpha=0.3)
        
        # Attitude plots (convert to degrees)
        ax4 = plt.subplot(3, 3, 4)
        ax4.plot(t, np.degrees(att[:, 0]), 'b-', label='Actual', linewidth=1.5)
        ax4.plot(t, np.degrees(att_ref[:, 0]), 'r--', label='Reference', linewidth=1)
        ax4.set_ylabel('Roll (deg)')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        ax4.set_title('Attitude')
        
        ax5 = plt.subplot(3, 3, 5)
        ax5.plot(t, np.degrees(att[:, 1]), 'b-', linewidth=1.5)
        ax5.plot(t, np.degrees(att_ref[:, 1]), 'r--', linewidth=1)
        ax5.set_ylabel('Pitch (deg)')
        ax5.grid(True, alpha=0.3)
        
        ax6 = plt.subplot(3, 3, 6)
        ax6.plot(t, np.degrees(att[:, 2]), 'b-', linewidth=1.5)
        ax6.plot(t, np.degrees(att_ref[:, 2]), 'r--', linewidth=1)
        ax6.set_ylabel('Yaw (deg)')
        ax6.grid(True, alpha=0.3)
        
        # Control signal plots
        ax7 = plt.subplot(3, 3, 7)
        ax7.plot(t, control[:, 0], 'g-', linewidth=1.5)
        ax7.set_ylabel('Thrust (N)')
        ax7.set_xlabel('Time (s)')
        ax7.grid(True, alpha=0.3)
        ax7.set_title('Control Signals')
        
        ax8 = plt.subplot(3, 3, 8)
        ax8.plot(t, control[:, 1], label='τx', linewidth=1.5)
        ax8.plot(t, control[:, 2], label='τy', linewidth=1.5)
        ax8.plot(t, control[:, 3], label='τz', linewidth=1.5)
        ax8.set_ylabel('Torques (Nm)')
        ax8.set_xlabel('Time (s)')
        ax8.grid(True, alpha=0.3)
        ax8.legend()
        
        ax9 = plt.subplot(3, 3, 9)
        ax9.plot(t, control[:, 4], label='Joint 1', linewidth=1.5)
        ax9.plot(t, control[:, 5], label='Joint 2', linewidth=1.5)
        ax9.set_ylabel('Arm Vel (rad/s)')
        ax9.set_xlabel('Time (s)')
        ax9.grid(True, alpha=0.3)
        ax9.legend()
        
        plt.suptitle(f'Acados MPC - {self.trajectory_mode.capitalize()} Trajectory', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.show()
        
        self.get_logger().info('Plot displayed')
    
    def print_performance_stats(self):
        """Print performance statistics"""
        if len(self.solve_times) == 0:
            self.get_logger().warn('No performance data available')
            return
        
        solve_times_ms = np.array(self.solve_times) * 1000
        errors = np.array(self.tracking_errors)
        
        self.get_logger().info('\n=== Acados MPC Performance Statistics ===')
        self.get_logger().info(f'Solve time (ms): mean={np.mean(solve_times_ms):.2f}, '
                              f'max={np.max(solve_times_ms):.2f}, '
                              f'min={np.min(solve_times_ms):.2f}, '
                              f'std={np.std(solve_times_ms):.2f}')
        self.get_logger().info(f'Tracking error (m): mean={np.mean(errors):.4f}, '
                              f'max={np.max(errors):.4f}, '
                              f'RMS={np.sqrt(np.mean(errors**2)):.4f}')
        
        control_period_ms = self.dt * 1000
        if np.max(solve_times_ms) < control_period_ms:
            slack = control_period_ms - np.max(solve_times_ms)
            self.get_logger().info(f'Real-time: ✓ FEASIBLE (slack: {slack:.2f}ms / {slack/control_period_ms*100:.1f}%)')
        else:
            self.get_logger().info(f'Real-time: ✗ NOT FEASIBLE')
        
        self.get_logger().info('==========================================')


def main(args=None):
    rclpy.init(args=args)
    controller = AcadosMPCController()
    
    import threading
    
    def command_loop():
        controller.get_logger().info('\n=== Acados MPC Controller Commands ===')
        controller.get_logger().info('  start <mode>  - Start trajectory (takeoff/hover/circle/square/figure8/up)')
        controller.get_logger().info('  stop          - Stop controller')
        controller.get_logger().info('  status        - Show current status')
        controller.get_logger().info('  stats         - Show performance statistics')
        controller.get_logger().info('  plot          - Plot trajectory and control data')
        controller.get_logger().info('  quit          - Exit')
        controller.get_logger().info('========================================\n')
        
        while rclpy.ok():
            try:
                cmd = input("Enter command: ").strip().lower().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'start':
                    mode = cmd[1] if len(cmd) > 1 else 'circle'
                    if mode in ['takeoff', 'hover', 'circle', 'square', 'figure8', 'up']:
                        controller.start_trajectory(mode)
                    else:
                        print(f"Unknown mode: {mode}")
                        
                elif cmd[0] == 'stop':
                    controller.stop()
                    
                elif cmd[0] == 'status':
                    state = controller.state_estimator.get_state()
                    print(f"Control enabled: {controller.control_enabled}")
                    print(f"Mode: {controller.trajectory_mode}")
                    print(f"Position: [{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}]")
                    print(f"Logged samples: {len(controller.log_time)}")
                    
                elif cmd[0] == 'stats':
                    controller.print_performance_stats()
                    
                elif cmd[0] == 'plot':
                    controller.plot_data()
                    
                elif cmd[0] in ['quit', 'exit']:
                    controller.stop()
                    rclpy.shutdown()
                    break
                    
                else:
                    print("Unknown command")
                    
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
