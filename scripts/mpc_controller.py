#!/usr/bin/env python3
"""
MPC Controller for Aerial Manipulator

Uses Model Predictive Control to track trajectories with coupled quadrotor-arm dynamics.
Replaces PID controller with optimization-based control considering constraints.
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
import sys
import os

# Add parent directory to path to import mpc module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from am_description.mpc import MPCSolver, StateEstimator, euler_to_quaternion


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
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
        
        # MPC parameters (can be loaded from yaml)
        mpc_params = {
            'N_prediction': 20,
            'N_control': 20,
            'dt': 0.05,
            'Q_pos': 10.0,
            'Q_vel': 1.0,
            'Q_att': 5.0,
            'Q_omega': 0.5,
            'Q_arm': 2.0,
            'R_thrust': 0.01,
            'R_torque': 0.1,
            'R_arm': 0.05,
            'Rd_rate': 0.5,
            'thrust_min': 0.0,
            'thrust_max': 35.0,
            'torque_limit': 2.0,
            'arm_vel_limit': 2.0,
            'tilt_limit': 0.52,
            'max_iter': 100,
            'tolerance': 1e-4,
        }
        
        # Initialize MPC solver
        self.mpc_solver = MPCSolver(mpc_params)
        
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
        
        self.get_logger().info('MPC Controller initialized!')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Prediction horizon: {mpc_params["N_prediction"]} steps')
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
            # Assuming first two joints are arm joints
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
            x_ref: reference state trajectory [horizon x n_states]
        """
        x_ref = np.zeros((horizon + 1, 15))
        
        for k in range(horizon + 1):
            t = current_time + k * self.mpc_solver.params['dt']
            
            if self.trajectory_mode == 'hover':
                pos = np.array([0.0, 0.0, self.trajectory_height])
                vel = np.array([0.0, 0.0, 0.0])
                
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
            else:
                pos = np.array([0.0, 0.0, self.trajectory_height])
                vel = np.array([0.0, 0.0, 0.0])
            
            # Reference state: level attitude, zero angular velocity, zero arm joints
            quat = np.array([0.0, 0.0, 0.0, 1.0])  # Level orientation
            omega = np.array([0.0, 0.0, 0.0])
            q_arm = np.array([0.0, 0.0])
            
            x_ref[k] = np.concatenate([pos, vel, quat, omega, q_arm])
        
        return x_ref
    
    def control_loop(self):
        """Main MPC control loop"""
        if not self.control_enabled:
            # Send zero thrust when disabled
            msg = Wrench()
            msg.force.z = 0.0
            self.thrust_pub.publish(msg)
            return
        
        # Wait for sensor data
        if not self.imu_received or not self.joints_received:
            return
        
        # Initialize logging time
        if self.start_log_time is None:
            self.start_log_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get current state estimate
        x_current = self.state_estimator.get_state()
        
        # Generate reference trajectory
        N = self.mpc_solver.params['N_prediction']
        x_ref = self.generate_reference_trajectory(self.trajectory_time, N)
        
        # Solve MPC optimization
        solve_start = time.time()
        try:
            u_opt, x_pred, solve_info = self.mpc_solver.solve(x_current, x_ref)
            solve_time = time.time() - solve_start
            
            # Apply first control input
            u_apply = u_opt[0]
            
            # Extract control components
            thrust = u_apply[0]
            torque_x = u_apply[1]
            torque_y = u_apply[2]
            torque_z = u_apply[3]
            arm_vel_1 = u_apply[4]
            arm_vel_2 = u_apply[5]
            
            # Publish base control (thrust and torques)
            thrust_msg = Wrench()
            thrust_msg.force.x = 0.0
            thrust_msg.force.y = 0.0
            thrust_msg.force.z = thrust
            thrust_msg.torque.x = torque_x
            thrust_msg.torque.y = torque_y
            thrust_msg.torque.z = torque_z
            self.thrust_pub.publish(thrust_msg)
            
            # Publish arm commands (joint velocities)
            arm_msg = Float64MultiArray()
            arm_msg.data = [arm_vel_1, arm_vel_2]
            self.arm_cmd_pub.publish(arm_msg)
            
            # Update state estimator prediction
            self.state_estimator.predict(u_apply, self.dt)
            
            # Log data
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_log_time
            self.log_time.append(current_time)
            self.log_pos.append(x_current[0:3].copy())
            self.log_pos_ref.append(x_ref[0, 0:3].copy())
            self.log_control.append(u_apply.copy())
            self.log_solve_time.append(solve_time)
            self.log_cost.append(solve_info['cost'])
            
            # Track performance
            self.solve_times.append(solve_time)
            pos_error = np.linalg.norm(x_current[0:3] - x_ref[0, 0:3])
            self.tracking_errors.append(pos_error)
            
            # Log status periodically
            if int(self.trajectory_time / self.dt) % 40 == 0:  # Every 2 seconds at 20Hz
                avg_solve_time = np.mean(self.solve_times) * 1000  # ms
                avg_error = np.mean(self.tracking_errors)
                self.get_logger().info(
                    f'Mode: {self.trajectory_mode} | '
                    f'Pos: [{x_current[0]:.2f}, {x_current[1]:.2f}, {x_current[2]:.2f}] | '
                    f'Error: {pos_error:.3f}m | '
                    f'Thrust: {thrust:.1f}N | '
                    f'Solve: {solve_time*1000:.1f}ms (avg: {avg_solve_time:.1f}ms) | '
                    f'Cost: {solve_info["cost"]:.2f}'
                )
                
                # Warn if solve time exceeds budget
                if solve_time > self.dt:
                    self.get_logger().warn(
                        f'MPC solve time ({solve_time*1000:.1f}ms) exceeds control period ({self.dt*1000:.1f}ms)!'
                    )
            
        except Exception as e:
            self.get_logger().error(f'MPC solve failed: {str(e)}')
            # Fallback: hover control
            thrust_msg = Wrench()
            thrust_msg.force.z = self.mpc_solver.dynamics.total_mass * self.mpc_solver.dynamics.g
            self.thrust_pub.publish(thrust_msg)
        
        # Update trajectory time
        self.trajectory_time += self.dt
    
    def start_trajectory(self, mode='circle'):
        """Start trajectory following with MPC"""
        self.trajectory_mode = mode
        self.trajectory_time = 0.0
        self.control_enabled = True
        self.state_estimator.reset()
        self.get_logger().info(f'Starting MPC trajectory: {mode}')
    
    def stop(self):
        """Stop MPC controller"""
        self.control_enabled = False
        # Send zero thrust
        msg = Wrench()
        msg.force.z = 0.0
        self.thrust_pub.publish(msg)
        self.get_logger().info('MPC Controller stopped')
    
    def print_performance_stats(self):
        """Print performance statistics"""
        if len(self.solve_times) == 0:
            self.get_logger().warn('No performance data available')
            return
        
        solve_times_ms = np.array(self.solve_times) * 1000
        errors = np.array(self.tracking_errors)
        
        self.get_logger().info('\n=== MPC Performance Statistics ===')
        self.get_logger().info(f'Solve time (ms): mean={np.mean(solve_times_ms):.2f}, '
                              f'max={np.max(solve_times_ms):.2f}, '
                              f'min={np.min(solve_times_ms):.2f}')
        self.get_logger().info(f'Tracking error (m): mean={np.mean(errors):.4f}, '
                              f'max={np.max(errors):.4f}, '
                              f'RMS={np.sqrt(np.mean(errors**2)):.4f}')
        self.get_logger().info('===================================')


def main(args=None):
    rclpy.init(args=args)
    controller = MPCController()
    
    # Interactive command loop
    import threading
    
    def command_loop():
        controller.get_logger().info('\n=== MPC Controller Commands ===')
        controller.get_logger().info('  start <mode>  - Start trajectory (hover/circle/square/figure8)')
        controller.get_logger().info('  stop          - Stop controller')
        controller.get_logger().info('  status        - Show current status')
        controller.get_logger().info('  stats         - Show performance statistics')
        controller.get_logger().info('  quit          - Exit')
        controller.get_logger().info('================================\n')
        
        while rclpy.ok():
            try:
                cmd = input("Enter command: ").strip().lower().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'start':
                    mode = cmd[1] if len(cmd) > 1 else 'circle'
                    if mode in ['hover', 'circle', 'square', 'figure8']:
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
