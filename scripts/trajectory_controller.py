#!/usr/bin/env python3
"""
Trajectory Controller - Makes the drone follow a predefined trajectory
Uses PID control to track position and heading
Estimates position from IMU data through integration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
import math
import numpy as np


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # Drone parameters
        self.mass = 2.1  # kg
        self.hover_thrust = self.mass * 9.81  # N
        
        # Current state (estimated from IMU)
        self.position = np.array([0.0, 0.0, 1.0])  # Start at spawn height
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])
        
        # IMU integration
        self.last_imu_time = None
        self.gravity = np.array([0.0, 0.0, 9.81])  # Gravity in world frame
        self.velocity_bias = np.array([0.0, 0.0, 0.0])  # Drift correction
        self.position_initialized = False
        
        # Complementary filter for height (using acceleration)
        self.alpha_height = 0.98  # Trust velocity integration vs acceleration
        
        # Target state
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
        
        # Trajectory parameters
        self.trajectory_mode = 'circle'  # 'hover', 'circle', 'square', 'figure8'
        self.trajectory_radius = 1.5
        self.trajectory_speed = 0.3  # rad/s
        self.trajectory_height = 2.0
        self.trajectory_time = 0.0
        
        # Control state
        self.control_enabled = False
        self.takeoff_complete = False
        
        # Subscriber for IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/aerial_manipulator/imu',
            self.imu_callback,
            10
        )
        
        # Publisher for thrust commands
        self.thrust_pub = self.create_publisher(
            Wrench,
            '/aerial_manipulator/thrust',
            10
        )
        
        # Control timer (50 Hz)
        self.dt = 0.02
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Trajectory Controller initialized!')
        self.get_logger().info('Using IMU-based state estimation (position from integration)')
        self.get_logger().info('WARNING: Position will drift over time due to integration errors!')
        self.get_logger().info('Waiting for IMU data...')
        
    def imu_callback(self, msg: Imu):
        """Update state from IMU and integrate to estimate position/velocity"""
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get orientation directly from IMU (this is reliable)
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
            
            if dt > 0 and dt < 0.1:  # Sanity check on dt
                # Integrate acceleration to velocity (with drift correction)
                self.velocity += self.linear_acceleration * dt
                
                # Apply velocity damping to reduce drift (simple approach)
                # This assumes drone returns to zero velocity when hovering
                drift_correction = 0.01  # Small damping factor
                self.velocity *= (1.0 - drift_correction * dt)
                
                # Integrate velocity to position
                self.position += self.velocity * dt
                
                # Ground constraint: position z cannot go below 0
                if self.position[2] < 0:
                    self.position[2] = 0
                    self.velocity[2] = max(0, self.velocity[2])
        
        self.last_imu_time = current_time
    
    def rotate_vector_by_quaternion(self, v, q):
        """Rotate vector v by quaternion q"""
        # q = [x, y, z, w]
        qx, qy, qz, qw = q
        
        # Quaternion rotation: v' = q * v * q^-1
        # Using rotation matrix derived from quaternion
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        
        return R @ v
    
    def compute_trajectory(self):
        """Compute target position and velocity based on trajectory mode"""
        if not self.control_enabled:
            return
        
        self.trajectory_time += self.dt
        
        if self.trajectory_mode == 'hover':
            self.target_position = np.array([0.0, 0.0, self.trajectory_height])
            self.target_velocity = np.array([0.0, 0.0, 0.0])
            
        elif self.trajectory_mode == 'circle':
            t = self.trajectory_time * self.trajectory_speed
            r = self.trajectory_radius
            self.target_position = np.array([
                r * math.cos(t),
                r * math.sin(t),
                self.trajectory_height
            ])
            self.target_velocity = np.array([
                -r * self.trajectory_speed * math.sin(t),
                r * self.trajectory_speed * math.cos(t),
                0.0
            ])
            
        elif self.trajectory_mode == 'square':
            t = self.trajectory_time * self.trajectory_speed
            side_length = 2.0
            period = 4.0  # 4 sides
            phase = (t % (2 * math.pi)) / (2 * math.pi) * period
            
            if phase < 1.0:  # Side 1: +X
                self.target_position = np.array([
                    side_length * phase,
                    0.0,
                    self.trajectory_height
                ])
                self.target_velocity = np.array([side_length * self.trajectory_speed, 0.0, 0.0])
            elif phase < 2.0:  # Side 2: +Y
                self.target_position = np.array([
                    side_length,
                    side_length * (phase - 1.0),
                    self.trajectory_height
                ])
                self.target_velocity = np.array([0.0, side_length * self.trajectory_speed, 0.0])
            elif phase < 3.0:  # Side 3: -X
                self.target_position = np.array([
                    side_length * (3.0 - phase),
                    side_length,
                    self.trajectory_height
                ])
                self.target_velocity = np.array([-side_length * self.trajectory_speed, 0.0, 0.0])
            else:  # Side 4: -Y
                self.target_position = np.array([
                    0.0,
                    side_length * (4.0 - phase),
                    self.trajectory_height
                ])
                self.target_velocity = np.array([0.0, -side_length * self.trajectory_speed, 0.0])
                
        elif self.trajectory_mode == 'figure8':
            t = self.trajectory_time * self.trajectory_speed
            r = self.trajectory_radius
            self.target_position = np.array([
                r * math.sin(t),
                r * math.sin(t) * math.cos(t),
                self.trajectory_height
            ])
            self.target_velocity = np.array([
                r * self.trajectory_speed * math.cos(t),
                r * self.trajectory_speed * (math.cos(2*t)),
                0.0
            ])
    
    def control_loop(self):
        """Main control loop - PID position control"""
        if not self.control_enabled:
            # Send zero thrust when disabled
            msg = Wrench()
            msg.force.z = 0.0
            self.thrust_pub.publish(msg)
            return
        
        # Compute trajectory
        self.compute_trajectory()
        
        # Position error
        pos_error = self.target_position - self.position
        vel_error = self.target_velocity - self.velocity
        
        # Update integral
        self.integral_error += pos_error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        
        # PID control output (desired acceleration)
        acc_desired = (self.kp_pos * pos_error + 
                      self.kd_pos * vel_error + 
                      self.ki_pos * self.integral_error)
        
        # Compute thrust (compensate for gravity + desired acceleration)
        thrust_z = self.mass * (9.81 + acc_desired[2])
        
        # Compute desired tilt angles for horizontal control
        # Small angle approximation: thrust_x ≈ thrust_total * theta_y
        thrust_total = max(thrust_z, 1.0)  # Avoid division by zero
        desired_roll = -math.atan2(acc_desired[1] * self.mass, thrust_total)
        desired_pitch = math.atan2(acc_desired[0] * self.mass, thrust_total)
        
        # Get current roll, pitch from quaternion
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)
        
        # Attitude error (simple P control for now)
        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch
        
        # Compute torques
        torque_x = self.kp_att[0] * roll_error
        torque_y = self.kp_att[1] * pitch_error
        torque_z = 0.0  # No yaw control for now
        
        # Clamp thrust
        thrust_z = np.clip(thrust_z, 0.0, 40.0)
        
        # Publish command
        msg = Wrench()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = thrust_z
        msg.torque.x = torque_x
        msg.torque.y = torque_y
        msg.torque.z = torque_z
        
        self.thrust_pub.publish(msg)
        
        # Log status periodically
        if int(self.trajectory_time * 10) % 20 == 0:  # Every 2 seconds
            pos_err_mag = np.linalg.norm(pos_error)
            self.get_logger().info(
                f'Mode: {self.trajectory_mode} | '
                f'Pos: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}] | '
                f'Target: [{self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f}] | '
                f'Error: {pos_err_mag:.2f}m | Thrust: {thrust_z:.1f}N'
            )
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to roll, pitch, yaw"""
        # q = [x, y, z, w]
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
    
    def start_trajectory(self, mode='circle'):
        """Start trajectory following"""
        self.trajectory_mode = mode
        self.trajectory_time = 0.0
        self.control_enabled = True
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.get_logger().info(f'Starting trajectory: {mode}')
    
    def stop(self):
        """Stop controller"""
        self.control_enabled = False
        self.get_logger().info('Controller stopped')


def main(args=None):
    rclpy.init(args=args)
    controller = TrajectoryController()
    
    # Interactive command loop in a separate thread
    import threading
    
    def command_loop():
        controller.get_logger().info('\n=== Trajectory Controller Commands ===')
        controller.get_logger().info('  start <mode>  - Start trajectory (hover/circle/square/figure8)')
        controller.get_logger().info('  stop          - Stop controller')
        controller.get_logger().info('  status        - Show current status')
        controller.get_logger().info('  quit          - Exit')
        controller.get_logger().info('=====================================\n')
        
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
                        print(f"Unknown mode: {mode}. Use: hover, circle, square, or figure8")
                        
                elif cmd[0] == 'stop':
                    controller.stop()
                    
                elif cmd[0] == 'status':
                    print(f"Control enabled: {controller.control_enabled}")
                    print(f"Mode: {controller.trajectory_mode}")
                    print(f"Position: {controller.position}")
                    print(f"Target: {controller.target_position}")
                    
                elif cmd[0] in ['quit', 'exit']:
                    controller.stop()
                    rclpy.shutdown()
                    break
                    
                else:
                    print("Unknown command. Try: start <mode>, stop, status, quit")
                    
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
