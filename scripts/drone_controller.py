#!/usr/bin/env python3
"""
Drone Controller Node - Controls thrust for the quadrotor
Publishes Wrench messages to control the drone
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
import sys


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Total mass: base(1.5) + arm_base(0.2) + arm1(0.1) + arm2(0.1) + 4*rotor(0.05*4) = 2.1 kg
        # Hover thrust = 2.1 * 9.81 ≈ 20.6 N
        self.hover_thrust = 20.6
        self.current_thrust = 0.0
        self.current_torque = [0.0, 0.0, 0.0]  # roll, pitch, yaw torques
        
        # Publisher for thrust (single force on base_link)
        self.thrust_pub = self.create_publisher(
            Wrench, '/aerial_manipulator/thrust', 10
        )
        
        # Arm controller publisher
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10
        )
        
        # Timer for publishing thrust commands
        self.timer = self.create_timer(0.02, self.publish_thrust)  # 50 Hz
        
        self.get_logger().info('Drone Controller started!')
        self.get_logger().info(f'Hover thrust: {self.hover_thrust:.2f} N')
        self.get_logger().info('Commands:')
        self.get_logger().info('  takeoff  - Take off and hover')
        self.get_logger().info('  hover    - Maintain hover')
        self.get_logger().info('  land     - Land the drone')
        self.get_logger().info('  stop     - Stop motors')
        self.get_logger().info('  up       - Increase thrust')
        self.get_logger().info('  down     - Decrease thrust')
        self.get_logger().info('  thrust <N> - Set specific thrust')
        self.get_logger().info('  arm <j1> <j2> - Set arm joint positions (radians)')
        
    def publish_thrust(self):
        """Publish current thrust"""
        msg = Wrench()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = self.current_thrust  # Upward force in body frame
        msg.torque.x = self.current_torque[0]
        msg.torque.y = self.current_torque[1]
        msg.torque.z = self.current_torque[2]
        self.thrust_pub.publish(msg)
    
    def takeoff(self):
        """Set thrust for takeoff (20% extra)"""
        self.current_thrust = self.hover_thrust * 1.3
        self.get_logger().info(f'Taking off with thrust: {self.current_thrust:.2f} N')
        
    def hover(self):
        """Set hover thrust"""
        self.current_thrust = self.hover_thrust
        self.get_logger().info(f'Hovering with thrust: {self.current_thrust:.2f} N')
        
    def land(self):
        """Reduce thrust for landing"""
        self.current_thrust = self.hover_thrust * 0.7
        self.get_logger().info(f'Landing with thrust: {self.current_thrust:.2f} N')
        
    def stop(self):
        """Stop all thrust"""
        self.current_thrust = 0.0
        self.current_torque = [0.0, 0.0, 0.0]
        self.get_logger().info('Motors stopped')
        
    def adjust_thrust(self, delta):
        """Adjust thrust by delta"""
        self.current_thrust = max(0.0, self.current_thrust + delta)
        self.get_logger().info(f'Thrust adjusted to: {self.current_thrust:.2f} N')
        
    def set_arm_position(self, joint1, joint2):
        """Set arm joint positions"""
        msg = Float64MultiArray()
        msg.data = [joint1, joint2]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Arm positions set: joint1={joint1:.2f}, joint2={joint2:.2f}')


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    
    # Interactive command loop in a separate thread
    import threading
    
    def command_loop():
        while rclpy.ok():
            try:
                cmd = input("Enter command: ").strip().lower().split()
                if not cmd:
                    continue
                    
                if cmd[0] == 'takeoff':
                    controller.takeoff()
                elif cmd[0] == 'hover':
                    controller.hover()
                elif cmd[0] == 'land':
                    controller.land()
                elif cmd[0] == 'stop':
                    controller.stop()
                elif cmd[0] == 'up':
                    controller.adjust_thrust(2.0)
                elif cmd[0] == 'down':
                    controller.adjust_thrust(-2.0)
                elif cmd[0] == 'arm' and len(cmd) >= 3:
                    j1 = float(cmd[1])
                    j2 = float(cmd[2])
                    controller.set_arm_position(j1, j2)
                elif cmd[0] == 'thrust' and len(cmd) >= 2:
                    t = float(cmd[1])
                    controller.current_thrust = t
                    controller.get_logger().info(f'Thrust set to: {t:.2f} N')
                elif cmd[0] == 'quit' or cmd[0] == 'exit':
                    controller.stop()
                    rclpy.shutdown()
                    break
                else:
                    print("Unknown command. Try: takeoff, hover, land, stop, up, down, arm <j1> <j2>, thrust <value>, quit")
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
