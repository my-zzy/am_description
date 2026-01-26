#!/usr/bin/env python3
"""
Drone Controller Node - Controls thrust for the quadrotor
Publishes Wrench messages to four rotor thrust topics
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
        # Hover thrust per rotor = (2.1 * 9.81) / 4 ≈ 5.15 N
        self.hover_thrust = 5.15
        self.current_thrust = [0.0, 0.0, 0.0, 0.0]
        
        # Publishers for each rotor thrust
        self.thrust_pubs = [
            self.create_publisher(Wrench, '/aerial_manipulator/rotor_1/thrust', 10),
            self.create_publisher(Wrench, '/aerial_manipulator/rotor_2/thrust', 10),
            self.create_publisher(Wrench, '/aerial_manipulator/rotor_3/thrust', 10),
            self.create_publisher(Wrench, '/aerial_manipulator/rotor_4/thrust', 10),
        ]
        
        # Arm controller publisher
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10
        )
        
        # Timer for publishing thrust commands
        self.timer = self.create_timer(0.02, self.publish_thrust)  # 50 Hz
        
        self.get_logger().info('Drone Controller started!')
        self.get_logger().info(f'Hover thrust per rotor: {self.hover_thrust:.2f} N')
        self.get_logger().info('Commands:')
        self.get_logger().info('  takeoff  - Take off and hover')
        self.get_logger().info('  land     - Land the drone')
        self.get_logger().info('  up       - Increase thrust')
        self.get_logger().info('  down     - Decrease thrust')
        self.get_logger().info('  arm <j1> <j2> - Set arm joint positions (radians)')
        
    def set_thrust(self, thrust_values):
        """Set thrust for all rotors"""
        self.current_thrust = thrust_values
        
    def publish_thrust(self):
        """Publish current thrust to all rotors"""
        for i, pub in enumerate(self.thrust_pubs):
            msg = Wrench()
            msg.force.x = 0.0
            msg.force.y = 0.0
            msg.force.z = self.current_thrust[i]  # Upward force in body frame
            msg.torque.x = 0.0
            msg.torque.y = 0.0
            msg.torque.z = 0.0
            pub.publish(msg)
    
    def takeoff(self):
        """Set hover thrust for takeoff"""
        thrust = self.hover_thrust * 1.2  # 20% extra for takeoff
        self.set_thrust([thrust, thrust, thrust, thrust])
        self.get_logger().info(f'Taking off with thrust: {thrust:.2f} N per rotor')
        
    def hover(self):
        """Set hover thrust"""
        self.set_thrust([self.hover_thrust] * 4)
        self.get_logger().info(f'Hovering with thrust: {self.hover_thrust:.2f} N per rotor')
        
    def land(self):
        """Reduce thrust for landing"""
        thrust = self.hover_thrust * 0.8
        self.set_thrust([thrust, thrust, thrust, thrust])
        self.get_logger().info(f'Landing with thrust: {thrust:.2f} N per rotor')
        
    def stop(self):
        """Stop all rotors"""
        self.set_thrust([0.0, 0.0, 0.0, 0.0])
        self.get_logger().info('Motors stopped')
        
    def adjust_thrust(self, delta):
        """Adjust all thrust by delta"""
        new_thrust = [t + delta for t in self.current_thrust]
        self.set_thrust(new_thrust)
        self.get_logger().info(f'Thrust adjusted to: {new_thrust[0]:.2f} N per rotor')
        
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
                    controller.adjust_thrust(1.0)
                elif cmd[0] == 'down':
                    controller.adjust_thrust(-1.0)
                elif cmd[0] == 'arm' and len(cmd) >= 3:
                    j1 = float(cmd[1])
                    j2 = float(cmd[2])
                    controller.set_arm_position(j1, j2)
                elif cmd[0] == 'thrust' and len(cmd) >= 2:
                    t = float(cmd[1])
                    controller.set_thrust([t, t, t, t])
                    controller.get_logger().info(f'Thrust set to: {t:.2f} N per rotor')
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
