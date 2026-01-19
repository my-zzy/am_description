from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('am_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'am_min.urdf')

    return LaunchDescription([

        # Start Gazebo server
        ExecuteProcess(
            cmd=[
                'gzserver',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Start Gazebo client (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Spawn robot from URDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'aerial_manipulator',
                '-file', urdf_path,
                '-x', '0',
                '-y', '0',
                '-z', '1'
            ],
            output='screen'
        ),

        # # Controller Manager
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[urdf_path, os.path.join(pkg_path, 'config', 'controllers.yaml')],
        #     output='screen'
        # ),

        # # Joint State Broadcaster Spawner
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster'],
        #     output='screen'
        # ),

        # # Arm Joint 1 Position Controller Spawner
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['arm_joint_1_position_controller'],
        #     output='screen'
        # ),

        # # Arm Joint 2 Position Controller Spawner
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['arm_joint_2_position_controller'],
        #     output='screen'
        # ),
    ])
