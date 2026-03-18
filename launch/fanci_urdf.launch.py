from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re


def generate_launch_description():

    pkg_path = get_package_share_directory('am_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'FanciSwarmPro-URDF.urdf')

    # Read URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # gazebo_ros/spawn_entity.py parses a Unicode string; lxml rejects an XML
    # encoding declaration in that case, so strip it if present.
    robot_description = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', robot_description, count=1)

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    # Start Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Spawn robot from robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'fanci_swarm_pro',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.2',
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
    ])
