from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('am_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'am_min.urdf')
    controllers_path = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # Read URDF file and substitute controller path
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    
    # Replace the placeholder with actual path
    robot_description = robot_description.replace('$(controllers_yaml_path)', controllers_path)

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Start Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn robot from robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'aerial_manipulator',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '1'
        ],
        output='screen'
    )

    # Load and start controllers after spawning
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # Delay controller loading to ensure Gazebo and robot are ready
    delayed_controller_load = TimerAction(
        period=5.0,
        actions=[
            load_joint_state_broadcaster,
            load_arm_controller,
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        delayed_controller_load,
    ])
