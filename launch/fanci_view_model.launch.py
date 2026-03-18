from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('am_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'FanciSwarmPro-URDF.urdf')

    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    rviz_config = LaunchConfiguration('rviz_config')

    # Read URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Joint State Publisher GUI (optional; useful only if URDF has movable joints)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_jsp_gui),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        on_exit=None,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_jsp_gui',
            default_value='false',
            description='Start joint_state_publisher_gui (only needed for URDF with joints).',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_path, 'config', 'view_model.rviz'),
            description='RViz config file',
        ),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
