import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    default_xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Path to the robot xacro file',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    return LaunchDescription([
        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config_path],
        ),
    ])
