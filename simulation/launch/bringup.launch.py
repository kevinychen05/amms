"""Top-level system bringup: one command that starts every subsystem.

Phase 0: starts the three stub nodes that exist today (perception,
manipulation, mission_manager). robot_description, simulation's own Gazebo
world, and navigation have no launchable content yet -- see the commented
placeholders below for where each phase wires itself in.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
        Node(
            package='manipulation',
            executable='grasp_planner_node',
            name='grasp_planner_node',
            output='screen',
        ),
        Node(
            package='mission_manager',
            executable='mission_executive_node',
            name='mission_executive_node',
            output='screen',
        ),
        # Phase 1: robot_state_publisher, using robot_description's URDF
        # Phase 2: Gazebo world + ros_gz bridge, from this package's own launch/config
        # Phase 3: slam_toolbox / Nav2 stack, from /navigation
    ])
