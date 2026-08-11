"""Core Nav2 stack: global planner, local controller, costmaps, and
recovery behaviors, orchestrated by bt_navigator. Run alongside
simulation.launch.py and localization.launch.py (map->odom must already
exist for the global costmap/planner to have a map frame to work in).

Deliberately minimal and hand-written rather than reusing nav2_bringup's
own navigation_launch.py: that file's lifecycle manager brings up 10
components (route_server, velocity_smoother, collision_monitor,
waypoint_follower, docking_server, smoother_server, in addition to the
four this phase actually needs), and since ALL listed lifecycle nodes
must reach "active" for autostart to succeed, reusing it as-is would
mean either configuring six components we don't want or risking the
whole bringup failing to activate. This brings up exactly
controller_server, planner_server, behavior_server, and bt_navigator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('navigation')
    params_path = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[params_path, {'use_sim_time': True}],
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[params_path, {'use_sim_time': True}],
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[params_path, {'use_sim_time': True}],
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[params_path, {'use_sim_time': True}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': lifecycle_nodes,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
