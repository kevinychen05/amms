"""AMCL localization against the saved map, as an alternative to live
SLAM. Run alongside simulation.launch.py (must already be running: it
provides /scan, /odom, and the TF tree this depends on), and instead of
navigation/launch/slam.launch.py, not alongside it -- both would fight
over publishing map->odom.

Reuses nav2_bringup's own localization_launch.py (map_server + amcl +
a lifecycle_manager that auto-activates both) rather than reimplementing
lifecycle-node bringup ourselves, same reasoning as slam.launch.py
reusing slam_toolbox's own launch file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_navigation = get_package_share_directory('navigation')
    params_path = os.path.join(pkg_navigation, 'config', 'amcl_params.yaml')
    map_path = os.path.join(pkg_navigation, 'maps', 'warehouse.yaml')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': params_path,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        localization,
    ])
