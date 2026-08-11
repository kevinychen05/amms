"""Live SLAM mapping. Run alongside simulation.launch.py (which must
already be running: it provides /scan, /odom, and the TF tree this
depends on). Reuses slam_toolbox's own online_async_launch.py rather
than reimplementing its lifecycle-node configure/activate handling.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_navigation = get_package_share_directory('navigation')
    params_path = os.path.join(pkg_navigation, 'config', 'slam_toolbox_params.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': params_path,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        slam,
    ])
