"""Phase 2 simulation bringup: the Gazebo world, robot_state_publisher (so
/tf and /robot_description exist), the robot spawned into the world, the
ros_gz_bridge translating Gazebo's internal topics (clock, odometry,
joint states, odom->base_footprint TF, RGB-D camera, lidar) into standard
ROS 2 topics, a corrected point cloud computed from the depth image
(Gazebo's own /camera/points has a body-vs-optical axis mismatch), and a
corrected lidar frame_id (Gazebo's gpu_lidar has no frame override, so it
stamps scans with a non-existent TF frame).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_simulation = get_package_share_directory('simulation')
    pkg_robot_description = get_package_share_directory('robot_description')
    world_path = os.path.join(pkg_simulation, 'worlds', 'warehouse.sdf')
    xacro_path = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    bridge_config_path = os.path.join(pkg_simulation, 'config', 'ros_gz_bridge.yaml')
    rviz_config_path = os.path.join(pkg_simulation, 'rviz', 'simulation.rviz')

    # '-r' starts the world already running; without it gz sim starts
    # paused and nothing happens until a human clicks play in the GUI.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawns by subscribing to the /robot_description topic above (Transient
    # Local durability means it doesn't matter whether robot_state_publisher
    # or this spawn request starts first) rather than re-reading the xacro
    # file independently -- one source of truth for what gets spawned.
    # base_footprint (this URDF's root link) is defined at ground level, so
    # z=0 places the robot correctly resting on the floor, not embedded in it
    # or floating above it.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'warehouse',
            '-topic', 'robot_description',
            '-name', 'amms_robot',
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )

    # Bridges Gazebo's internal transport to ROS 2 topics per
    # config/ros_gz_bridge.yaml -- see that file for exactly which topics
    # and why. use_sim_time is intentionally NOT set here: this node is
    # what PRODUCES /clock for everyone else to consume, so it has to run
    # on its own wall-clock timing to bridge Gazebo's clock in the first
    # place.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_path}],
    )

    # Gazebo's own rgbd_camera sensor publishes /camera/points directly,
    # but its raw XYZ values turned out to use body convention (X=forward,
    # Y=left, Z=up) while stamping the message with the optical frame_id
    # -- a genuine upstream mismatch, confirmed by cross-checking raw
    # point values against known object distances/camera height, not a
    # bug in this URDF's own optical-frame setup (image/depth_image/
    # camera_info were all independently verified correct). Rather than
    # trust Gazebo's point cloud, this computes one ourselves from the
    # already-verified depth image + camera_info, via the same pinhole
    # back-projection Phase 4 will need anyway for turning a detected
    # object's pixel into a 3D position.
    point_cloud_container = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', '/camera/image'),
                    ('rgb/camera_info', '/camera/camera_info'),
                    ('depth_registered/image_rect', '/camera/depth_image'),
                    ('points', '/camera/points_corrected'),
                ],
                parameters=[{'use_sim_time': True}],
            ),
        ],
        output='screen',
    )

    # See ros_gz_bridge.yaml for why this exists: the lidar's LaserScan
    # messages arrive on /scan_raw stamped with a frame_id that isn't a
    # real TF frame. This republishes as /scan with the frame corrected.
    fix_lidar_frame = Node(
        package='simulation',
        executable='fix_lidar_frame.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # use_sim_time here matters concretely, not just as boilerplate: RViz's
    # own internal notion of "now" (used for TF buffer lookups) would
    # otherwise run on wall-clock time while every message it renders is
    # stamped with simulated time. Since this VM has consistently run
    # below real-time under rendering load all phase, those two clocks
    # drift apart -- exactly the kind of silent mismatch this phase's
    # most-common pitfall warns about, and a plausible contributor to
    # intermittent rendering glitches observed while developing this
    # launch file.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        point_cloud_container,
        fix_lidar_frame,
        rviz,
    ])
