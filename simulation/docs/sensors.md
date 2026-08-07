# Sensor and Simulation Topic Reference

Phase 2 deliverable: every topic the simulation environment produces or
consumes, its message type, frame, and measured publish rate. Rates below
were measured with the full system running together (Gazebo GUI, all
three RViz sensor displays, and the point cloud/lidar correction nodes
all active simultaneously) -- this is the honest, full-load number, not
an isolated best-case measurement, since that's what any future consumer
(Nav2, perception) will actually have to work with.

## Core simulation

| Topic | Type | Direction | Rate (requested / measured) | Notes |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS | n/a / ~700 Hz | Simulated clock; every other node syncs to this via `use_sim_time`. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo -> ROS | n/a / ~690 Hz | Published every physics step (no rate limit configured); drives `robot_state_publisher`'s TF computation. |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS | 50 Hz / ~33 Hz | `frame_id: odom`, `child_frame_id: base_footprint`. From DiffDrive's wheel-motion integration. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo -> ROS | n/a / ~33 Hz | Dynamic transforms only (`odom -> base_footprint` from DiffDrive, plus every non-fixed robot joint from `robot_state_publisher`). |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS | published once | Every `fixed` joint (base_footprint_joint, camera/lidar mounts, casters, etc.) -- published once at startup, never republished, per Transient Local QoS. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS -> Gazebo | n/a | Control input, not a sensor; drives DiffDrive. |

## RGB-D camera

| Topic | Type | Frame | Rate (requested / measured) | Notes |
|---|---|---|---|---|
| `/camera/image` | `sensor_msgs/msg/Image` (`rgb8`, 640x480) | `camera_link_optical` | 5 Hz / ~1.6 Hz | Rate lowered from an initial 15 Hz request after measuring the real achievable throughput; see Common Pitfalls below. |
| `/camera/depth_image` | `sensor_msgs/msg/Image` (`32FC1`, 640x480) | `camera_link_optical` | 5 Hz / same family as image | Verified non-degenerate: no NaN, not all-zero; `Inf` values correctly represent "nothing in range," not corrupted data. |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_link_optical` | same family | Intrinsics (`fx=fy≈554.4`, `cx=320`, `cy=240`) independently verified to mathematically match the configured 60° FOV. |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | `camera_link_optical` (label only) | -- | **Do not use.** Gazebo's native point cloud stores XYZ in body convention (X=forward, Y=left, Z=up) despite the optical frame_id label -- confirmed via raw-value cross-check against known object distances and camera height. |
| `/camera/points_corrected` | `sensor_msgs/msg/PointCloud2` | `camera_link_optical` | derived / ~0.6 Hz | Computed correctly from `/camera/depth_image` + `/camera/camera_info` via `depth_image_proc`, bypassing Gazebo's broken native output. |

## Lidar

| Topic | Type | Frame | Rate (requested / measured) | Notes |
|---|---|---|---|---|
| `/scan_raw` | `sensor_msgs/msg/LaserScan` | `amms_robot/base_footprint/lidar` (invalid) | 10 Hz / ~6.7 Hz | **Internal only, do not use.** Gazebo's gpu_lidar has no frame_id override (unlike the camera's `optical_frame_id`), so this is stamped with an auto-generated name that does not exist in the TF tree. |
| `/scan` | `sensor_msgs/msg/LaserScan` | `lidar_link` | derived / same as above | `/scan_raw` republished by `fix_lidar_frame.py` with the frame corrected. This is the topic Nav2/slam_toolbox should consume in Phase 3. Verified: 0 NaN, 0 Inf on a stationary robot; ~20% of the ring is occluded by the robot's own arm while it's uncontrolled/drooping under gravity (Phase 5 will resolve this at the root). |

## Common pitfalls encountered and fixed this phase

- **`use_sim_time` propagation**: audited across every node via `ros2 param get <node> use_sim_time`. Found `rviz2` was missing it (defaulting to wall-clock time while every message it renders uses sim time); fixed by launching RViz through `simulation.launch.py` with the parameter set explicitly, rather than launched by hand.
- **Camera rendering cost**: this VM's rendering throughput is well below any real sensor's datasheet numbers once Gazebo's GUI and RViz's own displays compete for the same GPU. Rates in this document are updated to reflect real, measured, full-system throughput rather than a requested target.
- **Frame-labeling mismatches**: two independent instances this phase (camera point cloud, lidar frame_id) where Gazebo produced technically-valid-looking data with an incorrect or unresolvable frame label. Both were caught by cross-checking raw values/frames against independently-known ground truth (object positions, TF resolution) rather than trusting metadata at face value.
