#!/usr/bin/env python3
"""Republishes /scan_raw as /scan with two corrections applied:

1. header.frame_id set to lidar_link. See ros_gz_bridge.yaml for why:
   Gazebo's gpu_lidar sensor has no frame_id override, so it stamps
   LaserScan messages with an internal name that isn't a real TF frame.

2. Ranges below SELF_HIT_MAX_RANGE replaced with inf. The robot's own
   arm currently sags into the lidar's scanning plane (it has no active
   control yet -- Phase 5's job), producing short, ~0.1-0.15m "self-hit"
   readings on about 20% of the scan. Left in place, SLAM (and later
   Nav2's live costmap) has no way to tell these apart from real
   obstacles, and marks a small occupied blob wherever the robot
   currently is -- visible as a trail of stray dots tracing the robot's
   own driven path across the map. 0.2m is a safe cutoff: comfortably
   above the measured self-hit range (topped out ~0.150m), comfortably
   below any real obstacle distance in this room, so no genuine reading
   is ever discarded.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SELF_HIT_MAX_RANGE = 0.2


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)

    def callback(self, msg):
        msg.header.frame_id = 'lidar_link'
        msg.ranges = [
            r if r >= SELF_HIT_MAX_RANGE else float('inf')
            for r in msg.ranges
        ]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
