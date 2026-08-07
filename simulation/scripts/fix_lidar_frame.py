#!/usr/bin/env python3
"""Republishes /scan_raw as /scan with header.frame_id corrected to
lidar_link. See ros_gz_bridge.yaml for why this exists: Gazebo's
gpu_lidar sensor has no frame_id override, so it stamps LaserScan
messages with an internal name that isn't a real TF frame. Only the
label is wrong here, not the data, so this only rewrites one field.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FixLidarFrame(Node):

    def __init__(self):
        super().__init__('fix_lidar_frame')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)

    def callback(self, msg):
        msg.header.frame_id = 'lidar_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixLidarFrame()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
