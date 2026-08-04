import rclpy
from rclpy.node import Node


class PerceptionNode(Node):
    """Stub node. Phase 4 adds RGB-D subscription, YOLO detection, and
    publishing DetectedObjectArray on /perception/detected_objects."""

    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info('perception_node started (stub)')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
