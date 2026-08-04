import rclpy
from rclpy.node import Node


class MissionExecutiveNode(Node):
    """Stub node. Phase 6 adds the MissionCommand action server and the
    Idle -> Navigate -> Search -> Detect -> Align -> Grasp -> Transport ->
    Place -> Complete state graph, calling Nav2 and grasp_planner_node
    through their action interfaces."""

    def __init__(self):
        super().__init__('mission_executive_node')
        self.get_logger().info('mission_executive_node started (stub)')


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutiveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
