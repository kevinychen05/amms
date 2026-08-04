import rclpy
from rclpy.node import Node


class GraspPlannerNode(Node):
    """Stub node. Phase 5 adds the GraspTarget action server, grasp pose
    generation from DetectedObjectArray, and MoveIt 2 planning/execution
    calls to move_group."""

    def __init__(self):
        super().__init__('grasp_planner_node')
        self.get_logger().info('grasp_planner_node started (stub)')


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
