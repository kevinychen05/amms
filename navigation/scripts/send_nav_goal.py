#!/usr/bin/env python3
"""Send a NavigateToPose goal to Nav2 programmatically via its action
client, rather than RViz's 2D Goal Pose tool. This is the same action
interface (/navigate_to_pose) Phase 6's mission executive will call to
drive the robot as part of a larger mission.

Usage:
    ros2 run navigation send_nav_goal.py <x> <y> [yaw_degrees]
"""

import sys
import time
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavGoalClient(Node):

    def __init__(self):
        super().__init__('send_nav_goal')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._recoveries = 0

    def send_goal(self, x, y, yaw_rad=0.0):
        """Returns a result dict: success, error_code, status, recoveries, elapsed_sec.

        error_code/status/recoveries are -1 if the goal never reached a
        terminal state (e.g. rejected, or the action server never responded).
        """
        self._recoveries = 0
        start = time.monotonic()

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return self._result(False, -1, -1)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info(f'Sending goal: ({x}, {y}), yaw={math.degrees(yaw_rad):.1f} deg')
        send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self._goal_handle = send_goal_future.result()

        if self._goal_handle is None:
            self.get_logger().error('Goal send did not complete (node shutdown or interrupted)')
            return self._result(False, -1, -1, time.monotonic() - start)

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return self._result(False, -1, -1, time.monotonic() - start)

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        elapsed = time.monotonic() - start

        if result is None:
            self.get_logger().error('Goal did not reach a terminal state (interrupted)')
            return self._result(False, -1, -1, elapsed)

        # error_code 0 == NavigationResult::NONE, i.e. no error.
        success = result.result.error_code == 0
        if success:
            self.get_logger().info(f'Goal succeeded in {elapsed:.1f}s')
        else:
            self.get_logger().error(
                f'Goal failed: status={result.status}, '
                f'error_code={result.result.error_code}')
        return self._result(success, result.result.error_code, result.status, elapsed)

    def _result(self, success, error_code, status, elapsed_sec=0.0):
        return {
            'success': success,
            'error_code': error_code,
            'status': status,
            'recoveries': self._recoveries,
            'elapsed_sec': elapsed_sec,
        }

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._recoveries = fb.number_of_recoveries
        self.get_logger().info(
            f'distance_remaining={fb.distance_remaining:.2f}m, '
            f'recoveries={fb.number_of_recoveries}',
            throttle_duration_sec=2.0)


def main():
    if len(sys.argv) < 3:
        print(f'Usage: {sys.argv[0]} <x> <y> [yaw_degrees]')
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw_deg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = NavGoalClient()
    result = node.send_goal(x, y, math.radians(yaw_deg))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
