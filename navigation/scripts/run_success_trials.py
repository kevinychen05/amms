#!/usr/bin/env python3
"""Run the robot through a fixed, repeatable set of waypoints and log a
navigation success-rate metric: completion/failure, Nav2's error_code,
number of recoveries, and time-to-goal for each trial. This is Phase 3's
success-rate deliverable -- a quantitative baseline other phases (Phase 8
in particular) can compare against after robustness improvements.

Waypoints are fixed (not randomized) so the test set is repeatable run to
run, and deliberately span both sides of the doorway plus a mix of short
and long-distance goals, verified clear against the saved map beforehand.

Usage:
    ros2 run navigation run_success_trials.py [output_csv_path]
"""

import sys
import os
import csv
import statistics

import rclpy

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from send_nav_goal import NavGoalClient  # noqa: E402


WAYPOINTS = [
    (1.5, 0.5), (-1.5, 0.0), (1.5, -0.5), (-1.5, 0.3), (1.0, 0.0),
    (4.3, 4.3), (-4.3, -4.3), (4.3, -4.3), (-4.3, 4.3), (1.0, 3.5),
    (3.0, 3.5), (-1.4, 3.4), (2.0, 0.0), (-3.5, -1.0), (2.5, 1.8),
    (2.0, -3.5), (-2.0, 3.0), (0.5, 3.0), (-0.8, -3.5), (2.5, 0.3),
]


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else 'success_rate_trials.csv'

    rclpy.init()
    node = NavGoalClient()

    rows = []
    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['trial', 'goal_x', 'goal_y', 'success', 'error_code',
                          'status', 'recoveries', 'elapsed_sec'])

        for i, (x, y) in enumerate(WAYPOINTS, 1):
            node.get_logger().info(f'=== Trial {i}/{len(WAYPOINTS)}: goal ({x}, {y}) ===')
            result = node.send_goal(x, y)
            row = [i, x, y, result['success'], result['error_code'],
                   result['status'], result['recoveries'], f"{result['elapsed_sec']:.2f}"]
            writer.writerow(row)
            f.flush()
            rows.append(result)

    node.destroy_node()
    rclpy.shutdown()

    successes = [r for r in rows if r['success']]
    success_rate = len(successes) / len(rows) * 100.0
    print(f'\n=== Summary: {len(successes)}/{len(rows)} succeeded ({success_rate:.1f}%) ===')
    if successes:
        times = [r['elapsed_sec'] for r in successes]
        print(f'Time-to-goal (successful trials): '
              f'mean={statistics.mean(times):.1f}s, median={statistics.median(times):.1f}s, '
              f'min={min(times):.1f}s, max={max(times):.1f}s')
    failures = [r for r in rows if not r['success']]
    if failures:
        codes = sorted(set(r['error_code'] for r in failures))
        print(f'Failures: {len(failures)}, error_codes seen: {codes}')
    print(f'Log written to {out_path}')


if __name__ == '__main__':
    main()
