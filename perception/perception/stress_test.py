"""Sample /perception/detected_objects for a fixed window under a named
scene condition (baseline, occlusion, poor lighting, edge-of-FOV) and log
the target-class detection rate and confidence to a CSV. Scene setup itself
(spawning an occluder, dimming lights, repositioning the robot) is done
externally before running this -- this script only measures and logs.

Usage:
    ros2 run perception stress_test.py <condition_name> <duration_sec> [csv_path]
"""

import sys
import time
import csv
import os
import statistics

import rclpy
from rclpy.node import Node
from interfaces.msg import DetectedObjectArray

TARGET_CLASS = 'cup'


class StressTestSampler(Node):

    def __init__(self, condition, duration_sec):
        super().__init__('stress_test_sampler')
        self.condition = condition
        self.duration_sec = duration_sec
        self.frames_sampled = 0
        self.detections = 0
        self.confidences = []
        self.create_subscription(
            DetectedObjectArray, '/perception/detected_objects', self._callback, 10)

    def _callback(self, msg):
        self.frames_sampled += 1
        hits = [o for o in msg.objects if o.class_label == TARGET_CLASS]
        if hits:
            self.detections += 1
            # If more than one hit somehow, take the most confident.
            self.confidences.append(max(float(o.confidence) for o in hits))

    def run(self):
        start = time.monotonic()
        while time.monotonic() - start < self.duration_sec:
            rclpy.spin_once(self, timeout_sec=0.2)

    def summary(self):
        rate = (self.detections / self.frames_sampled * 100.0) if self.frames_sampled else 0.0
        mean_conf = statistics.mean(self.confidences) if self.confidences else 0.0
        return {
            'condition': self.condition,
            'frames_sampled': self.frames_sampled,
            'detections': self.detections,
            'detection_rate_pct': round(rate, 1),
            'mean_confidence': round(mean_conf, 3),
        }


def main():
    if len(sys.argv) < 3:
        print(f'Usage: {sys.argv[0]} <condition_name> <duration_sec> [csv_path]')
        sys.exit(1)

    condition = sys.argv[1]
    duration_sec = float(sys.argv[2])
    csv_path = sys.argv[3] if len(sys.argv) > 3 else 'stress_test_results.csv'

    rclpy.init()
    node = StressTestSampler(condition, duration_sec)
    node.get_logger().info(f'Sampling condition "{condition}" for {duration_sec}s...')
    node.run()
    result = node.summary()
    node.destroy_node()
    rclpy.shutdown()

    write_header = not os.path.exists(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(result.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(result)

    print(f'Result: {result}')
    print(f'Appended to {csv_path}')


if __name__ == '__main__':
    main()
