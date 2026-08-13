import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from interfaces.msg import DetectedObject, DetectedObjectArray
import message_filters
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 -- registers PointStamped support for Buffer.transform()

# Our target object (a red mug in the world) maps to COCO's "cup" class,
# the only category a stock-pretrained detector could plausibly recognize
# among our fabricated warehouse geometry.
TARGET_CLASS = 'cup'
CONFIDENCE_THRESHOLD = 0.25

# Half-width (pixels) of the patch averaged around a box center when
# sampling depth, so a single bad pixel (a depth-image "hole") doesn't
# wreck the estimate.
DEPTH_PATCH_RADIUS = 3


class PerceptionNode(Node):
    """Detects a target object in RGB imagery, estimates its 3D pose from
    depth, and publishes DetectedObjectArray on /perception/detected_objects.
    """

    def __init__(self):
        super().__init__('perception_node')

        self.bridge = CvBridge()
        self.get_logger().info('Loading YOLOv8n...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8n loaded')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detections_pub = self.create_publisher(
            DetectedObjectArray, '/perception/detected_objects', 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, '/perception/detection_markers', 10)

        self.camera_info = None
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_callback, 1)

        rgb_sub = message_filters.Subscriber(self, Image, '/camera/image')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth_image')
        # ApproximateTimeSynchronizer (not exact) because real RGB-D hardware
        # publishes color and depth as two independently-timestamped streams
        # that rarely share an identical stamp; slop=0.05s tolerates that
        # while still rejecting genuinely mismatched pairs.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self._synced_callback)

        self.get_logger().info('perception_node started, waiting for synced RGB-D pairs...')

    def _camera_info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info(
                f'camera_info received: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}, size={msg.width}x{msg.height}')
        self.camera_info = msg

    def _synced_callback(self, rgb_msg, depth_msg):
        if self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        t0 = time.monotonic()
        results = self.model(cv_image, verbose=False)[0]
        inference_ms = (time.monotonic() - t0) * 1000.0

        objects = []
        for box in results.boxes:
            class_name = self.model.names[int(box.cls[0])]
            confidence = float(box.conf[0])
            if confidence < CONFIDENCE_THRESHOLD or class_name != TARGET_CLASS:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = int((x1 + x2) / 2.0)
            v = int((y1 + y2) / 2.0)

            point = self._back_project(u, v, depth_image)
            if point is None:
                self.get_logger().warn(
                    f'{class_name} ({confidence:.2f}) at pixel ({u},{v}): no valid depth in patch')
                continue

            map_point = self._transform_to_map(point, rgb_msg.header)
            if map_point is None:
                self.get_logger().warn(
                    f'{class_name} ({confidence:.2f}): back-projected but TF transform '
                    f'to map failed', throttle_duration_sec=1.0)
                continue

            bbox_size = self._estimate_bbox_size(x1, y1, x2, y2, point[2])

            obj = DetectedObject()
            obj.class_label = class_name
            obj.confidence = confidence
            obj.bounding_box_size = bbox_size
            obj.pose.header.frame_id = 'map'
            obj.pose.header.stamp = rgb_msg.header.stamp
            obj.pose.pose.position.x = map_point[0]
            obj.pose.pose.position.y = map_point[1]
            obj.pose.pose.position.z = map_point[2]
            # No orientation estimate: we're using the simple centroid/bbox
            # approach (Step 3's scope decision), not ICP refinement against
            # a known object model, so identity is the honest answer here
            # rather than fabricating one.
            obj.pose.pose.orientation.w = 1.0
            objects.append(obj)

            self.get_logger().info(
                f'{class_name} ({confidence:.2f}) -> map-frame point '
                f'({map_point[0]:.3f}, {map_point[1]:.3f}, {map_point[2]:.3f})m, '
                f'bbox_size=({bbox_size[0]:.3f}, {bbox_size[1]:.3f}, {bbox_size[2]:.3f})m, '
                f'inference={inference_ms:.0f}ms',
                throttle_duration_sec=1.0)

        array_msg = DetectedObjectArray()
        array_msg.header.frame_id = 'map'
        array_msg.header.stamp = rgb_msg.header.stamp
        array_msg.objects = objects
        self.detections_pub.publish(array_msg)

        self.markers_pub.publish(self._build_markers(objects, rgb_msg.header.stamp))

    def _build_markers(self, objects, stamp):
        """One CYLINDER marker per detection (sized/positioned from the
        DetectedObject itself) plus a text label above it showing class and
        confidence, so detections can be sanity-checked visually in RViz
        rather than only by reading logged numbers. A short lifetime lets
        stale markers auto-expire if detection stops, instead of leaving a
        ghost marker behind at the last-known pose."""
        marker_array = MarkerArray()
        for i, obj in enumerate(objects):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'detected_objects'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = obj.pose.pose
            # float(...): bounding_box_size is a float32[3] array field,
            # backed by numpy in rclpy -- indexing it yields numpy.float32,
            # which fails Vector3's strict PyFloat_Check, not a plain float.
            marker.scale.x = max(float(obj.bounding_box_size[0]), 0.01)
            marker.scale.y = max(float(obj.bounding_box_size[1]), 0.01)
            marker.scale.z = max(float(obj.bounding_box_size[2]), 0.01)
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = max(obj.confidence, 0.2)
            marker.lifetime = Duration(seconds=1.5).to_msg()
            marker_array.markers.append(marker)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = 'detected_object_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            # Built from obj's fields individually, not `label.pose =
            # obj.pose.pose`, which would alias the same Pose object rather
            # than copy it -- mutating label.pose.position below would then
            # silently mutate obj.pose.pose too.
            label.pose.position.x = obj.pose.pose.position.x
            label.pose.position.y = obj.pose.pose.position.y
            label.pose.position.z = obj.pose.pose.position.z + float(obj.bounding_box_size[2]) + 0.08
            label.scale.z = 0.08
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'{obj.class_label} ({obj.confidence:.2f})'
            label.lifetime = Duration(seconds=1.5).to_msg()
            marker_array.markers.append(label)

        return marker_array

    def _back_project(self, u, v, depth_image):
        """Back-project pixel (u, v) into a 3D point in the camera's optical
        frame, using a small averaged patch of depth values (not a single
        pixel sample) to stay robust to depth-image holes."""
        h, w = depth_image.shape
        u0, u1 = max(0, u - DEPTH_PATCH_RADIUS), min(w, u + DEPTH_PATCH_RADIUS + 1)
        v0, v1 = max(0, v - DEPTH_PATCH_RADIUS), min(h, v + DEPTH_PATCH_RADIUS + 1)
        patch = depth_image[v0:v1, u0:u1]

        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        z = float(np.mean(valid))

        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)

    def _estimate_bbox_size(self, x1, y1, x2, y2, z):
        """Estimate real-world object extent from the 2D box's pixel size
        at the measured depth (same back-projection math as _back_project,
        applied to the box's width/height instead of its center). This
        gives real X/Y extent, but the object's depth-axis extent (how far
        it extends away from the camera) can't be observed from a single
        RGB-D view -- we approximate it as the X/Y average rather than
        inventing an unmeasured number."""
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        width = (x2 - x1) * z / fx
        height = (y2 - y1) * z / fy
        depth_estimate = (width + height) / 2.0
        return [width, height, depth_estimate]

    def _transform_to_map(self, point, header):
        """Transform a camera-optical-frame point into the map frame, using
        the SOURCE IMAGE's timestamp (not "now") so the lookup uses the TF
        tree as it actually was when the image was captured -- using "now"
        here would silently introduce a pose offset proportional to how far
        the robot moved during detection processing."""
        point_stamped = PointStamped()
        point_stamped.header.frame_id = header.frame_id
        point_stamped.header.stamp = header.stamp
        point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = point

        try:
            transformed = self.tf_buffer.transform(
                point_stamped, 'map', timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF transform to map failed: {e}', throttle_duration_sec=1.0)
            return None

        return (transformed.point.x, transformed.point.y, transformed.point.z)


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
