#!/usr/bin/env python3
"""Track red conveyor objects in RGB images and localize them with aligned depth."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class RgbdObjectTracker(Node):
    def __init__(self):
        super().__init__('rgbd_object_tracker')
        self.declare_parameter('color_topic', '/camera_yolo/color/image_raw')
        self.declare_parameter('depth_topic', '/camera_yolo/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera_yolo/depth/camera_info')
        # CameraInfo's frame is authoritative: Gazebo publishes both RGB-D
        # products using the sensor's configured optical frame.
        self.declare_parameter('optical_frame', '')
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('min_contour_area', 120.0)
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('object_depth', 0.045)
        self.declare_parameter('prediction_horizon', 0.0)
        self.declare_parameter('velocity_filter', 0.35)
        self.declare_parameter('conveyor_length', 0.5)
        self.declare_parameter('conveyor_width', 0.16)
        self.declare_parameter('conveyor_height', 0.08)
        self.declare_parameter('conveyor_center_x', 0.32)
        self.declare_parameter('conveyor_center_y', 0.14)
        self.declare_parameter('pick_zone_length', 0.05)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.color = None
        self.depth = None
        self.color_stamp = None
        self.depth_stamp = None
        self.last_target = None
        self.last_stamp = None
        self.velocity = np.zeros(3, dtype=float)
        self.color_sub = self.create_subscription(
            Image, self.get_parameter('color_topic').value, self.on_color, 10)
        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.on_depth, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, 10)
        self.objects_pub = self.create_publisher(PoseArray, '/perception/objects', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/perception/target', 10)
        self.get_logger().info('RGB-D tracker waiting for images and CameraInfo')

    def on_camera_info(self, message):
        self.camera_info = message

    def on_color(self, message):
        try:
            self.color = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge reports encoding and storage errors here
            self.get_logger().warning(f'Ignoring invalid color frame: {exc}')
            return
        self.color_stamp = message.header.stamp
        self.process_pair()

    def on_depth(self, message):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Ignoring invalid depth frame: {exc}')
            return
        self.depth_stamp = message.header.stamp
        self.process_pair()

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def process_pair(self):
        if (self.color is None or self.depth is None or self.camera_info is None
                or self.color_stamp is None or self.depth_stamp is None):
            return
        color_time = self.stamp_seconds(self.color_stamp)
        depth_time = self.stamp_seconds(self.depth_stamp)
        if abs(color_time - depth_time) > 0.15:
            return
        if self.depth.shape[:2] != self.color.shape[:2]:
            self.get_logger().warning(
                'Color and depth dimensions differ; waiting for aligned depth')
            return

        hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
        red_low = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        red_high = cv2.inRange(hsv, np.array([170, 100, 50]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(red_low, red_high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        info = self.camera_info
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return

        poses = []
        candidates = []
        min_area = float(self.get_parameter('min_contour_area').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                continue
            moments = cv2.moments(contour)
            if moments['m00'] == 0:
                continue
            u = int(moments['m10'] / moments['m00'])
            v = int(moments['m01'] / moments['m00'])
            radius = max(2, min(8, int(np.sqrt(cv2.contourArea(contour) / np.pi) / 5)))
            roi = self.depth[max(0, v-radius):v+radius+1, max(0, u-radius):u+radius+1]
            depth = roi.astype(np.float32)
            if self.depth.dtype == np.uint16:
                depth *= 0.001
            valid = depth[np.isfinite(depth) & (depth > 0.05) & (depth < max_depth)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))
            z += 0.5 * float(self.get_parameter('object_depth').value)
            source = PointStamped()
            source.header.stamp = self.color_stamp
            source.header.frame_id = (
                info.header.frame_id
                or self.get_parameter('optical_frame').value
                or 'camera_yolo_color_optical_frame')
            source.point.x = (u - cx) * z / fx
            source.point.y = (v - cy) * z / fy
            source.point.z = z
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter('planning_frame').value,
                    source.header.frame_id,
                    Time.from_msg(self.color_stamp),
                    timeout=Duration(seconds=0.1))
                point = do_transform_point(source, transform).point
            except TransformException as exc:
                self.get_logger().debug(f'Waiting for camera transform: {exc}')
                return
            conveyor_length = float(self.get_parameter('conveyor_length').value)
            conveyor_width = float(self.get_parameter('conveyor_width').value)
            conveyor_height = float(self.get_parameter('conveyor_height').value)
            center_x = float(self.get_parameter('conveyor_center_x').value)
            center_y = float(self.get_parameter('conveyor_center_y').value)
            x_bounds = [center_x - conveyor_length / 2.0 - 0.03,
                        center_x - conveyor_length / 2.0
                        + float(self.get_parameter('pick_zone_length').value)]
            y_bounds = [center_y - conveyor_width / 2.0,
                        center_y + conveyor_width / 2.0]
            z_bounds = [conveyor_height + 0.02, conveyor_height + 0.55]
            if not (x_bounds[0] <= point.x <= x_bounds[1]
                    and y_bounds[0] <= point.y <= y_bounds[1]
                    and z_bounds[0] <= point.z <= z_bounds[1]):
                continue
            pose = Pose()
            pose.position = point
            pose.orientation.w = 1.0
            poses.append(pose)
            candidates.append((cv2.contourArea(contour), np.array([point.x, point.y, point.z])))

        array = PoseArray()
        array.header.stamp = self.color_stamp
        array.header.frame_id = self.get_parameter('planning_frame').value
        array.poses = poses
        self.objects_pub.publish(array)
        if not candidates:
            return

        now = color_time
        if self.last_target is None:
            selected = max(candidates, key=lambda item: item[0])[1]
        else:
            selected = min(candidates, key=lambda item: np.linalg.norm(item[1] - self.last_target))[1]
            dt = now - self.last_stamp if self.last_stamp is not None else 0.0
            if 0.02 < dt < 1.0:
                measured = (selected - self.last_target) / dt
                alpha = float(self.get_parameter('velocity_filter').value)
                self.velocity = alpha * measured + (1.0 - alpha) * self.velocity
        self.last_target = selected
        self.last_stamp = now

        horizon = float(self.get_parameter('prediction_horizon').value)
        target = PoseStamped()
        target.header = array.header
        target.pose.position.x = float(selected[0] + self.velocity[0] * horizon)
        target.pose.position.y = float(selected[1] + self.velocity[1] * horizon)
        target.pose.position.z = float(selected[2] + self.velocity[2] * horizon)
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdObjectTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
