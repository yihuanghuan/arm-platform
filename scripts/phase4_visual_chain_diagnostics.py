#!/usr/bin/env python3

import argparse
import csv
import json
import os
import sys
import time
from collections import deque

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
import tf2_ros


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def ns_to_sec(value):
    return float(value) * 1e-9


def format_float(value):
    if value is None:
        return ''
    return f'{float(value):.9f}'


def format_bool(value):
    if value is None:
        return ''
    return str(bool(value)).lower()


class Phase4VisualChainDiagnostics(Node):
    def __init__(self, args):
        super().__init__(
            'phase4_visual_chain_diagnostics',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.started_wall = time.monotonic()
        self.window_sec = args.window_sec

        self.image_times = deque()
        self.camera_info_times = deque()
        self.apriltag_msg_times = deque()
        self.target_detection_times = deque()
        self.tag_tf_update_times = deque()
        self.visual_pose_times = deque()
        self.clock_times = deque()

        self.image_count = 0
        self.camera_info_count = 0
        self.apriltag_msg_count = 0
        self.target_detection_count = 0
        self.visual_pose_count = 0
        self.clock_count = 0

        self.last_image_stamp_ns = 0
        self.last_camera_info_stamp_ns = 0
        self.last_apriltag_stamp_ns = 0
        self.last_target_detection_stamp_ns = 0
        self.last_visual_pose_stamp_ns = 0
        self.last_clock_stamp_ns = 0
        self.last_tag_tf_stamp_ns = 0
        self.latest_visual_valid = None
        self.latest_debug_reason = ''
        self.latest_debug_payload = {}
        self.latest_tag_tf_error = ''
        self.latest_tag_tf_available = False
        self.latest_tag_tf_age_sec = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            AprilTagDetectionArray,
            args.detections_topic,
            self.detections_callback,
            20)
        self.create_subscription(
            PoseStamped,
            args.visual_pose_topic,
            self.visual_pose_callback,
            20)
        self.create_subscription(
            Bool,
            args.visual_valid_topic,
            self.visual_valid_callback,
            20)
        self.create_subscription(
            String,
            args.visual_debug_topic,
            self.visual_debug_callback,
            20)
        self.create_subscription(
            Clock,
            args.clock_topic,
            self.clock_callback,
            qos_profile_sensor_data)

        output_dir = os.path.dirname(os.path.abspath(args.output_csv))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        self.output_handle = open(args.output_csv, 'w', newline='', encoding='utf-8')
        self.fieldnames = [
            'wall_time_sec',
            'sim_time_sec',
            'image_rate_hz',
            'camera_info_rate_hz',
            'apriltag_msg_rate_hz',
            'target_detection_rate_hz',
            'tag_tf_update_rate_hz',
            'visual_pose_rate_hz',
            'clock_rate_hz',
            'image_count',
            'camera_info_count',
            'apriltag_msg_count',
            'target_detection_count',
            'visual_pose_count',
            'clock_count',
            'image_age_sec',
            'camera_info_age_sec',
            'apriltag_msg_age_sec',
            'target_detection_age_sec',
            'tag_tf_available',
            'tag_tf_age_sec',
            'tag_tf_error',
            'visual_pose_age_sec',
            'visual_valid',
            'visual_debug_reason',
            'visual_debug_payload',
        ]
        self.writer = csv.DictWriter(self.output_handle, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.create_timer(1.0 / args.sample_hz, self.sample_once)

        self.get_logger().info(
            f'Writing phase 4 visual chain diagnostics to {args.output_csv}')

    def destroy_node(self):
        try:
            self.output_handle.flush()
            self.output_handle.close()
        finally:
            super().destroy_node()

    def append_event(self, queue):
        now = time.monotonic()
        queue.append(now)
        self.prune_queue(queue, now)

    def prune_queue(self, queue, now=None):
        if now is None:
            now = time.monotonic()
        cutoff = now - self.window_sec
        while queue and queue[0] < cutoff:
            queue.popleft()

    def rate(self, queue):
        now = time.monotonic()
        self.prune_queue(queue, now)
        if len(queue) < 2:
            return 0.0
        elapsed = queue[-1] - queue[0]
        return (len(queue) - 1) / elapsed if elapsed > 0.0 else 0.0

    def age_sec(self, stamp_ns, now_ns):
        if stamp_ns <= 0 or now_ns <= 0:
            return None
        return ns_to_sec(now_ns - stamp_ns)

    def image_callback(self, msg):
        self.image_count += 1
        self.last_image_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.append_event(self.image_times)

    def camera_info_callback(self, msg):
        self.camera_info_count += 1
        self.last_camera_info_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.append_event(self.camera_info_times)

    def detections_callback(self, msg):
        self.apriltag_msg_count += 1
        self.last_apriltag_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.append_event(self.apriltag_msg_times)
        target_seen = any(
            detection.family == self.args.tag_family and detection.id == self.args.tag_id
            for detection in msg.detections)
        if target_seen:
            self.target_detection_count += 1
            self.last_target_detection_stamp_ns = self.last_apriltag_stamp_ns
            self.append_event(self.target_detection_times)

    def visual_pose_callback(self, msg):
        self.visual_pose_count += 1
        self.last_visual_pose_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.append_event(self.visual_pose_times)

    def visual_valid_callback(self, msg):
        self.latest_visual_valid = bool(msg.data)

    def visual_debug_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_debug_reason = 'debug_json_decode_failed'
            self.latest_debug_payload = {'raw': msg.data}
            return
        self.latest_debug_payload = payload
        self.latest_debug_reason = str(payload.get('reason', ''))

    def clock_callback(self, msg):
        self.clock_count += 1
        self.last_clock_stamp_ns = stamp_to_ns(msg.clock)
        self.append_event(self.clock_times)

    def update_tag_tf_status(self, now_ns):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.camera_frame,
                self.args.detected_tag_frame,
                Time(),
                timeout=Duration(seconds=self.args.tf_timeout_sec))
        except Exception as exc:
            self.latest_tag_tf_available = False
            self.latest_tag_tf_age_sec = None
            self.latest_tag_tf_error = str(exc)
            return

        stamp_ns = stamp_to_ns(transform.header.stamp)
        if stamp_ns > 0 and stamp_ns != self.last_tag_tf_stamp_ns:
            self.last_tag_tf_stamp_ns = stamp_ns
            self.append_event(self.tag_tf_update_times)
        self.latest_tag_tf_available = True
        self.latest_tag_tf_age_sec = self.age_sec(stamp_ns, now_ns)
        self.latest_tag_tf_error = ''

    def sample_once(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        self.update_tag_tf_status(now_ns)
        row = {
            'wall_time_sec': format_float(time.monotonic() - self.started_wall),
            'sim_time_sec': format_float(ns_to_sec(now_ns) if now_ns > 0 else None),
            'image_rate_hz': format_float(self.rate(self.image_times)),
            'camera_info_rate_hz': format_float(self.rate(self.camera_info_times)),
            'apriltag_msg_rate_hz': format_float(self.rate(self.apriltag_msg_times)),
            'target_detection_rate_hz': format_float(self.rate(self.target_detection_times)),
            'tag_tf_update_rate_hz': format_float(self.rate(self.tag_tf_update_times)),
            'visual_pose_rate_hz': format_float(self.rate(self.visual_pose_times)),
            'clock_rate_hz': format_float(self.rate(self.clock_times)),
            'image_count': self.image_count,
            'camera_info_count': self.camera_info_count,
            'apriltag_msg_count': self.apriltag_msg_count,
            'target_detection_count': self.target_detection_count,
            'visual_pose_count': self.visual_pose_count,
            'clock_count': self.clock_count,
            'image_age_sec': format_float(self.age_sec(self.last_image_stamp_ns, now_ns)),
            'camera_info_age_sec': format_float(
                self.age_sec(self.last_camera_info_stamp_ns, now_ns)),
            'apriltag_msg_age_sec': format_float(
                self.age_sec(self.last_apriltag_stamp_ns, now_ns)),
            'target_detection_age_sec': format_float(
                self.age_sec(self.last_target_detection_stamp_ns, now_ns)),
            'tag_tf_available': format_bool(self.latest_tag_tf_available),
            'tag_tf_age_sec': format_float(self.latest_tag_tf_age_sec),
            'tag_tf_error': self.latest_tag_tf_error,
            'visual_pose_age_sec': format_float(
                self.age_sec(self.last_visual_pose_stamp_ns, now_ns)),
            'visual_valid': format_bool(self.latest_visual_valid),
            'visual_debug_reason': self.latest_debug_reason,
            'visual_debug_payload': json.dumps(
                self.latest_debug_payload,
                sort_keys=True,
                separators=(',', ':')),
        }
        self.writer.writerow(row)
        self.output_handle.flush()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Record phase 4 visual-chain health while moving-base replay runs.')
    parser.add_argument('--output-csv', default='/tmp/phase4_visual_chain_diagnostics.csv')
    parser.add_argument('--duration-sec', type=float, default=0.0)
    parser.add_argument('--sample-hz', type=float, default=2.0)
    parser.add_argument('--window-sec', type=float, default=2.0)
    parser.add_argument('--image-topic', default='/d435i/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/d435i/color/camera_info')
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--visual-pose-topic', default='/visual_ee_pose')
    parser.add_argument('--visual-valid-topic', default='/visual_ee_pose_valid')
    parser.add_argument('--visual-debug-topic', default='/visual_ee_pose_debug')
    parser.add_argument('--clock-topic', default='/clock')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--detected-tag-frame', default='apriltag_36h11_00000')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--tf-timeout-sec', type=float, default=0.02)
    parser.add_argument('--use-sim-time', action='store_true')
    args, _ = parser.parse_known_args(argv)
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')
    if args.window_sec <= 0.0:
        raise SystemExit('--window-sec must be positive')
    if args.duration_sec < 0.0:
        raise SystemExit('--duration-sec must be non-negative')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = Phase4VisualChainDiagnostics(args)
    deadline = None
    if args.duration_sec > 0.0:
        deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info(f'Diagnostics CSV: {args.output_csv}')
        except Exception:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
