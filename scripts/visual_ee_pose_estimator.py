#!/usr/bin/env python3

import json
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from std_msgs.msg import String
import tf2_ros


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def ns_to_sec(stamp_ns):
    return float(stamp_ns) * 1e-9


def vector_parameter(node, name, default, expected_length):
    raw_value = node.declare_parameter(name, default).value
    if isinstance(raw_value, str):
        value = [float(part) for part in raw_value.replace(',', ' ').split()]
    else:
        value = list(raw_value)
    if len(value) != expected_length:
        raise ValueError(f'{name} must contain {expected_length} values')
    return [float(item) for item in value]


def make_transform_matrix(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def transform_to_matrix(transform):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]).as_matrix()
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def matrix_to_pose_stamped(matrix, frame_id, stamp):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(matrix[0, 3])
    msg.pose.position.y = float(matrix[1, 3])
    msg.pose.position.z = float(matrix[2, 3])
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


class VisualEePoseEstimator(Node):
    def __init__(self):
        super().__init__('visual_ee_pose_estimator')
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'link6').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'camera_color_optical_frame').value
        self.detected_tag_frame = self.declare_parameter(
            'detected_tag_frame', 'apriltag_36h11_00000').value
        self.tag_family = self.declare_parameter('tag_family', 'tag36h11').value
        self.tag_id = int(self.declare_parameter('tag_id', 0).value)
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/apriltag/detections').value
        self.pose_topic = self.declare_parameter(
            'pose_topic', '/visual_ee_pose').value
        self.valid_topic = self.declare_parameter(
            'valid_topic', '/visual_ee_pose_valid').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/visual_ee_pose_debug').value
        self.detection_timeout_sec = float(self.declare_parameter(
            'detection_timeout_sec', 0.5).value)
        self.tf_timeout_sec = float(self.declare_parameter(
            'tf_timeout_sec', 0.1).value)
        self.tag_tf_mode = self.declare_parameter(
            'tag_tf_mode', 'stamped').value
        if self.tag_tf_mode not in ('stamped', 'latest'):
            raise ValueError('tag_tf_mode must be stamped or latest')
        self.max_tag_tf_age_sec = float(self.declare_parameter(
            'max_tag_tf_age_sec', 0.5).value)
        self.debug_publish_period_sec = float(self.declare_parameter(
            'debug_publish_period_sec', 0.5).value)
        self.position_estimation_mode = self.declare_parameter(
            'position_estimation_mode', 'kinematic_orientation').value
        if self.position_estimation_mode not in ('kinematic_orientation', 'full_pose'):
            raise ValueError(
                'position_estimation_mode must be kinematic_orientation or full_pose')

        tag_xyz = vector_parameter(
            self, 'world_to_tag_xyz', '1.23042456 0.000976374 0.35065986', 3)
        tag_rpy = vector_parameter(
            self, 'world_to_tag_rpy', '-3.12204785 -1.56214388 3.12103003', 3)
        self.world_to_tag = make_transform_matrix(tag_xyz, tag_rpy)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.detections_callback,
            10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.valid_pub = self.create_publisher(Bool, self.valid_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.pending_detections = deque()

        self.message_count = 0
        self.target_detection_count = 0
        self.valid_pose_count = 0
        self.invalid_count = 0
        self.latest_detection_stamp_ns = 0
        self.latest_target_detection_stamp_ns = 0
        self.latest_target_detection_id = None
        self.latest_valid_stamp_ns = 0
        self.last_debug_wall_time = 0.0
        self.last_valid = False
        self.last_reason = 'waiting_for_detection'
        self.tf_counters = {
            'camera_tag_tf_success': 0,
            'camera_tag_tf_lookup_failed': 0,
            'camera_tag_tf_connectivity_failed': 0,
            'camera_tag_tf_future_extrapolation': 0,
            'camera_tag_tf_past_extrapolation': 0,
            'camera_tag_tf_stale': 0,
            'ee_camera_tf_success': 0,
            'ee_camera_tf_lookup_failed': 0,
            'ee_camera_tf_connectivity_failed': 0,
            'ee_camera_tf_future_extrapolation': 0,
            'ee_camera_tf_past_extrapolation': 0,
            'ee_camera_tf_latest_fallback_success': 0,
            'ee_orientation_tf_success': 0,
            'ee_orientation_tf_latest_fallback_success': 0,
            'detection_tf_timeout': 0,
            'detection_tf_superseded': 0,
        }

        self.create_timer(0.02, self.process_measurements)
        self.create_timer(0.1, self.timeout_check)

        self.get_logger().info(
            f'Publishing {self.pose_topic} as {self.world_frame}->{self.ee_frame} '
            f'from {self.camera_frame}->{self.detected_tag_frame}; '
            f'tag_tf_mode={self.tag_tf_mode}')

    def detections_callback(self, msg):
        self.message_count += 1
        detection_stamp_ns = stamp_to_ns(msg.header.stamp)
        if detection_stamp_ns <= 0:
            detection_stamp_ns = self.get_clock().now().nanoseconds
        self.latest_detection_stamp_ns = detection_stamp_ns

        detection = self.find_target_detection(msg)
        if detection is None:
            self.publish_invalid('target_not_detected', msg.header.stamp)
            return

        self.target_detection_count += 1
        self.latest_target_detection_stamp_ns = detection_stamp_ns
        self.latest_target_detection_id = detection.id
        if self.tag_tf_mode == 'latest':
            return
        self.pending_detections.append((detection_stamp_ns, msg.header.stamp, detection))

    def process_measurements(self):
        if self.tag_tf_mode == 'latest':
            self.process_latest_tag_tf()
        else:
            self.process_pending_detections()

    def process_pending_detections(self):
        while self.pending_detections:
            detection_stamp_ns, stamp, detection = self.pending_detections[0]
            now_ns = self.get_clock().now().nanoseconds
            age_sec = (
                ns_to_sec(now_ns - detection_stamp_ns)
                if now_ns > 0 and detection_stamp_ns > 0 else 0.0)
            if age_sec > self.detection_timeout_sec:
                self.pending_detections.popleft()
                self.tf_counters['detection_tf_timeout'] += 1
                self.publish_invalid('detection_tf_timeout', stamp)
                continue

            status = self.process_detection(stamp, detection)
            if status == 'waiting':
                if len(self.pending_detections) > 1:
                    self.pending_detections.popleft()
                    self.tf_counters['detection_tf_superseded'] += 1
                    self.publish_invalid('detection_tf_superseded', stamp)
                    continue
                return
            self.pending_detections.popleft()

    def process_latest_tag_tf(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self.latest_target_detection_stamp_ns <= 0:
            self.publish_periodic_debug('waiting_for_detection')
            return

        detection_age_sec = ns_to_sec(now_ns - self.latest_target_detection_stamp_ns)
        if detection_age_sec > self.detection_timeout_sec:
            if self.last_valid:
                self.valid_pub.publish(Bool(data=False))
            self.last_valid = False
            self.last_reason = 'detection_timeout'
            self.publish_periodic_debug('detection_timeout')
            return

        try:
            camera_to_tag_tf = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.detected_tag_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec))
            self.tf_counters['camera_tag_tf_success'] += 1
        except Exception as exc:
            self.record_tf_failure('camera_tag_tf', exc)
            self.publish_invalid(f'tag_tf_lookup_failed_latest: {exc}', now.to_msg())
            return

        tf_stamp = camera_to_tag_tf.header.stamp
        tf_stamp_ns = stamp_to_ns(tf_stamp)
        if tf_stamp_ns <= 0:
            self.tf_counters['camera_tag_tf_stale'] += 1
            self.publish_invalid('tag_tf_missing_stamp', now.to_msg())
            return
        tf_age_sec = ns_to_sec(now_ns - tf_stamp_ns)
        if tf_age_sec < -self.max_tag_tf_age_sec:
            self.tf_counters['camera_tag_tf_future_extrapolation'] += 1
            self.publish_invalid(f'tag_tf_future_age:{tf_age_sec:.6f}', tf_stamp)
            return
        if tf_age_sec > self.max_tag_tf_age_sec:
            self.tf_counters['camera_tag_tf_stale'] += 1
            self.publish_invalid(f'tag_tf_stale:{tf_age_sec:.6f}', tf_stamp)
            return

        try:
            ee_to_camera_tf = self.tf_buffer.lookup_transform(
                self.ee_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec))
            self.tf_counters['ee_camera_tf_success'] += 1
        except Exception as exc:
            self.record_tf_failure('ee_camera_tf', exc)
            self.publish_invalid(f'ee_camera_tf_lookup_failed_latest: {exc}', tf_stamp)
            return

        self.publish_pose_from_transforms(
            camera_to_tag_tf,
            ee_to_camera_tf,
            tf_stamp,
            self.latest_target_detection_id,
            extra={
                'tag_tf_mode': self.tag_tf_mode,
                'tag_tf_age_sec': tf_age_sec,
                'target_detection_age_sec': detection_age_sec,
            })

    def process_detection(self, stamp, detection):
        stamp_time = Time.from_msg(stamp)
        try:
            camera_to_tag_tf = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.detected_tag_frame,
                stamp_time,
                timeout=Duration(seconds=self.tf_timeout_sec))
            self.tf_counters['camera_tag_tf_success'] += 1
        except Exception as exc:
            category = self.record_tf_failure('camera_tag_tf', exc)
            if category == 'future_extrapolation':
                return 'waiting'
            self.publish_invalid(f'tag_tf_lookup_failed: {exc}', stamp)
            return 'done'

        try:
            ee_to_camera_tf = self.tf_buffer.lookup_transform(
                self.ee_frame,
                self.camera_frame,
                stamp_time,
                timeout=Duration(seconds=self.tf_timeout_sec))
            self.tf_counters['ee_camera_tf_success'] += 1
        except Exception as exc:
            self.record_tf_failure('ee_camera_tf', exc)
            try:
                ee_to_camera_tf = self.tf_buffer.lookup_transform(
                    self.ee_frame,
                    self.camera_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec))
                self.tf_counters['ee_camera_tf_latest_fallback_success'] += 1
            except Exception as fallback_exc:
                self.record_tf_failure('ee_camera_tf', fallback_exc)
                self.publish_invalid(
                    f'ee_camera_tf_lookup_failed: {exc}; latest: {fallback_exc}',
                    stamp)
                return 'done'

        self.publish_pose_from_transforms(
            camera_to_tag_tf,
            ee_to_camera_tf,
            stamp,
            detection.id,
            extra={'tag_tf_mode': self.tag_tf_mode})
        return 'done'

    def publish_pose_from_transforms(
            self, camera_to_tag_tf, ee_to_camera_tf, stamp, target_id, extra=None):
        camera_to_tag = transform_to_matrix(camera_to_tag_tf.transform)
        ee_to_camera = transform_to_matrix(ee_to_camera_tf.transform)
        world_to_ee_full = (
            self.world_to_tag
            @ np.linalg.inv(camera_to_tag)
            @ np.linalg.inv(ee_to_camera)
        )
        if self.position_estimation_mode == 'full_pose':
            world_to_ee = world_to_ee_full
        else:
            world_to_ee = self.estimate_position_with_kinematic_orientation(
                stamp, camera_to_tag, ee_to_camera, world_to_ee_full)

        if not np.all(np.isfinite(world_to_ee)):
            self.publish_invalid('non_finite_pose', stamp)
            return

        pose_msg = matrix_to_pose_stamped(world_to_ee, self.world_frame, stamp)
        self.pose_pub.publish(pose_msg)
        self.valid_pose_count += 1
        self.latest_valid_stamp_ns = stamp_to_ns(stamp)
        self.last_valid = True
        self.last_reason = 'ok'
        self.valid_pub.publish(Bool(data=True))
        debug_extra = {
            'target_id': target_id,
            'ee_position': [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ],
        }
        if extra:
            debug_extra.update(extra)
        self.publish_debug(
            valid=True,
            reason='ok',
            stamp=stamp,
            extra=debug_extra)

    def estimate_position_with_kinematic_orientation(
            self, stamp, camera_to_tag, ee_to_camera, fallback_world_to_ee):
        orientation_matrix = self.lookup_ee_orientation_matrix(stamp)
        if orientation_matrix is None:
            orientation_matrix = fallback_world_to_ee[:3, :3]

        world_to_ee = np.eye(4)
        world_to_ee[:3, :3] = orientation_matrix
        world_to_ee[:3, 3] = (
            self.world_to_tag[:3, 3]
            - orientation_matrix @ ee_to_camera[:3, :3] @ camera_to_tag[:3, 3]
            - orientation_matrix @ ee_to_camera[:3, 3]
        )
        return world_to_ee

    def lookup_ee_orientation_matrix(self, stamp):
        stamp_time = Time.from_msg(stamp)
        for target_frame, source_frame in (
            (self.world_frame, self.ee_frame),
            (self.base_frame, self.ee_frame),
        ):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp_time,
                    timeout=Duration(seconds=self.tf_timeout_sec))
                self.tf_counters['ee_orientation_tf_success'] += 1
            except Exception:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=self.tf_timeout_sec))
                    self.tf_counters['ee_orientation_tf_latest_fallback_success'] += 1
                except Exception:
                    continue
            return transform_to_matrix(tf_msg.transform)[:3, :3]
        return None

    def find_target_detection(self, msg):
        for detection in msg.detections:
            if detection.family == self.tag_family and detection.id == self.tag_id:
                return detection
        return None

    def publish_invalid(self, reason, stamp):
        self.invalid_count += 1
        self.last_valid = False
        self.last_reason = reason
        self.valid_pub.publish(Bool(data=False))
        self.publish_debug(valid=False, reason=reason, stamp=stamp)

    def record_tf_failure(self, prefix, exc):
        text = str(exc).lower()
        if 'extrapolation into the future' in text:
            category = 'future_extrapolation'
        elif 'extrapolation into the past' in text:
            category = 'past_extrapolation'
        elif 'could not find a connection' in text or 'connectivity' in text:
            category = 'connectivity_failed'
        else:
            category = 'lookup_failed'
        key = f'{prefix}_{category}'
        if key in self.tf_counters:
            self.tf_counters[key] += 1
        return category

    def timeout_check(self):
        latest_ns = max(self.latest_detection_stamp_ns, self.latest_valid_stamp_ns)
        now_ns = self.get_clock().now().nanoseconds
        if latest_ns <= 0 or now_ns <= 0:
            self.publish_periodic_debug('waiting_for_detection')
            return
        age_sec = ns_to_sec(now_ns - latest_ns)
        if age_sec > self.detection_timeout_sec:
            if self.last_valid:
                self.valid_pub.publish(Bool(data=False))
            self.last_valid = False
            self.last_reason = 'detection_timeout'
            self.publish_periodic_debug('detection_timeout')

    def publish_periodic_debug(self, reason):
        wall_now = time.monotonic()
        if wall_now - self.last_debug_wall_time < self.debug_publish_period_sec:
            return
        self.publish_debug(valid=False, reason=reason, stamp=None)

    def publish_debug(self, valid, reason, stamp, extra=None):
        wall_now = time.monotonic()
        self.last_debug_wall_time = wall_now
        stamp_ns = stamp_to_ns(stamp) if stamp is not None else 0
        now_ns = self.get_clock().now().nanoseconds
        age_sec = ns_to_sec(now_ns - stamp_ns) if stamp_ns > 0 and now_ns > 0 else None
        payload = {
            'valid': bool(valid),
            'reason': reason,
            'message_count': self.message_count,
            'target_detection_count': self.target_detection_count,
            'valid_pose_count': self.valid_pose_count,
            'invalid_count': self.invalid_count,
            'stamp_sec': ns_to_sec(stamp_ns) if stamp_ns > 0 else None,
            'measurement_age_sec': age_sec,
            'world_frame': self.world_frame,
            'base_frame': self.base_frame,
            'ee_frame': self.ee_frame,
            'camera_frame': self.camera_frame,
            'detected_tag_frame': self.detected_tag_frame,
            'position_estimation_mode': self.position_estimation_mode,
            'tag_tf_mode': self.tag_tf_mode,
            'max_tag_tf_age_sec': self.max_tag_tf_age_sec,
            'tag_family': self.tag_family,
            'tag_id': self.tag_id,
            'latest_target_detection_stamp_sec': (
                ns_to_sec(self.latest_target_detection_stamp_ns)
                if self.latest_target_detection_stamp_ns > 0 else None),
            'tf_counters': dict(self.tf_counters),
        }
        if extra:
            payload.update(extra)
        self.debug_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(argv=None):
    _ = argv
    rclpy.init()
    try:
        node = VisualEePoseEstimator()
    except Exception as exc:
        rclpy.shutdown()
        raise SystemExit(str(exc))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if rclpy.ok():
            raise
        node.get_logger().debug(f'Ignoring shutdown exception: {exc}')
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
