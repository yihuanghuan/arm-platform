#!/usr/bin/env python3

import json
import sys
import time
from collections import deque
from threading import Thread

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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


def string_list_from_value(raw_value):
    if raw_value is None:
        return []
    if isinstance(raw_value, str):
        return [part for part in raw_value.replace(',', ' ').split() if part]
    return [str(item) for item in raw_value]


def int_list_from_value(raw_value):
    return [int(item) for item in string_list_from_value(raw_value)]


def vector_list_from_value(raw_value, expected_length):
    if raw_value is None:
        return []
    if isinstance(raw_value, str):
        text = raw_value.strip()
        if not text:
            return []
        if ';' in text:
            rows = []
            for chunk in text.split(';'):
                chunk = chunk.strip()
                if not chunk:
                    continue
                rows.append([
                    float(part) for part in chunk.replace(',', ' ').split()
                ])
        else:
            values = [float(part) for part in text.replace(',', ' ').split()]
            if not values:
                return []
            if len(values) % expected_length != 0:
                raise ValueError(
                    f'vector list must contain groups of {expected_length} values')
            rows = [
                values[index:index + expected_length]
                for index in range(0, len(values), expected_length)
            ]
    else:
        values = list(raw_value)
        if not values:
            return []
        if all(isinstance(item, (list, tuple)) for item in values):
            rows = [list(item) for item in values]
        else:
            if len(values) % expected_length != 0:
                raise ValueError(
                    f'vector list must contain groups of {expected_length} values')
            rows = [
                values[index:index + expected_length]
                for index in range(0, len(values), expected_length)
            ]
    for row in rows:
        if len(row) != expected_length:
            raise ValueError(f'each vector must contain {expected_length} values')
    return [[float(item) for item in row] for row in rows]


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


def matrix_to_xyz_quat(matrix):
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return [
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        float(quat[0]),
        float(quat[1]),
        float(quat[2]),
        float(quat[3]),
    ]


def rotation_angle_between(a_matrix, b_matrix):
    relative = a_matrix[:3, :3].T @ b_matrix[:3, :3]
    return float(Rotation.from_matrix(relative).magnitude())


def transform_delta(a_matrix, b_matrix):
    translation_delta = float(np.linalg.norm(a_matrix[:3, 3] - b_matrix[:3, 3]))
    rotation_delta = rotation_angle_between(a_matrix, b_matrix)
    return translation_delta, rotation_delta


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
        tag_ids_raw = self.declare_parameter('tag_ids', '').value
        detected_tag_frames_raw = self.declare_parameter(
            'detected_tag_frames', '').value
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
        self.ee_orientation_sync_wait_sec = float(self.declare_parameter(
            'ee_orientation_sync_wait_sec', 0.08).value)
        if self.ee_orientation_sync_wait_sec < 0.0:
            raise ValueError('ee_orientation_sync_wait_sec must be non-negative')
        self.tag_tf_mode = self.declare_parameter(
            'tag_tf_mode', 'stamped').value
        if self.tag_tf_mode not in ('stamped', 'latest'):
            raise ValueError('tag_tf_mode must be stamped or latest')
        self.max_tag_tf_age_sec = float(self.declare_parameter(
            'max_tag_tf_age_sec', 0.5).value)
        self.duplicate_translation_epsilon_m = float(self.declare_parameter(
            'duplicate_translation_epsilon_m', 1e-6).value)
        self.duplicate_rotation_epsilon_rad = float(self.declare_parameter(
            'duplicate_rotation_epsilon_rad', 1e-6).value)
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
        world_to_tag_xyzs = vector_list_from_value(
            self.declare_parameter('world_to_tag_xyzs', '').value, 3)
        world_to_tag_rpys = vector_list_from_value(
            self.declare_parameter('world_to_tag_rpys', '').value, 3)
        tag_ids = int_list_from_value(tag_ids_raw) or [self.tag_id]
        detected_tag_frames = (
            string_list_from_value(detected_tag_frames_raw)
            or [self.detected_tag_frame])
        if not world_to_tag_xyzs:
            world_to_tag_xyzs = [tag_xyz]
        if not world_to_tag_rpys:
            world_to_tag_rpys = [tag_rpy]
        if not (
                len(tag_ids)
                == len(detected_tag_frames)
                == len(world_to_tag_xyzs)
                == len(world_to_tag_rpys)):
            raise ValueError(
                'tag_ids, detected_tag_frames, world_to_tag_xyzs, and '
                'world_to_tag_rpys must have the same length')
        self.tag_configs = []
        self.tag_configs_by_id = {}
        for priority, (tag_id, frame, xyz, rpy) in enumerate(zip(
                tag_ids,
                detected_tag_frames,
                world_to_tag_xyzs,
                world_to_tag_rpys)):
            if tag_id in self.tag_configs_by_id:
                raise ValueError(f'duplicate tag id in tag_ids: {tag_id}')
            config = {
                'id': int(tag_id),
                'priority': priority,
                'frame': str(frame),
                'world_to_tag': make_transform_matrix(xyz, rpy),
                'world_to_tag_xyz': [float(item) for item in xyz],
                'world_to_tag_rpy': [float(item) for item in rpy],
                'latest_detection_stamp_ns': 0,
                'last_published_tag_tf_stamp_ns': 0,
                'last_published_camera_to_tag': None,
                'duplicate_tag_tf_drop_count': 0,
                'new_measurement_count': 0,
            }
            self.tag_configs.append(config)
            self.tag_configs_by_id[int(tag_id)] = config
        self.configured_tag_ids = [config['id'] for config in self.tag_configs]
        self.configured_tag_frames = [config['frame'] for config in self.tag_configs]
        self.primary_tag_config = self.tag_configs[0]

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # Receive TF on a dedicated node/executor.  Sharing the estimator's
        # executor can starve TF callbacks behind the 50 Hz measurement timer,
        # leaving the local buffer hundreds of milliseconds behind /tf.
        self.tf_listener_node = Node(
            'visual_ee_pose_tf_listener',
            use_global_arguments=False,
            enable_rosout=False)
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.tf_listener_node)
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
        self.last_published_tag_tf_stamp_ns = 0
        self.last_published_camera_to_tag = None
        self.last_debug_wall_time = 0.0
        self.last_valid = False
        self.last_reason = 'waiting_for_detection'
        self.last_ee_orientation_source = 'not_evaluated'
        self.last_ee_orientation_tf_stamp_delta_sec = None
        self.last_ee_orientation_lookup_errors = []
        self.visual_valid_true_count = 0
        self.visual_valid_false_count = 0
        self.visual_valid_toggle_count = 0
        self.duplicate_tag_tf_drop_count = 0
        self.old_or_coincident_tag_tf_drop_count = 0
        self.duplicate_same_stamp_same_value_count = 0
        self.same_stamp_changed_transform_count = 0
        self.new_stamp_new_transform_count = 0
        self.new_stamp_same_transform_count = 0
        self.new_measurement_count = 0
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
            'ee_orientation_tf_stamped_unavailable': 0,
            'ee_orientation_tf_latest_fallback_success': 0,
            'ee_orientation_tf_unavailable': 0,
            'detection_tf_timeout': 0,
            'detection_tf_superseded': 0,
        }

        self.create_timer(0.02, self.process_measurements)
        self.create_timer(0.1, self.timeout_check)

        self.get_logger().info(
            f'Publishing {self.pose_topic} as {self.world_frame}->{self.ee_frame} '
            f'from {self.camera_frame}->[{", ".join(self.configured_tag_frames)}]; '
            f'tag_tf_mode={self.tag_tf_mode}')

    def detections_callback(self, msg):
        self.message_count += 1
        detection_stamp_ns = stamp_to_ns(msg.header.stamp)
        if detection_stamp_ns <= 0:
            detection_stamp_ns = self.get_clock().now().nanoseconds
        self.latest_detection_stamp_ns = detection_stamp_ns

        detections = self.find_target_detections(msg)
        if not detections:
            now_ns = self.get_clock().now().nanoseconds
            if self.latest_target_detection_stamp_ns <= 0:
                self.publish_periodic_debug('target_not_detected')
                return
            target_age_sec = ns_to_sec(now_ns - self.latest_target_detection_stamp_ns)
            if target_age_sec > self.detection_timeout_sec:
                self.publish_invalid('target_not_detected_timeout', msg.header.stamp)
            return

        self.target_detection_count += len(detections)
        self.latest_target_detection_stamp_ns = detection_stamp_ns
        self.latest_target_detection_id = int(detections[0].id)
        for detection in detections:
            config = self.tag_configs_by_id.get(int(detection.id))
            if config is not None:
                config['latest_detection_stamp_ns'] = detection_stamp_ns
        if self.tag_tf_mode == 'latest':
            return
        for detection in detections:
            self.pending_detections.append(
                (detection_stamp_ns, msg.header.stamp, detection))

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

        active_configs = []
        for config in self.tag_configs:
            stamp_ns = config['latest_detection_stamp_ns']
            if stamp_ns <= 0:
                continue
            detection_age_sec = ns_to_sec(now_ns - stamp_ns)
            if detection_age_sec <= self.detection_timeout_sec:
                active_configs.append((config, detection_age_sec))
        if not active_configs:
            self.publish_valid(False)
            self.publish_periodic_debug('detection_timeout')
            return

        candidates = []
        duplicate_seen = False
        last_error = None
        for config, detection_age_sec in active_configs:
            try:
                camera_to_tag_tf = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    config['frame'],
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec))
                self.tf_counters['camera_tag_tf_success'] += 1
            except Exception as exc:
                self.record_tf_failure('camera_tag_tf', exc)
                last_error = f'{config["frame"]}: {exc}'
                continue

            tf_stamp = camera_to_tag_tf.header.stamp
            tf_stamp_ns = stamp_to_ns(tf_stamp)
            if tf_stamp_ns <= 0:
                self.tf_counters['camera_tag_tf_stale'] += 1
                last_error = f'{config["frame"]}: tag_tf_missing_stamp'
                continue
            tf_age_sec = ns_to_sec(now_ns - tf_stamp_ns)
            if tf_age_sec < -self.max_tag_tf_age_sec:
                self.tf_counters['camera_tag_tf_future_extrapolation'] += 1
                last_error = f'{config["frame"]}: tag_tf_future_age:{tf_age_sec:.6f}'
                continue
            if tf_age_sec > self.max_tag_tf_age_sec:
                self.tf_counters['camera_tag_tf_stale'] += 1
                last_error = f'{config["frame"]}: tag_tf_stale:{tf_age_sec:.6f}'
                continue

            camera_to_tag = transform_to_matrix(camera_to_tag_tf.transform)
            if tf_stamp_ns <= self.last_published_tag_tf_stamp_ns:
                duplicate_seen = True
                self.old_or_coincident_tag_tf_drop_count += 1
                continue
            duplicate_status = self.classify_camera_tag_measurement(
                config, tf_stamp_ns, camera_to_tag)
            if duplicate_status['is_duplicate']:
                duplicate_seen = True
                config['duplicate_tag_tf_drop_count'] += 1
                self.duplicate_tag_tf_drop_count += 1
                self.duplicate_same_stamp_same_value_count += 1
                continue
            candidates.append({
                'config': config,
                'camera_to_tag_tf': camera_to_tag_tf,
                'tf_stamp_ns': tf_stamp_ns,
                'tf_age_sec': tf_age_sec,
                'detection_age_sec': detection_age_sec,
                'duplicate_status': duplicate_status,
            })

        if not candidates:
            if duplicate_seen:
                self.last_reason = 'duplicate_tag_tf'
                self.publish_debug(
                    valid=self.last_valid,
                    reason='duplicate_tag_tf',
                    stamp=now.to_msg(),
                    extra={'tag_tf_mode': self.tag_tf_mode})
            else:
                self.publish_invalid(
                    f'tag_tf_lookup_failed_latest: {last_error or "no active tag tf"}',
                    now.to_msg())
            return

        newest_stamp_ns = max(item['tf_stamp_ns'] for item in candidates)
        selected = min(
            (
                item for item in candidates
                if item['tf_stamp_ns'] == newest_stamp_ns
            ),
            key=lambda item: item['config']['priority'])
        config = selected['config']
        camera_to_tag_tf = selected['camera_to_tag_tf']
        tf_stamp = camera_to_tag_tf.header.stamp

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
            config,
            extra={
                'tag_tf_mode': self.tag_tf_mode,
                'tag_tf_age_sec': selected['tf_age_sec'],
                'target_detection_age_sec': selected['detection_age_sec'],
                'tag_tf_stamp_sec': ns_to_sec(selected['tf_stamp_ns']),
                **selected['duplicate_status'],
            })

    def classify_camera_tag_measurement(self, config, tf_stamp_ns, camera_to_tag):
        previous_camera_to_tag = config['last_published_camera_to_tag']
        previous_stamp_ns = config['last_published_tag_tf_stamp_ns']
        if previous_camera_to_tag is None:
            return {
                'is_duplicate': False,
                'tag_tf_stamp_changed': True,
                'tag_tf_translation_delta_m': None,
                'tag_tf_rotation_delta_rad': None,
                'tag_tf_translation_changed': True,
                'tag_tf_rotation_changed': True,
                'tag_tf_change_class': 'first_measurement',
            }

        translation_delta, rotation_delta = transform_delta(
            previous_camera_to_tag,
            camera_to_tag)
        translation_changed = (
            translation_delta > self.duplicate_translation_epsilon_m)
        rotation_changed = (
            rotation_delta > self.duplicate_rotation_epsilon_rad)
        value_changed = translation_changed or rotation_changed
        stamp_changed = tf_stamp_ns != previous_stamp_ns

        if not stamp_changed and not value_changed:
            change_class = 'duplicate_same_stamp_same_value'
            is_duplicate = True
        elif not stamp_changed and value_changed:
            self.same_stamp_changed_transform_count += 1
            change_class = 'same_stamp_changed_transform'
            is_duplicate = False
        elif stamp_changed and value_changed:
            self.new_stamp_new_transform_count += 1
            change_class = 'new_stamp_new_transform'
            is_duplicate = False
        else:
            self.new_stamp_same_transform_count += 1
            change_class = 'new_stamp_same_transform'
            is_duplicate = False

        return {
            'is_duplicate': is_duplicate,
            'tag_tf_stamp_changed': stamp_changed,
            'tag_tf_translation_delta_m': translation_delta,
            'tag_tf_rotation_delta_rad': rotation_delta,
            'tag_tf_translation_changed': translation_changed,
            'tag_tf_rotation_changed': rotation_changed,
            'tag_tf_change_class': change_class,
        }

    def process_detection(self, stamp, detection):
        stamp_time = Time.from_msg(stamp)
        try:
            config = self.tag_configs_by_id[int(detection.id)]
            camera_to_tag_tf = self.tf_buffer.lookup_transform(
                self.camera_frame,
                config['frame'],
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
            config,
            extra={'tag_tf_mode': self.tag_tf_mode})
        return 'done'

    def publish_pose_from_transforms(
            self, camera_to_tag_tf, ee_to_camera_tf, stamp, tag_config, extra=None):
        camera_to_tag = transform_to_matrix(camera_to_tag_tf.transform)
        ee_to_camera = transform_to_matrix(ee_to_camera_tf.transform)
        world_to_tag = tag_config['world_to_tag']
        world_to_ee_full = (
            world_to_tag
            @ np.linalg.inv(camera_to_tag)
            @ np.linalg.inv(ee_to_camera)
        )
        world_to_ee_kinematic = self.estimate_position_with_kinematic_orientation(
            stamp, camera_to_tag, ee_to_camera, world_to_tag, world_to_ee_full)
        if world_to_ee_kinematic is None:
            self.last_reason = 'waiting_for_ee_orientation_sync'
            self.publish_debug(
                valid=self.last_valid,
                reason=self.last_reason,
                stamp=stamp,
                extra={
                    'target_id': tag_config['id'],
                    'selected_tag_id': tag_config['id'],
                    'selected_tag_frame': tag_config['frame'],
                })
            return
        if self.position_estimation_mode == 'full_pose':
            world_to_ee = world_to_ee_full
        else:
            world_to_ee = world_to_ee_kinematic

        if not np.all(np.isfinite(world_to_ee)):
            self.publish_invalid('non_finite_pose', stamp)
            return

        pose_msg = matrix_to_pose_stamped(world_to_ee, self.world_frame, stamp)
        self.pose_pub.publish(pose_msg)
        self.valid_pose_count += 1
        self.new_measurement_count += 1
        self.latest_valid_stamp_ns = stamp_to_ns(stamp)
        tag_tf_stamp_ns = stamp_to_ns(camera_to_tag_tf.header.stamp)
        if tag_tf_stamp_ns > 0:
            tag_config['last_published_tag_tf_stamp_ns'] = tag_tf_stamp_ns
            tag_config['last_published_camera_to_tag'] = camera_to_tag
            tag_config['new_measurement_count'] += 1
            self.last_published_tag_tf_stamp_ns = tag_tf_stamp_ns
            self.last_published_camera_to_tag = camera_to_tag
        self.last_reason = 'ok'
        self.publish_valid(True)
        debug_extra = {
            'target_id': tag_config['id'],
            'selected_tag_id': tag_config['id'],
            'selected_tag_frame': tag_config['frame'],
            'ee_position': [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ],
            'camera_to_tag_measured': matrix_to_xyz_quat(camera_to_tag),
            'ee_to_camera': matrix_to_xyz_quat(ee_to_camera),
            'world_to_tag': matrix_to_xyz_quat(world_to_tag),
            'world_to_ee_full': matrix_to_xyz_quat(world_to_ee_full),
            'world_to_ee_kinematic': matrix_to_xyz_quat(world_to_ee_kinematic),
            'world_to_ee_full_kinematic_position_delta_m': float(
                np.linalg.norm(
                    world_to_ee_full[:3, 3] - world_to_ee_kinematic[:3, 3])),
        }
        if extra:
            debug_extra.update(extra)
        self.publish_debug(
            valid=True,
            reason='ok',
            stamp=stamp,
            extra=debug_extra)

    def estimate_position_with_kinematic_orientation(
            self, stamp, camera_to_tag, ee_to_camera, world_to_tag,
            fallback_world_to_ee):
        orientation_matrix = self.lookup_ee_orientation_matrix(stamp)
        if (
                orientation_matrix is None
                and self.last_ee_orientation_source
                == 'waiting_for_stamped_tf'):
            return None
        if orientation_matrix is None:
            orientation_matrix = fallback_world_to_ee[:3, :3]

        world_to_ee = np.eye(4)
        world_to_ee[:3, :3] = orientation_matrix
        world_to_ee[:3, 3] = (
            world_to_tag[:3, 3]
            - orientation_matrix @ ee_to_camera[:3, :3] @ camera_to_tag[:3, 3]
            - orientation_matrix @ ee_to_camera[:3, 3]
        )
        return world_to_ee

    def lookup_ee_orientation_matrix(self, stamp):
        stamp_time = Time.from_msg(stamp)
        requested_stamp_ns = stamp_to_ns(stamp)
        now_ns = self.get_clock().now().nanoseconds
        measurement_age_sec = (
            ns_to_sec(now_ns - requested_stamp_ns)
            if now_ns > requested_stamp_ns > 0 else 0.0)
        wait_for_exact_tf = (
            measurement_age_sec < self.ee_orientation_sync_wait_sec)
        lookup_errors = []
        stamped_tf_pending = False
        for target_frame, source_frame in (
            (self.base_frame, self.ee_frame),
            (self.world_frame, self.ee_frame),
        ):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp_time)
                self.tf_counters['ee_orientation_tf_success'] += 1
                self.last_ee_orientation_source = (
                    f'{target_frame}_to_{source_frame}_stamped')
            except Exception as stamped_exc:
                lookup_errors.append(
                    f'{target_frame}->{source_frame} stamped: {stamped_exc}')
                self.tf_counters[
                    'ee_orientation_tf_stamped_unavailable'] += 1
                if target_frame == self.base_frame and wait_for_exact_tf:
                    stamped_tf_pending = True
                    continue
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time())
                    self.tf_counters['ee_orientation_tf_latest_fallback_success'] += 1
                    self.last_ee_orientation_source = (
                        f'{target_frame}_to_{source_frame}_latest')
                except Exception as latest_exc:
                    lookup_errors.append(
                        f'{target_frame}->{source_frame} latest: {latest_exc}')
                    self.tf_counters['ee_orientation_tf_unavailable'] += 1
                    continue
            tf_stamp_ns = stamp_to_ns(tf_msg.header.stamp)
            self.last_ee_orientation_tf_stamp_delta_sec = (
                ns_to_sec(tf_stamp_ns - requested_stamp_ns)
                if tf_stamp_ns > 0 and requested_stamp_ns > 0 else None)
            self.last_ee_orientation_lookup_errors = lookup_errors
            return transform_to_matrix(tf_msg.transform)[:3, :3]
        if stamped_tf_pending:
            self.last_ee_orientation_source = 'waiting_for_stamped_tf'
            self.last_ee_orientation_tf_stamp_delta_sec = None
            self.last_ee_orientation_lookup_errors = lookup_errors
            return None
        self.last_ee_orientation_source = 'full_pose_fallback'
        self.last_ee_orientation_tf_stamp_delta_sec = None
        self.last_ee_orientation_lookup_errors = lookup_errors
        return None

    def find_target_detections(self, msg):
        detections = []
        for detection in msg.detections:
            if (
                    detection.family == self.tag_family
                    and int(detection.id) in self.tag_configs_by_id):
                detections.append(detection)
        return detections

    def publish_invalid(self, reason, stamp):
        self.invalid_count += 1
        self.last_reason = reason
        self.publish_valid(False)
        self.publish_debug(valid=False, reason=reason, stamp=stamp)

    def publish_valid(self, valid):
        valid = bool(valid)
        if valid:
            self.visual_valid_true_count += 1
        else:
            self.visual_valid_false_count += 1
        if valid != self.last_valid:
            self.visual_valid_toggle_count += 1
        self.last_valid = valid
        self.valid_pub.publish(Bool(data=valid))

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
        target_age_sec = ns_to_sec(now_ns - self.latest_target_detection_stamp_ns)
        if target_age_sec > self.detection_timeout_sec:
            self.publish_valid(False)
            self.last_reason = 'detection_timeout'
            self.publish_periodic_debug('detection_timeout')
            return
        if self.latest_valid_stamp_ns <= 0:
            return
        valid_age_sec = ns_to_sec(now_ns - self.latest_valid_stamp_ns)
        if valid_age_sec > self.detection_timeout_sec:
            self.publish_valid(False)
            self.last_reason = 'visual_measurement_timeout'
            self.publish_periodic_debug('visual_measurement_timeout')

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
            'new_measurement_count': self.new_measurement_count,
            'duplicate_tag_tf_drop_count': self.duplicate_tag_tf_drop_count,
            'old_or_coincident_tag_tf_drop_count': (
                self.old_or_coincident_tag_tf_drop_count),
            'duplicate_same_stamp_same_value_count': (
                self.duplicate_same_stamp_same_value_count),
            'same_stamp_changed_transform_count': (
                self.same_stamp_changed_transform_count),
            'new_stamp_new_transform_count': self.new_stamp_new_transform_count,
            'new_stamp_same_transform_count': self.new_stamp_same_transform_count,
            'visual_valid_true_count': self.visual_valid_true_count,
            'visual_valid_false_count': self.visual_valid_false_count,
            'visual_valid_toggle_count': self.visual_valid_toggle_count,
            'stamp_sec': ns_to_sec(stamp_ns) if stamp_ns > 0 else None,
            'measurement_age_sec': age_sec,
            'world_frame': self.world_frame,
            'base_frame': self.base_frame,
            'ee_frame': self.ee_frame,
            'camera_frame': self.camera_frame,
            'detected_tag_frame': self.detected_tag_frame,
            'detected_tag_frames': list(self.configured_tag_frames),
            'position_estimation_mode': self.position_estimation_mode,
            'ee_orientation_sync_wait_sec': self.ee_orientation_sync_wait_sec,
            'ee_orientation_source': self.last_ee_orientation_source,
            'ee_orientation_tf_stamp_delta_sec': (
                self.last_ee_orientation_tf_stamp_delta_sec),
            'ee_orientation_lookup_errors': list(
                self.last_ee_orientation_lookup_errors),
            'tag_tf_mode': self.tag_tf_mode,
            'max_tag_tf_age_sec': self.max_tag_tf_age_sec,
            'duplicate_translation_epsilon_m': self.duplicate_translation_epsilon_m,
            'duplicate_rotation_epsilon_rad': self.duplicate_rotation_epsilon_rad,
            'tag_family': self.tag_family,
            'tag_id': self.tag_id,
            'tag_ids': list(self.configured_tag_ids),
            'latest_target_detection_stamp_sec': (
                ns_to_sec(self.latest_target_detection_stamp_ns)
                if self.latest_target_detection_stamp_ns > 0 else None),
            'last_published_tag_tf_stamp_sec': (
                ns_to_sec(self.last_published_tag_tf_stamp_ns)
                if self.last_published_tag_tf_stamp_ns > 0 else None),
            'tf_counters': dict(self.tf_counters),
            'per_tag_state': {
                str(config['id']): {
                    'frame': config['frame'],
                    'latest_detection_stamp_sec': (
                        ns_to_sec(config['latest_detection_stamp_ns'])
                        if config['latest_detection_stamp_ns'] > 0 else None),
                    'last_published_tag_tf_stamp_sec': (
                        ns_to_sec(config['last_published_tag_tf_stamp_ns'])
                        if config['last_published_tag_tf_stamp_ns'] > 0 else None),
                    'duplicate_tag_tf_drop_count': (
                        config['duplicate_tag_tf_drop_count']),
                    'new_measurement_count': config['new_measurement_count'],
                }
                for config in self.tag_configs
            },
        }
        if extra:
            payload.update(extra)
        self.debug_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(argv=None):
    _ = argv
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = VisualEePoseEstimator()
    except Exception as exc:
        rclpy.shutdown()
        raise SystemExit(str(exc))
    tf_executor = SingleThreadedExecutor()
    tf_executor.add_node(node.tf_listener_node)
    tf_thread = Thread(target=tf_executor.spin, daemon=True)
    tf_thread.start()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if rclpy.ok():
            raise
        node.get_logger().debug(f'Ignoring shutdown exception: {exc}')
    finally:
        executor.shutdown(timeout_sec=2.0)
        tf_executor.shutdown(timeout_sec=2.0)
        tf_thread.join(timeout=2.0)
        try:
            node.destroy_node()
            node.tf_listener_node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
