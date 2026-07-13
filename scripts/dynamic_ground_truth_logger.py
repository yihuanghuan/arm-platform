#!/usr/bin/env python3

import argparse
import csv
from collections import deque
import math
import os
import sys
import time

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from gazebo_msgs.srv import GetEntityState
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf2_ros


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def ns_to_sec(stamp_ns):
    return float(stamp_ns) * 1e-9


def pose_to_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
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


def rpy_to_matrix(rpy):
    return Rotation.from_euler('xyz', rpy).as_matrix()


def make_pose_matrix(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(rpy)
    matrix[:3, 3] = xyz
    return matrix


def rotation_error_rad(a_matrix, b_matrix):
    delta = a_matrix[:3, :3] @ b_matrix[:3, :3].T
    return Rotation.from_matrix(delta).magnitude()


def serialize_vector(values):
    if values is None:
        return ''
    return ';'.join(f'{value:.9g}' for value in values)


class TransformBuffer:
    def __init__(self, buffer_sec):
        self.buffer_sec = buffer_sec
        self.samples = deque()

    def append(self, stamp_ns, matrix):
        if stamp_ns <= 0:
            return
        if self.samples and stamp_ns < self.samples[-1][0]:
            return
        self.samples.append((stamp_ns, matrix.copy()))
        min_stamp = stamp_ns - int(self.buffer_sec * 1e9)
        while self.samples and self.samples[0][0] < min_stamp:
            self.samples.popleft()

    def latest(self):
        if not self.samples:
            return None
        return self.samples[-1]

    def frequency_hz(self):
        if len(self.samples) < 2:
            return 0.0
        elapsed = ns_to_sec(self.samples[-1][0] - self.samples[0][0])
        return (len(self.samples) - 1) / elapsed if elapsed > 0.0 else 0.0

    def interpolate(self, target_ns):
        if len(self.samples) < 2:
            return None, None
        if target_ns < self.samples[0][0] or target_ns > self.samples[-1][0]:
            return None, None

        previous = self.samples[0]
        for current in self.samples:
            if current[0] == target_ns:
                return current[1].copy(), 0.0
            if current[0] > target_ns:
                dt = min(abs(target_ns - previous[0]), abs(current[0] - target_ns))
                return interpolate_matrix(previous, current, target_ns), ns_to_sec(dt)
            previous = current
        return None, None


def interpolate_matrix(sample_a, sample_b, target_ns):
    stamp_a, matrix_a = sample_a
    stamp_b, matrix_b = sample_b
    if stamp_a == stamp_b:
        return matrix_a.copy()

    ratio = (target_ns - stamp_a) / float(stamp_b - stamp_a)
    matrix = np.eye(4)
    matrix[:3, 3] = (
        matrix_a[:3, 3] * (1.0 - ratio) + matrix_b[:3, 3] * ratio
    )
    slerp = Slerp(
        [0.0, 1.0],
        Rotation.from_matrix([matrix_a[:3, :3], matrix_b[:3, :3]]))
    matrix[:3, :3] = slerp([ratio]).as_matrix()[0]
    return matrix


class DynamicGroundTruthLogger(Node):
    def __init__(self, args):
        super().__init__(
            'dynamic_ground_truth_logger',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])
        self.args = args
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=args.buffer_sec + 2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.entity_client = self.create_client(GetEntityState, args.entity_state_service)

        self.gt_buffers = {
            'base': TransformBuffer(args.buffer_sec),
            'link6': TransformBuffer(args.buffer_sec),
            'camera': TransformBuffer(args.buffer_sec),
            'tag': TransformBuffer(args.buffer_sec),
        }
        self.world_to_tag = None
        self.samples = []
        self.message_count = 0
        self.detected_count = 0
        self.invalid_count = 0
        self.gt_timer_count = 0
        self.first_gt_wall_time = None
        self.last_gt_wall_time = None

        self.latest_joint_state = None
        self.latest_velocity_command = None
        self.joint_state_samples = deque(maxlen=2000)
        self.velocity_command_samples = deque(maxlen=2000)
        self.pending_detection_stamps = deque()

        self.create_subscription(
            AprilTagDetectionArray,
            args.detections_topic,
            self.detections_callback,
            10)
        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self.joint_state_callback,
            50)
        self.create_subscription(
            Float64MultiArray,
            args.velocity_command_topic,
            self.velocity_command_callback,
            50)
        self.create_timer(
            1.0 / args.gt_hz,
            self.sample_ground_truth,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def wait_for_inputs(self):
        if not self.entity_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f'Gazebo entity state service not available: {self.args.entity_state_service}')

        deadline = time.monotonic() + self.args.service_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > 0:
                break
        if self.get_clock().now().nanoseconds <= 0:
            self.get_logger().warn('Sim time has not advanced yet; waiting for TF may fail early')

        request = GetEntityState.Request()
        request.name = self.args.tag_model_name
        request.reference_frame = self.args.world_frame
        future = self.entity_client.call_async(request)
        deadline = time.monotonic() + self.args.service_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError('Timed out waiting for Gazebo tag entity state')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'Gazebo entity state failed for {self.args.tag_model_name}')

        world_to_tag_model = pose_to_matrix(response.state.pose)
        model_to_tag = make_pose_matrix(
            self.args.tag_frame_xyz_in_model,
            self.args.tag_frame_rpy_in_model)
        self.world_to_tag = world_to_tag_model @ model_to_tag

    def joint_state_callback(self, msg):
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        values = list(msg.position)
        self.latest_joint_state = (stamp_ns, values)
        self.joint_state_samples.append(self.latest_joint_state)

    def velocity_command_callback(self, msg):
        stamp_ns = self.get_clock().now().nanoseconds
        values = list(msg.data)
        self.latest_velocity_command = (stamp_ns, values)
        self.velocity_command_samples.append(self.latest_velocity_command)

    def sample_ground_truth(self):
        if self.world_to_tag is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return

        frame_specs = {
            'base': self.args.base_frame,
            'link6': self.args.ee_frame,
            'camera': self.args.camera_frame,
        }
        for key, frame in frame_specs.items():
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.args.world_frame,
                    frame,
                    Time())
            except Exception as exc:
                self.get_logger().warn(
                    f'GT TF lookup failed for {self.args.world_frame}->{frame}: {exc}',
                    throttle_duration_sec=2.0)
                return

            stamp_ns = stamp_to_ns(tf_msg.header.stamp)
            if stamp_ns <= 0:
                stamp_ns = now_ns
            self.gt_buffers[key].append(stamp_ns, transform_to_matrix(tf_msg.transform))

        self.gt_buffers['tag'].append(now_ns, self.world_to_tag)
        self.gt_timer_count += 1
        wall_now = time.monotonic()
        self.first_gt_wall_time = self.first_gt_wall_time or wall_now
        self.last_gt_wall_time = wall_now
        self.process_pending_detections()

    def process_pending_detections(self):
        latest_camera = self.gt_buffers['camera'].latest()
        if latest_camera is None:
            return
        latest_stamp_ns = latest_camera[0]
        min_stamp_ns = (
            self.gt_buffers['camera'].samples[0][0]
            if self.gt_buffers['camera'].samples else latest_stamp_ns)

        while self.pending_detection_stamps:
            detection_stamp_ns = self.pending_detection_stamps[0]
            if detection_stamp_ns > latest_stamp_ns:
                return
            self.pending_detection_stamps.popleft()
            if detection_stamp_ns < min_stamp_ns:
                self.record_invalid_detection(detection_stamp_ns, 'detection_older_than_gt_buffer')
                continue
            self.record_detection(detection_stamp_ns)

    def detections_callback(self, msg):
        self.message_count += 1
        detection_stamp_ns = stamp_to_ns(msg.header.stamp)
        if detection_stamp_ns <= 0:
            detection_stamp_ns = self.get_clock().now().nanoseconds

        has_target = any(
            detection.family == self.args.tag_family and detection.id == self.args.tag_id
            for detection in msg.detections
        )
        if has_target:
            self.detected_count += 1
            self.pending_detection_stamps.append(detection_stamp_ns)

    def make_base_row(self, detection_stamp_ns):
        return {
            'detection_stamp_sec': ns_to_sec(detection_stamp_ns),
            'valid': False,
            'invalid_reason': '',
            'gt_match_error_sec': '',
            'new_position_error_m': '',
            'new_orientation_error_deg': '',
            'old_position_error_m': '',
            'old_orientation_error_deg': '',
            'joint_positions': serialize_vector(
                self.latest_joint_state[1] if self.latest_joint_state else None),
            'velocity_command': serialize_vector(
                self.latest_velocity_command[1] if self.latest_velocity_command else None),
        }

    def record_invalid_detection(self, detection_stamp_ns, reason):
        row = self.make_base_row(detection_stamp_ns)
        row['invalid_reason'] = reason
        self.invalid_count += 1
        self.samples.append(row)

    def record_detection(self, detection_stamp_ns):
        row = self.make_base_row(detection_stamp_ns)

        try:
            detected_tf = self.tf_buffer.lookup_transform(
                self.args.camera_frame,
                self.args.detected_tag_frame,
                Time(seconds=ns_to_sec(detection_stamp_ns)))
        except Exception as exc:
            self.record_invalid_detection(
                detection_stamp_ns,
                f'detection_tf_at_stamp_failed: {exc}')
            return

        world_to_camera, camera_dt = self.gt_buffers['camera'].interpolate(detection_stamp_ns)
        world_to_tag = self.world_to_tag
        tag_dt = 0.0
        if world_to_camera is None or world_to_tag is None:
            self.record_invalid_detection(detection_stamp_ns, 'interpolated_gt_unavailable')
            return

        camera_to_tag_detected = transform_to_matrix(detected_tf.transform)
        camera_to_tag_ground_truth = np.linalg.inv(world_to_camera) @ world_to_tag
        new_position_error = np.linalg.norm(
            camera_to_tag_detected[:3, 3] - camera_to_tag_ground_truth[:3, 3])
        new_orientation_error = rotation_error_rad(
            camera_to_tag_detected, camera_to_tag_ground_truth)

        row['valid'] = True
        row['gt_match_error_sec'] = max(camera_dt or 0.0, tag_dt or 0.0)
        row['new_position_error_m'] = new_position_error
        row['new_orientation_error_deg'] = math.degrees(new_orientation_error)

        old_errors = self.compute_latest_method_error()
        if old_errors is not None:
            row['old_position_error_m'] = old_errors[0]
            row['old_orientation_error_deg'] = math.degrees(old_errors[1])

        self.samples.append(row)

    def compute_latest_method_error(self):
        latest_camera = self.gt_buffers['camera'].latest()
        latest_tag = self.gt_buffers['tag'].latest()
        if latest_camera is None or latest_tag is None:
            return None
        try:
            detected_tf = self.tf_buffer.lookup_transform(
                self.args.camera_frame,
                self.args.detected_tag_frame,
                Time())
        except Exception:
            return None
        camera_to_tag_detected = transform_to_matrix(detected_tf.transform)
        camera_to_tag_ground_truth = np.linalg.inv(latest_camera[1]) @ latest_tag[1]
        return (
            np.linalg.norm(
                camera_to_tag_detected[:3, 3] - camera_to_tag_ground_truth[:3, 3]),
            rotation_error_rad(camera_to_tag_detected, camera_to_tag_ground_truth),
        )

    def print_summary(self, elapsed_sec):
        detection_rate = self.detected_count / elapsed_sec if elapsed_sec > 0.0 else 0.0
        loss_rate = (
            1.0 - (self.detected_count / self.message_count)
            if self.message_count > 0 else 1.0
        )
        gt_wall_elapsed = (
            self.last_gt_wall_time - self.first_gt_wall_time
            if self.first_gt_wall_time and self.last_gt_wall_time else 0.0
        )
        gt_timer_hz = (
            (self.gt_timer_count - 1) / gt_wall_elapsed
            if self.gt_timer_count > 1 and gt_wall_elapsed > 0.0 else 0.0
        )
        valid_samples = [sample for sample in self.samples if sample['valid']]

        print('Dynamic ground truth summary')
        print(f'  messages: {self.message_count}')
        print(f'  detections_with_id_{self.args.tag_id}: {self.detected_count}')
        print(f'  detection_rate_hz: {detection_rate:.3f}')
        print(f'  loss_rate: {loss_rate:.3f}')
        print(f'  gt_timer_frequency_hz: {gt_timer_hz:.3f}')
        print(f'  camera_gt_buffer_frequency_hz: {self.gt_buffers["camera"].frequency_hz():.3f}')
        print(f'  valid_samples: {len(valid_samples)}')
        print(f'  invalid_samples: {self.invalid_count}')

        if not valid_samples:
            return

        new_pos = np.array([float(sample['new_position_error_m']) for sample in valid_samples])
        new_rot = np.array([float(sample['new_orientation_error_deg']) for sample in valid_samples])
        match = np.array([float(sample['gt_match_error_sec']) for sample in valid_samples])
        print(f'  new_position_error_mean_m: {new_pos.mean():.5f}')
        print(f'  new_position_error_std_m: {new_pos.std():.5f}')
        print(f'  new_orientation_error_mean_deg: {new_rot.mean():.3f}')
        print(f'  gt_match_error_max_sec: {match.max():.5f}')

        old_pos = [
            float(sample['old_position_error_m'])
            for sample in valid_samples
            if sample['old_position_error_m'] != ''
        ]
        if old_pos:
            old_pos_array = np.array(old_pos)
            print(f'  old_position_error_mean_m: {old_pos_array.mean():.5f}')
            print(
                '  position_error_improvement_m: '
                f'{old_pos_array.mean() - new_pos.mean():.5f}')

    def write_csv(self):
        if not self.args.output_csv:
            return
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'detection_stamp_sec',
            'valid',
            'invalid_reason',
            'gt_match_error_sec',
            'new_position_error_m',
            'new_orientation_error_deg',
            'old_position_error_m',
            'old_orientation_error_deg',
            'joint_positions',
            'velocity_command',
        ]
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)
        print(f'  csv: {self.args.output_csv}')


def parse_vector(value, expected_length, name):
    parts = [float(part) for part in value.replace(',', ' ').split()]
    if len(parts) != expected_length:
        raise argparse.ArgumentTypeError(
            f'{name} must have {expected_length} values, got {len(parts)}')
    return parts


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Log AprilTag detections against timestamp-synchronized Gazebo GT.')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--gt-hz', type=float, default=100.0)
    parser.add_argument('--buffer-sec', type=float, default=10.0)
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--joint-state-topic', default='/joint_states')
    parser.add_argument('--velocity-command-topic', default='/arm_velocity_controller/commands')
    parser.add_argument('--entity-state-service', default='/get_entity_state')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--ee-frame', default='link6')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--detected-tag-frame', default='apriltag_36h11_00000')
    parser.add_argument('--tag-model-name', default='apriltag_36h11_00000_target')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--service-timeout', type=float, default=5.0)
    parser.add_argument('--output-csv', default='/tmp/d435i_dynamic_ground_truth.csv')
    parser.add_argument(
        '--tag-frame-xyz-in-model',
        type=lambda value: parse_vector(value, 3, 'tag-frame-xyz-in-model'),
        default=[0.0, 0.0, 0.0])
    parser.add_argument(
        '--tag-frame-rpy-in-model',
        type=lambda value: parse_vector(value, 3, 'tag-frame-rpy-in-model'),
        default=[0.0, 0.0, 0.0])
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.duration <= 0.0:
        raise SystemExit('--duration must be positive')
    if args.gt_hz <= 0.0:
        raise SystemExit('--gt-hz must be positive')
    if args.buffer_sec <= 1.0:
        raise SystemExit('--buffer-sec must be greater than 1.0')

    rclpy.init()
    node = DynamicGroundTruthLogger(args)
    try:
        node.wait_for_inputs()
        start = time.monotonic()
        spin_timeout = min(0.002, 0.5 / args.gt_hz)
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=spin_timeout)
        flush_deadline = time.monotonic() + 1.0
        while (
            rclpy.ok()
            and node.pending_detection_stamps
            and time.monotonic() < flush_deadline
        ):
            rclpy.spin_once(node, timeout_sec=spin_timeout)
        elapsed = time.monotonic() - start
        node.print_summary(elapsed)
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
