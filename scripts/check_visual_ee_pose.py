#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
import sys
import time
from collections import deque

from apriltag_msgs.msg import AprilTagDetectionArray
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import tf2_ros


DEFAULT_CONFIGS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.18, -0.22, 0.16, 0.10, -0.12, 0.18],
    [-0.16, 0.14, -0.12, -0.10, 0.10, -0.16],
]


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def ns_to_sec(stamp_ns):
    return float(stamp_ns) * 1e-9


def pose_position(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def position_error(a_pose, b_pose):
    ax, ay, az = pose_position(a_pose)
    bx, by, bz = pose_position(b_pose)
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def serialize_pose(prefix, pose):
    return {
        f'{prefix}_x': f'{pose.position.x:.9f}',
        f'{prefix}_y': f'{pose.position.y:.9f}',
        f'{prefix}_z': f'{pose.position.z:.9f}',
        f'{prefix}_qx': f'{pose.orientation.x:.9f}',
        f'{prefix}_qy': f'{pose.orientation.y:.9f}',
        f'{prefix}_qz': f'{pose.orientation.z:.9f}',
        f'{prefix}_qw': f'{pose.orientation.w:.9f}',
    }


def transform_to_pose(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w
    return pose


def parse_config(value):
    parts = [float(part) for part in value.replace(',', ' ').split()]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            f'joint config must contain 6 values, got {len(parts)}')
    return parts


class VisualEePoseChecker(Node):
    def __init__(self, args):
        super().__init__(
            'check_visual_ee_pose',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.latest_pose = None
        self.latest_valid = False
        self.latest_detection_stamp_ns = 0
        self.detection_stamp_history = deque(maxlen=1000)
        self.detection_messages = 0
        self.target_detections = 0
        self.samples = []

        self.command_pub = self.create_publisher(
            JointState, args.joint_command_topic, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PoseStamped, args.visual_pose_topic, self.pose_callback, 20)
        self.create_subscription(
            Bool, args.visual_valid_topic, self.valid_callback, 20)
        self.create_subscription(
            AprilTagDetectionArray,
            args.detections_topic,
            self.detections_callback,
            20)
        self.entity_client = self.create_client(
            GetEntityState, args.entity_state_service)

    def pose_callback(self, msg):
        self.latest_pose = msg

    def valid_callback(self, msg):
        self.latest_valid = bool(msg.data)

    def detections_callback(self, msg):
        self.detection_messages += 1
        self.latest_detection_stamp_ns = stamp_to_ns(msg.header.stamp)
        if any(
            detection.family == self.args.tag_family
            and detection.id == self.args.tag_id
            for detection in msg.detections
        ):
            self.target_detections += 1
            self.detection_stamp_history.append(self.latest_detection_stamp_ns)

    def wait_for_service(self):
        if self.args.ground_truth_source != 'gazebo_entity':
            return
        if not self.entity_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f'Gazebo entity state service not available: {self.args.entity_state_service}')

    def wait_for_initial_pose(self):
        deadline = time.monotonic() + self.args.initial_pose_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_pose is not None and self.latest_valid:
                return
        raise RuntimeError('Timed out waiting for an initial valid /visual_ee_pose')

    def publish_joint_config(self, config):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.args.joint_names)
        msg.position = list(config)
        self.command_pub.publish(msg)

    def command_and_settle(self, config):
        deadline = time.monotonic() + self.args.settle_sec
        period = 1.0 / self.args.command_rate_hz
        next_command = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_command:
                self.publish_joint_config(config)
                next_command += period
            rclpy.spin_once(self, timeout_sec=0.01)

    def get_ground_truth_pose(self, pose_stamp):
        if self.args.ground_truth_source == 'tf':
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.world_frame,
                    self.args.ee_frame,
                    Time.from_msg(pose_stamp),
                    timeout=Duration(seconds=self.args.tf_timeout_sec))
            except Exception:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.args.world_frame,
                        self.args.ee_frame,
                        Time(),
                        timeout=Duration(seconds=self.args.tf_timeout_sec))
                except Exception as exc:
                    return None, f'tf_ground_truth_failed: {exc}'
            return transform_to_pose(transform.transform), ''

        request = GetEntityState.Request()
        request.name = self.args.ee_entity_name
        request.reference_frame = self.args.world_frame
        future = self.entity_client.call_async(request)
        deadline = time.monotonic() + self.args.service_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
        if not future.done():
            return None, 'entity_state_timeout'
        response = future.result()
        if response is None or not response.success:
            return None, 'entity_state_failed'
        return response.state.pose, ''

    def sample_config(self, config_index, config):
        self.command_and_settle(config)
        deadline = time.monotonic() + self.args.sample_duration_sec
        period = 1.0 / self.args.sample_hz
        next_sample = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_sample:
                rclpy.spin_once(self, timeout_sec=min(0.01, next_sample - now))
                continue
            next_sample += period
            rclpy.spin_once(self, timeout_sec=0.001)
            self.sample_once(config_index, config)

    def sample_once(self, config_index, config):
        now_ns = self.get_clock().now().nanoseconds
        pose = self.latest_pose
        valid = self.latest_valid and pose is not None
        invalid_reason = ''
        pose_stamp_ns = stamp_to_ns(pose.header.stamp) if pose is not None else 0
        pose_age_sec = (
            ns_to_sec(now_ns - pose_stamp_ns)
            if now_ns > 0 and pose_stamp_ns > 0 else None)
        if not valid:
            invalid_reason = 'visual_pose_invalid'
        elif pose.header.frame_id != self.args.world_frame:
            valid = False
            invalid_reason = 'wrong_frame'
        elif pose_age_sec is None or pose_age_sec > self.args.pose_timeout_sec:
            valid = False
            invalid_reason = 'visual_pose_stale'

        row = {
            'config_index': config_index,
            'joint_config': ';'.join(f'{value:.9g}' for value in config),
            'valid': valid,
            'invalid_reason': invalid_reason,
            'visual_frame': pose.header.frame_id if pose is not None else '',
            'visual_stamp_sec': ns_to_sec(pose_stamp_ns) if pose_stamp_ns > 0 else '',
            'latest_detection_stamp_sec': (
                ns_to_sec(self.latest_detection_stamp_ns)
                if self.latest_detection_stamp_ns > 0 else ''),
            'stamp_in_detection_history': (
                pose_stamp_ns in self.detection_stamp_history
                if pose_stamp_ns > 0 else False),
            'pose_age_sec': f'{pose_age_sec:.9f}' if pose_age_sec is not None else '',
            'position_error_m': '',
        }

        if not valid:
            self.samples.append(row)
            return

        gt_pose, gt_error = self.get_ground_truth_pose(pose.header.stamp)
        if gt_pose is None:
            row['valid'] = False
            row['invalid_reason'] = gt_error
            self.samples.append(row)
            return

        row.update(serialize_pose('visual', pose.pose))
        row.update(serialize_pose('ground_truth', gt_pose))
        row['position_error_m'] = f'{position_error(pose.pose, gt_pose):.9f}'
        self.samples.append(row)

    def run(self, configs):
        self.wait_for_service()
        self.wait_for_initial_pose()
        for index, config in enumerate(configs):
            self.get_logger().info(f'Sampling config {index}: {config}')
            self.sample_config(index, config)

    def write_csv(self):
        if not self.args.output_csv:
            return
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'config_index',
            'joint_config',
            'valid',
            'invalid_reason',
            'visual_frame',
            'visual_stamp_sec',
            'latest_detection_stamp_sec',
            'stamp_in_detection_history',
            'pose_age_sec',
            'position_error_m',
            'visual_x',
            'visual_y',
            'visual_z',
            'visual_qx',
            'visual_qy',
            'visual_qz',
            'visual_qw',
            'ground_truth_x',
            'ground_truth_y',
            'ground_truth_z',
            'ground_truth_qx',
            'ground_truth_qy',
            'ground_truth_qz',
            'ground_truth_qw',
        ]
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)
        print(f'  csv: {self.args.output_csv}')

    def print_summary(self):
        valid_samples = [
            sample for sample in self.samples
            if sample['valid'] and sample['position_error_m'] != ''
        ]
        loss_rate = (
            1.0 - (len(valid_samples) / len(self.samples))
            if self.samples else 1.0)
        detection_loss_rate = (
            1.0 - (self.target_detections / self.detection_messages)
            if self.detection_messages > 0 else 1.0)
        print('Visual EE pose validation summary')
        print(f'  samples: {len(self.samples)}')
        print(f'  valid_samples: {len(valid_samples)}')
        print(f'  visual_loss_rate: {loss_rate:.3f}')
        print(f'  detection_messages: {self.detection_messages}')
        print(f'  target_detections: {self.target_detections}')
        print(f'  detection_loss_rate: {detection_loss_rate:.3f}')
        if not valid_samples:
            return
        errors = [float(sample['position_error_m']) for sample in valid_samples]
        stamp_matches = [
            sample['stamp_in_detection_history'] for sample in valid_samples]
        frames = {sample['visual_frame'] for sample in valid_samples}
        print(f'  visual_frames: {sorted(frames)}')
        print(f'  stamp_in_detection_history_fraction: {sum(stamp_matches) / len(stamp_matches):.3f}')
        print(f'  position_error_mean_m: {statistics.mean(errors):.6f}')
        print(f'  position_error_max_m: {max(errors):.6f}')
        if len(errors) > 1:
            print(f'  position_error_std_m: {statistics.pstdev(errors):.6f}')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Validate /visual_ee_pose against Gazebo link6 ground truth.')
    parser.add_argument('--visual-pose-topic', default='/visual_ee_pose')
    parser.add_argument('--visual-valid-topic', default='/visual_ee_pose_valid')
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--joint-command-topic', default='/student/joint_command')
    parser.add_argument('--entity-state-service', default='/get_entity_state')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--ee-frame', default='link6')
    parser.add_argument('--ee-entity-name', default='windylab_arm::link6')
    parser.add_argument(
        '--ground-truth-source',
        choices=['tf', 'gazebo_entity'],
        default='tf')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--settle-sec', type=float, default=2.0)
    parser.add_argument('--sample-duration-sec', type=float, default=3.0)
    parser.add_argument('--sample-hz', type=float, default=5.0)
    parser.add_argument('--command-rate-hz', type=float, default=20.0)
    parser.add_argument('--pose-timeout-sec', type=float, default=0.5)
    parser.add_argument('--service-timeout', type=float, default=2.0)
    parser.add_argument('--tf-timeout-sec', type=float, default=0.1)
    parser.add_argument('--initial-pose-timeout-sec', type=float, default=10.0)
    parser.add_argument('--output-csv', default='/tmp/windylab_phase1_visual_ee_pose.csv')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument(
        '--joint-names',
        nargs=6,
        default=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
    parser.add_argument(
        '--config',
        action='append',
        type=parse_config,
        help='Six joint positions in radians. May be repeated.')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.settle_sec < 0.0:
        raise SystemExit('--settle-sec must be non-negative')
    if args.sample_duration_sec <= 0.0:
        raise SystemExit('--sample-duration-sec must be positive')
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')
    if args.command_rate_hz <= 0.0:
        raise SystemExit('--command-rate-hz must be positive')
    configs = args.config if args.config else DEFAULT_CONFIGS

    rclpy.init()
    node = VisualEePoseChecker(args)
    try:
        node.run(configs)
        node.print_summary()
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
