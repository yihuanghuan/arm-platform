#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from gazebo_msgs.srv import GetEntityState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
import tf2_ros


PHASE75_POSES = [
    ('pose0', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ('pose1', [0.20, -0.25, 0.18, 0.0, -0.10, 0.0]),
    ('pose2', [-0.18, -0.35, 0.28, 0.12, -0.18, 0.10]),
]


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


class CameraMountRegression(Node):
    def __init__(self, args):
        super().__init__(
            'camera_mount_regression',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])
        self.args = args
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.entity_client = self.create_client(GetEntityState, args.entity_state_service)
        self.command_publisher = self.create_publisher(JointState, args.command_topic, 10)
        self.create_subscription(
            AprilTagDetectionArray,
            args.detections_topic,
            self.detections_callback,
            10)

        self.message_count = 0
        self.detected_count = 0
        self.latest_detection_wall_time = None
        self.latest_detection_stamp = None
        self.latest_detection_has_target = False
        self.rows = []
        self.world_to_tag = None

    def detections_callback(self, msg):
        self.message_count += 1
        has_target = any(
            detection.family == self.args.tag_family and detection.id == self.args.tag_id
            for detection in msg.detections
        )
        if has_target:
            self.detected_count += 1
            self.latest_detection_wall_time = time.monotonic()
            self.latest_detection_stamp = msg.header.stamp
        self.latest_detection_has_target = has_target

    def wait_for_inputs(self):
        if not self.entity_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f'Gazebo entity state service not available: {self.args.entity_state_service}')

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

    def publish_pose_for(self, positions, duration_sec):
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.args.joint_names
            msg.position = positions
            self.command_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def lookup_matrix(self, parent_frame, child_frame):
        tf_msg = self.tf_buffer.lookup_transform(parent_frame, child_frame, Time())
        return transform_to_matrix(tf_msg.transform)

    def sample_error(self):
        if not self.latest_detection_has_target:
            return None
        if self.latest_detection_wall_time is None:
            return None
        if time.monotonic() - self.latest_detection_wall_time > self.args.detection_stale_sec:
            return None

        detected_tf = self.tf_buffer.lookup_transform(
            self.args.camera_frame,
            self.args.detected_tag_frame,
            Time())
        world_to_camera_tf = self.tf_buffer.lookup_transform(
            self.args.world_frame,
            self.args.camera_frame,
            Time())
        camera_to_tag_detected = transform_to_matrix(detected_tf.transform)
        world_to_camera = transform_to_matrix(world_to_camera_tf.transform)
        camera_to_tag_ground_truth = np.linalg.inv(world_to_camera) @ self.world_to_tag
        position_error = np.linalg.norm(
            camera_to_tag_detected[:3, 3] - camera_to_tag_ground_truth[:3, 3])
        orientation_error = rotation_error_rad(
            camera_to_tag_detected,
            camera_to_tag_ground_truth)
        detected_distance = np.linalg.norm(camera_to_tag_detected[:3, 3])
        ground_truth_distance = np.linalg.norm(camera_to_tag_ground_truth[:3, 3])
        return {
            'position_error_m': position_error,
            'orientation_error_deg': math.degrees(orientation_error),
            'detected_distance_m': detected_distance,
            'ground_truth_distance_m': ground_truth_distance,
        }

    def run(self):
        reference_base_to_camera = None
        previous_message_count = self.message_count
        previous_detected_count = self.detected_count

        for pose_name, positions in PHASE75_POSES:
            self.publish_pose_for(positions, self.args.settle_sec)
            base_to_camera = self.lookup_matrix(self.args.base_frame, self.args.camera_frame)
            if reference_base_to_camera is None:
                reference_base_to_camera = base_to_camera
            translation_delta = np.linalg.norm(
                base_to_camera[:3, 3] - reference_base_to_camera[:3, 3])
            orientation_delta = rotation_error_rad(base_to_camera, reference_base_to_camera)

            sample_start_messages = self.message_count
            sample_start_detected = self.detected_count
            samples = []
            end_time = time.monotonic() + self.args.sample_duration
            next_sample_time = time.monotonic()
            while rclpy.ok() and time.monotonic() < end_time:
                self.publish_pose_for(positions, 0.05)
                if time.monotonic() >= next_sample_time:
                    try:
                        sample = self.sample_error()
                    except Exception as exc:
                        self.get_logger().warn(
                            f'Pose {pose_name} sample failed: {exc}',
                            throttle_duration_sec=2.0)
                        sample = None
                    if sample is not None:
                        samples.append(sample)
                    next_sample_time += 1.0 / self.args.sample_hz

            elapsed_messages = self.message_count - sample_start_messages
            elapsed_detected = self.detected_count - sample_start_detected
            detection_rate = elapsed_detected / self.args.sample_duration
            loss_rate = (
                1.0 - (elapsed_detected / elapsed_messages)
                if elapsed_messages > 0 else 1.0
            )

            row = {
                'pose': pose_name,
                'joint_positions': ' '.join(f'{value:.6g}' for value in positions),
                'base_camera_translation_delta_m': translation_delta,
                'base_camera_orientation_delta_rad': orientation_delta,
                'detection_rate_hz': detection_rate,
                'loss_rate': loss_rate,
                'sample_count': len(samples),
                'position_error_mean_m': '',
                'position_error_std_m': '',
                'orientation_error_mean_deg': '',
                'orientation_error_std_deg': '',
                'detected_distance_mean_m': '',
                'ground_truth_distance_mean_m': '',
            }
            if samples:
                position_errors = np.array([sample['position_error_m'] for sample in samples])
                orientation_errors = np.array([
                    sample['orientation_error_deg'] for sample in samples])
                detected_distances = np.array([
                    sample['detected_distance_m'] for sample in samples])
                gt_distances = np.array([
                    sample['ground_truth_distance_m'] for sample in samples])
                row.update({
                    'position_error_mean_m': position_errors.mean(),
                    'position_error_std_m': position_errors.std(),
                    'orientation_error_mean_deg': orientation_errors.mean(),
                    'orientation_error_std_deg': orientation_errors.std(),
                    'detected_distance_mean_m': detected_distances.mean(),
                    'ground_truth_distance_mean_m': gt_distances.mean(),
                })

            self.rows.append(row)
            print(
                f'{pose_name}: base_camera_translation_delta={translation_delta:.9g} m, '
                f'base_camera_orientation_delta={orientation_delta:.9g} rad, '
                f'detection_rate={detection_rate:.3f} Hz, loss_rate={loss_rate:.3f}')

        total_messages = self.message_count - previous_message_count
        total_detected = self.detected_count - previous_detected_count
        print('Base camera regression summary')
        print(f'  messages: {total_messages}')
        print(f'  detections_with_id_{self.args.tag_id}: {total_detected}')
        self.write_csv()

    def write_csv(self):
        if not self.args.output_csv:
            return
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'pose',
            'joint_positions',
            'base_camera_translation_delta_m',
            'base_camera_orientation_delta_rad',
            'detection_rate_hz',
            'loss_rate',
            'sample_count',
            'position_error_mean_m',
            'position_error_std_m',
            'orientation_error_mean_deg',
            'orientation_error_std_deg',
            'detected_distance_mean_m',
            'ground_truth_distance_mean_m',
        ]
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)
        print(f'  csv: {self.args.output_csv}')


def parse_vector(value, expected_length, name):
    parts = [float(part) for part in value.replace(',', ' ').split()]
    if len(parts) != expected_length:
        raise argparse.ArgumentTypeError(
            f'{name} must have {expected_length} values, got {len(parts)}')
    return parts


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Run the phase 7.5 base camera mount and AprilTag regression.')
    parser.add_argument('--command-topic', default='/student/joint_command')
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--entity-state-service', default='/get_entity_state')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--detected-tag-frame', default='apriltag_36h11_00000')
    parser.add_argument('--tag-model-name', default='apriltag_36h11_00000_target')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--settle-sec', type=float, default=2.0)
    parser.add_argument('--sample-duration', type=float, default=8.0)
    parser.add_argument('--sample-hz', type=float, default=5.0)
    parser.add_argument('--detection-stale-sec', type=float, default=0.5)
    parser.add_argument('--service-timeout', type=float, default=5.0)
    parser.add_argument('--output-csv', default='/tmp/d435i_base_camera_regression.csv')
    parser.add_argument(
        '--tag-frame-xyz-in-model',
        type=lambda value: parse_vector(value, 3, 'tag-frame-xyz-in-model'),
        default=[0.0, 0.0, 0.0])
    parser.add_argument(
        '--tag-frame-rpy-in-model',
        type=lambda value: parse_vector(value, 3, 'tag-frame-rpy-in-model'),
        default=[0.0, 0.0, 0.0])
    parser.add_argument(
        '--joint-names',
        nargs='+',
        default=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.settle_sec < 0.0:
        raise SystemExit('--settle-sec must be non-negative')
    if args.sample_duration <= 0.0:
        raise SystemExit('--sample-duration must be positive')
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')

    rclpy.init()
    node = CameraMountRegression(args)
    try:
        node.wait_for_inputs()
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
