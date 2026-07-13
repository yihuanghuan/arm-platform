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
from rclpy.time import Time
from scipy.spatial.transform import Rotation
import tf2_ros


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


def make_model_to_tag_matrix(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(rpy)
    matrix[:3, 3] = xyz
    return matrix


def rotation_error_rad(a_matrix, b_matrix):
    delta = a_matrix[:3, :3] @ b_matrix[:3, :3].T
    return Rotation.from_matrix(delta).magnitude()


class AprilTagGroundTruthChecker(Node):
    def __init__(self, args):
        super().__init__('apriltag_ground_truth_checker')
        self.args = args
        self.message_count = 0
        self.detected_count = 0
        self.latest_detection_stamp = None
        self.latest_detection_has_target = False
        self.last_detected_wall_time = None
        self.samples = []

        self.create_subscription(
            AprilTagDetectionArray,
            args.detections_topic,
            self.detections_callback,
            10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.entity_client = self.create_client(GetEntityState, args.entity_state_service)

    def detections_callback(self, msg):
        self.message_count += 1
        has_target = any(
            detection.family == self.args.tag_family and detection.id == self.args.tag_id
            for detection in msg.detections
        )
        if has_target:
            self.detected_count += 1
            self.latest_detection_stamp = msg.header.stamp
            self.last_detected_wall_time = time.monotonic()
        self.latest_detection_has_target = has_target

    def wait_for_inputs(self):
        if not self.entity_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f'Gazebo entity state service not available: {self.args.entity_state_service}')

    def sample_once(self):
        if not self.latest_detection_has_target:
            return None
        if self.last_detected_wall_time is None:
            return None
        if time.monotonic() - self.last_detected_wall_time > self.args.detection_stale_sec:
            return None

        try:
            detected_tf = self.tf_buffer.lookup_transform(
                self.args.camera_frame,
                self.args.detected_tag_frame,
                Time())
            world_to_camera_tf = self.tf_buffer.lookup_transform(
                self.args.world_frame,
                self.args.camera_frame,
                Time())
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=2.0)
            return None

        request = GetEntityState.Request()
        request.name = self.args.tag_model_name
        request.reference_frame = self.args.world_frame
        future = self.entity_client.call_async(request)
        deadline = time.monotonic() + self.args.service_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            self.get_logger().warn('Timed out waiting for Gazebo entity state')
            return None

        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f'Gazebo entity state failed for {self.args.tag_model_name}')
            return None

        camera_to_tag_detected = transform_to_matrix(detected_tf.transform)
        world_to_camera = transform_to_matrix(world_to_camera_tf.transform)
        world_to_tag_model = pose_to_matrix(response.state.pose)
        model_to_tag = make_model_to_tag_matrix(
            self.args.tag_frame_xyz_in_model,
            self.args.tag_frame_rpy_in_model)
        camera_to_tag_ground_truth = (
            np.linalg.inv(world_to_camera) @ world_to_tag_model @ model_to_tag)

        position_error = np.linalg.norm(
            camera_to_tag_detected[:3, 3] - camera_to_tag_ground_truth[:3, 3])
        orientation_error = rotation_error_rad(
            camera_to_tag_detected, camera_to_tag_ground_truth)

        sample = {
            'stamp_sec': self.get_clock().now().nanoseconds * 1e-9,
            'detected_x': camera_to_tag_detected[0, 3],
            'detected_y': camera_to_tag_detected[1, 3],
            'detected_z': camera_to_tag_detected[2, 3],
            'ground_truth_x': camera_to_tag_ground_truth[0, 3],
            'ground_truth_y': camera_to_tag_ground_truth[1, 3],
            'ground_truth_z': camera_to_tag_ground_truth[2, 3],
            'position_error_m': position_error,
            'orientation_error_rad': orientation_error,
            'orientation_error_deg': math.degrees(orientation_error),
        }
        self.samples.append(sample)
        return sample

    def print_summary(self, elapsed_sec):
        detection_rate = self.detected_count / elapsed_sec if elapsed_sec > 0.0 else 0.0
        loss_rate = (
            1.0 - (self.detected_count / self.message_count)
            if self.message_count > 0 else 1.0
        )
        print('AprilTag validation summary')
        print(f'  messages: {self.message_count}')
        print(f'  detections_with_id_{self.args.tag_id}: {self.detected_count}')
        print(f'  detection_rate_hz: {detection_rate:.3f}')
        print(f'  loss_rate: {loss_rate:.3f}')
        print(f'  samples: {len(self.samples)}')

        if not self.samples:
            return

        position_errors = np.array([s['position_error_m'] for s in self.samples])
        orientation_errors = np.array([s['orientation_error_deg'] for s in self.samples])
        detected_z = np.array([s['detected_z'] for s in self.samples])
        ground_truth_z = np.array([s['ground_truth_z'] for s in self.samples])
        print(f'  detected_z_mean_m: {detected_z.mean():.4f}')
        print(f'  ground_truth_z_mean_m: {ground_truth_z.mean():.4f}')
        print(f'  position_error_mean_m: {position_errors.mean():.4f}')
        print(f'  position_error_std_m: {position_errors.std():.4f}')
        print(f'  orientation_error_mean_deg: {orientation_errors.mean():.2f}')
        print(f'  orientation_error_std_deg: {orientation_errors.std():.2f}')

    def write_csv(self):
        if not self.args.output_csv:
            return
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'stamp_sec',
            'detected_x',
            'detected_y',
            'detected_z',
            'ground_truth_x',
            'ground_truth_y',
            'ground_truth_z',
            'position_error_m',
            'orientation_error_rad',
            'orientation_error_deg',
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
        description='Compare apriltag_ros TF output with Gazebo ground truth.')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--sample-hz', type=float, default=5.0)
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--entity-state-service', default='/get_entity_state')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--detected-tag-frame', default='apriltag_36h11_00000')
    parser.add_argument('--tag-model-name', default='apriltag_36h11_00000_target')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--detection-stale-sec', type=float, default=0.5)
    parser.add_argument('--service-timeout', type=float, default=2.0)
    parser.add_argument('--output-csv', default='/tmp/d435i_apriltag_validation.csv')
    parser.add_argument(
        '--tag-frame-xyz-in-model',
        type=lambda value: parse_vector(value, 3, 'tag-frame-xyz-in-model'),
        default=[0.0, 0.0, 0.0],
        help='Translation from Gazebo tag model frame to apriltag_ros tag frame.')
    parser.add_argument(
        '--tag-frame-rpy-in-model',
        type=lambda value: parse_vector(value, 3, 'tag-frame-rpy-in-model'),
        default=[0.0, 0.0, 0.0],
        help='RPY from Gazebo tag model frame to apriltag_ros tag frame.')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.duration <= 0.0:
        raise SystemExit('--duration must be positive')
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')

    rclpy.init()
    node = AprilTagGroundTruthChecker(args)
    try:
        node.wait_for_inputs()
        start = time.monotonic()
        next_sample = start
        period = 1.0 / args.sample_hz
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_sample:
                node.sample_once()
                next_sample += period
        elapsed = time.monotonic() - start
        node.print_summary(elapsed)
        node.write_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
