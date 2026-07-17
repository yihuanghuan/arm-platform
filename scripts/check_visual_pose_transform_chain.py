#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from scipy.spatial.transform import Rotation
import tf2_ros


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def ns_to_sec(stamp_ns):
    return float(stamp_ns) * 1e-9


def parse_vector(text, expected_length):
    values = [float(part) for part in str(text).replace(',', ' ').split()]
    if len(values) != expected_length:
        raise ValueError(f'expected {expected_length} values, got {len(values)}')
    return values


def make_transform_matrix(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def pose_to_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]).as_matrix()
    matrix[:3, 3] = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ]
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


def matrix_to_xyz_quat(matrix):
    if matrix is None:
        return [''] * 7
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return [
        f'{matrix[0, 3]:.9f}',
        f'{matrix[1, 3]:.9f}',
        f'{matrix[2, 3]:.9f}',
        f'{quat[0]:.9f}',
        f'{quat[1]:.9f}',
        f'{quat[2]:.9f}',
        f'{quat[3]:.9f}',
    ]


def rotation_angle_between(a_matrix, b_matrix):
    if a_matrix is None or b_matrix is None:
        return None
    relative = a_matrix[:3, :3].T @ b_matrix[:3, :3]
    return float(Rotation.from_matrix(relative).magnitude())


def translation_delta(a_matrix, b_matrix):
    if a_matrix is None or b_matrix is None:
        return None
    return float(np.linalg.norm(a_matrix[:3, 3] - b_matrix[:3, 3]))


def format_float(value):
    if value is None:
        return ''
    return f'{float(value):.9f}'


def format_bool(value):
    if value is None:
        return ''
    return str(bool(value)).lower()


class VisualPoseTransformChainCheck(Node):
    def __init__(self, args):
        super().__init__(
            'check_visual_pose_transform_chain',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.started_wall = time.monotonic()
        self.model_poses = {}
        self.link_poses = {}
        self.latest_visual_pose = None
        self.latest_visual_stamp_ns = 0
        self.previous_measured = None
        self.previous_measured_stamp_ns = 0
        self.world_to_tag_config = make_transform_matrix(
            parse_vector(args.world_to_tag_xyz, 3),
            parse_vector(args.world_to_tag_rpy, 3))

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(ModelStates, args.model_states_topic,
                                 self.model_states_callback, 20)
        self.create_subscription(LinkStates, args.link_states_topic,
                                 self.link_states_callback, 20)
        self.create_subscription(PoseStamped, args.visual_pose_topic,
                                 self.visual_pose_callback, 20)

        output_dir = os.path.dirname(os.path.abspath(args.output_csv))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        self.output_handle = open(args.output_csv, 'w', newline='', encoding='utf-8')
        self.fieldnames = self.make_fieldnames()
        self.writer = csv.DictWriter(self.output_handle, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.timer = self.create_timer(1.0 / args.sample_hz, self.sample_once)
        self.get_logger().info(
            f'Writing visual pose transform chain diagnostics to {args.output_csv}')

    def destroy_node(self):
        try:
            self.output_handle.flush()
            self.output_handle.close()
        finally:
            super().destroy_node()

    def make_fieldnames(self):
        fields = [
            'wall_time_sec',
            'sim_time_sec',
            'tag_tf_stamp_sec',
            'tag_tf_lookup_ok',
            'tag_tf_error',
            'tag_tf_stamp_changed',
            'tag_tf_translation_changed',
            'tag_tf_rotation_changed',
            'tag_tf_translation_delta_m',
            'tag_tf_rotation_delta_rad',
            'world_to_tag_config_translation_error_m',
            'world_to_tag_config_rotation_error_rad',
            'visual_pose_stamp_sec',
            'visual_pose_age_sec',
        ]
        for prefix in (
                'base_gt',
                'link6_gt',
                'camera_gt',
                'tag_gt',
                'camera_tag_gt',
                'camera_tag_measured',
                'visual_ee'):
            fields.extend([
                f'{prefix}_x',
                f'{prefix}_y',
                f'{prefix}_z',
                f'{prefix}_qx',
                f'{prefix}_qy',
                f'{prefix}_qz',
                f'{prefix}_qw',
            ])
        return fields

    def model_states_callback(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            self.model_poses[name] = pose_to_matrix(pose)

    def link_states_callback(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            self.link_poses[name] = pose_to_matrix(pose)

    def visual_pose_callback(self, msg):
        self.latest_visual_pose = pose_to_matrix(msg.pose)
        self.latest_visual_stamp_ns = stamp_to_ns(msg.header.stamp)

    def lookup_tf_matrix(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=self.args.tf_timeout_sec))
        return transform_to_matrix(transform.transform), stamp_to_ns(transform.header.stamp)

    def world_to_camera_gt(self, world_to_link6):
        if world_to_link6 is None:
            return None
        try:
            link6_to_camera, _ = self.lookup_tf_matrix(
                self.args.ee_frame,
                self.args.camera_frame)
        except Exception:
            return None
        return world_to_link6 @ link6_to_camera

    def sample_once(self):
        now_ns = self.get_clock().now().nanoseconds
        world_to_base = self.link_poses.get(self.args.base_link_name)
        world_to_link6 = self.link_poses.get(self.args.ee_link_name)
        world_to_tag_gt = None
        if self.args.tag_link_name:
            world_to_tag_gt = self.link_poses.get(self.args.tag_link_name)
        if world_to_tag_gt is None:
            world_to_tag_gt = self.model_poses.get(self.args.tag_model_name)
        world_to_camera = self.world_to_camera_gt(world_to_link6)
        camera_to_tag_gt = None
        if world_to_camera is not None and world_to_tag_gt is not None:
            camera_to_tag_gt = np.linalg.inv(world_to_camera) @ world_to_tag_gt

        tag_tf_lookup_ok = False
        tag_tf_error = ''
        camera_to_tag_measured = None
        tag_tf_stamp_ns = 0
        try:
            camera_to_tag_measured, tag_tf_stamp_ns = self.lookup_tf_matrix(
                self.args.camera_frame,
                self.args.detected_tag_frame)
            tag_tf_lookup_ok = True
        except Exception as exc:
            tag_tf_error = str(exc)

        stamp_changed = None
        translation_changed = None
        rotation_changed = None
        measured_translation_delta = None
        measured_rotation_delta = None
        if camera_to_tag_measured is not None:
            stamp_changed = tag_tf_stamp_ns != self.previous_measured_stamp_ns
            measured_translation_delta = translation_delta(
                self.previous_measured, camera_to_tag_measured)
            measured_rotation_delta = rotation_angle_between(
                self.previous_measured, camera_to_tag_measured)
            translation_changed = (
                measured_translation_delta is None
                or measured_translation_delta > self.args.translation_epsilon_m)
            rotation_changed = (
                measured_rotation_delta is None
                or measured_rotation_delta > self.args.rotation_epsilon_rad)
            self.previous_measured = camera_to_tag_measured
            self.previous_measured_stamp_ns = tag_tf_stamp_ns

        config_translation_error = translation_delta(
            self.world_to_tag_config, world_to_tag_gt)
        config_rotation_error = rotation_angle_between(
            self.world_to_tag_config, world_to_tag_gt)
        visual_age_sec = (
            ns_to_sec(now_ns - self.latest_visual_stamp_ns)
            if now_ns > 0 and self.latest_visual_stamp_ns > 0 else None)

        row = {
            'wall_time_sec': format_float(time.monotonic() - self.started_wall),
            'sim_time_sec': format_float(ns_to_sec(now_ns) if now_ns > 0 else None),
            'tag_tf_stamp_sec': format_float(
                ns_to_sec(tag_tf_stamp_ns) if tag_tf_stamp_ns > 0 else None),
            'tag_tf_lookup_ok': format_bool(tag_tf_lookup_ok),
            'tag_tf_error': tag_tf_error,
            'tag_tf_stamp_changed': format_bool(stamp_changed),
            'tag_tf_translation_changed': format_bool(translation_changed),
            'tag_tf_rotation_changed': format_bool(rotation_changed),
            'tag_tf_translation_delta_m': format_float(measured_translation_delta),
            'tag_tf_rotation_delta_rad': format_float(measured_rotation_delta),
            'world_to_tag_config_translation_error_m': format_float(
                config_translation_error),
            'world_to_tag_config_rotation_error_rad': format_float(
                config_rotation_error),
            'visual_pose_stamp_sec': format_float(
                ns_to_sec(self.latest_visual_stamp_ns)
                if self.latest_visual_stamp_ns > 0 else None),
            'visual_pose_age_sec': format_float(visual_age_sec),
        }
        self.add_matrix(row, 'base_gt', world_to_base)
        self.add_matrix(row, 'link6_gt', world_to_link6)
        self.add_matrix(row, 'camera_gt', world_to_camera)
        self.add_matrix(row, 'tag_gt', world_to_tag_gt)
        self.add_matrix(row, 'camera_tag_gt', camera_to_tag_gt)
        self.add_matrix(row, 'camera_tag_measured', camera_to_tag_measured)
        self.add_matrix(row, 'visual_ee', self.latest_visual_pose)
        self.writer.writerow(row)
        self.output_handle.flush()

    def add_matrix(self, row, prefix, matrix):
        values = matrix_to_xyz_quat(matrix)
        for suffix, value in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), values):
            row[f'{prefix}_{suffix}'] = value


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Record phase4.3 visual pose transform chain diagnostics.')
    parser.add_argument('--output-csv',
                        default='/tmp/phase4_3_visual_pose_transform_chain.csv')
    parser.add_argument('--duration-sec', type=float, default=0.0)
    parser.add_argument('--sample-hz', type=float, default=5.0)
    parser.add_argument('--model-states-topic', default='/model_states')
    parser.add_argument('--link-states-topic', default='/link_states')
    parser.add_argument('--visual-pose-topic', default='/visual_ee_pose')
    parser.add_argument('--base-link-name', default='windylab_arm::base_link')
    parser.add_argument('--ee-link-name', default='windylab_arm::link6')
    parser.add_argument('--tag-model-name', default='apriltag_36h11_00000_target')
    parser.add_argument('--tag-link-name', default='')
    parser.add_argument('--ee-frame', default='link6')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--detected-tag-frame', default='apriltag_36h11_00000')
    parser.add_argument('--world-to-tag-xyz',
                        default='1.60042456 0.000976374 0.35065986')
    parser.add_argument('--world-to-tag-rpy',
                        default='-3.12204785 -1.56214388 3.12103003')
    parser.add_argument('--tf-timeout-sec', type=float, default=0.02)
    parser.add_argument('--translation-epsilon-m', type=float, default=1e-6)
    parser.add_argument('--rotation-epsilon-rad', type=float, default=1e-6)
    parser.add_argument('--use-sim-time', action='store_true')
    args, _ = parser.parse_known_args(argv)
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')
    if args.duration_sec < 0.0:
        raise SystemExit('--duration-sec must be non-negative')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = VisualPoseTransformChainCheck(args)
    deadline = None
    if args.duration_sec > 0.0:
        deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        text = str(exc)
        shutdown_exception = (
            'context is not valid' in text
            or 'Unable to convert call argument to Python object' in text)
        if rclpy.ok() and not shutdown_exception:
            raise
    finally:
        try:
            node.get_logger().info(f'Transform-chain CSV: {args.output_csv}')
        except Exception:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
