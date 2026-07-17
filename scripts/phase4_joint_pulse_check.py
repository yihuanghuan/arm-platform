#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time

from apriltag_msgs.msg import AprilTagDetectionArray
from gazebo_msgs.msg import LinkStates
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Phase4JointPulseCheck(Node):
    def __init__(self, args):
        super().__init__(
            'phase4_joint_pulse_check',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.latest_joint_state = None
        self.latest_base_pose = None
        self.latest_link_pose = None
        self.target_detected = False
        self.detection_count = 0
        self.command_pub = self.create_publisher(
            Float64MultiArray, args.command_topic, 10)
        self.create_subscription(JointState, args.joint_states_topic,
                                 self.joint_state_callback, 20)
        self.create_subscription(LinkStates, args.link_states_topic,
                                 self.link_states_callback, 20)
        self.create_subscription(AprilTagDetectionArray, args.detections_topic,
                                 self.detections_callback, 20)

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def link_states_callback(self, msg):
        try:
            base_index = msg.name.index(self.args.base_link_name)
        except ValueError:
            base_index = -1
        if 0 <= base_index < len(msg.pose):
            self.latest_base_pose = msg.pose[base_index]
        try:
            index = msg.name.index(self.args.ee_link_name)
        except ValueError:
            return
        if index < len(msg.pose):
            self.latest_link_pose = msg.pose[index]

    def detections_callback(self, msg):
        self.target_detected = any(
            detection.family == self.args.tag_family and detection.id == self.args.tag_id
            for detection in msg.detections)
        if self.target_detected:
            self.detection_count += 1

    def publish_command(self, values):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        self.command_pub.publish(msg)

    def current_joint_positions(self):
        values = [''] * len(self.joint_names)
        if self.latest_joint_state is None:
            return values
        for out_index, name in enumerate(self.joint_names):
            try:
                msg_index = self.latest_joint_state.name.index(name)
            except ValueError:
                continue
            if msg_index < len(self.latest_joint_state.position):
                values[out_index] = f'{self.latest_joint_state.position[msg_index]:.9f}'
        return values

    def sample_row(self, phase, command_value):
        now = self.get_clock().now().to_msg()
        row = {
            'wall_time_sec': f'{time.monotonic():.9f}',
            'sim_time_sec': f'{stamp_to_sec(now):.9f}',
            'phase': phase,
            'command_joint': self.args.joint_name,
            'command_velocity': f'{command_value:.9f}',
            'target_detected': str(bool(self.target_detected)).lower(),
            'detection_count': self.detection_count,
            'base_x': '',
            'base_y': '',
            'base_z': '',
            'base_qx': '',
            'base_qy': '',
            'base_qz': '',
            'base_qw': '',
            'ee_x': '',
            'ee_y': '',
            'ee_z': '',
            'ee_qx': '',
            'ee_qy': '',
            'ee_qz': '',
            'ee_qw': '',
        }
        if self.latest_base_pose is not None:
            pose = self.latest_base_pose
            row.update({
                'base_x': f'{pose.position.x:.9f}',
                'base_y': f'{pose.position.y:.9f}',
                'base_z': f'{pose.position.z:.9f}',
                'base_qx': f'{pose.orientation.x:.9f}',
                'base_qy': f'{pose.orientation.y:.9f}',
                'base_qz': f'{pose.orientation.z:.9f}',
                'base_qw': f'{pose.orientation.w:.9f}',
            })
        if self.latest_link_pose is not None:
            pose = self.latest_link_pose
            row.update({
                'ee_x': f'{pose.position.x:.9f}',
                'ee_y': f'{pose.position.y:.9f}',
                'ee_z': f'{pose.position.z:.9f}',
                'ee_qx': f'{pose.orientation.x:.9f}',
                'ee_qy': f'{pose.orientation.y:.9f}',
                'ee_qz': f'{pose.orientation.z:.9f}',
                'ee_qw': f'{pose.orientation.w:.9f}',
            })
        for name, value in zip(self.joint_names, self.current_joint_positions()):
            row[f'{name}_position'] = value
        return row

    def run(self):
        output_dir = os.path.dirname(os.path.abspath(self.args.output_csv))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        fieldnames = [
            'wall_time_sec', 'sim_time_sec', 'phase', 'command_joint',
            'command_velocity', 'target_detected', 'detection_count',
            'base_x', 'base_y', 'base_z',
            'base_qx', 'base_qy', 'base_qz', 'base_qw',
            'ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw',
        ] + [f'{name}_position' for name in self.joint_names]
        joint_index = self.joint_names.index(self.args.joint_name)
        command = [0.0] * len(self.joint_names)
        sample_period = 1.0 / self.args.sample_hz
        start = time.monotonic()
        next_sample = start
        sampled_rows = []
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            while time.monotonic() - start < self.args.duration_sec and rclpy.ok():
                elapsed = time.monotonic() - start
                if elapsed < self.args.pre_sec:
                    phase = 'pre'
                    command_value = 0.0
                elif elapsed < self.args.pre_sec + self.args.pulse_duration_sec:
                    phase = 'pulse'
                    command_value = self.args.velocity
                else:
                    phase = 'post'
                    command_value = 0.0
                command[joint_index] = command_value
                self.publish_command(command)
                rclpy.spin_once(self, timeout_sec=0.005)
                if time.monotonic() >= next_sample:
                    row = self.sample_row(phase, command_value)
                    writer.writerow(row)
                    sampled_rows.append(row)
                    handle.flush()
                    next_sample += sample_period
        self.publish_command([0.0] * len(self.joint_names))
        return self.summarize(sampled_rows)

    def summarize(self, rows):
        base_positions = []
        base_quaternions = []
        joint_positions = []
        for row in rows:
            if all(row[key] != '' for key in ('base_x', 'base_y', 'base_z')):
                base_positions.append(tuple(
                    float(row[key]) for key in ('base_x', 'base_y', 'base_z')))
            if all(row[key] != '' for key in (
                    'base_qx', 'base_qy', 'base_qz', 'base_qw')):
                base_quaternions.append(tuple(float(row[key]) for key in (
                    'base_qx', 'base_qy', 'base_qz', 'base_qw')))
            value = row.get(f'{self.args.joint_name}_position', '')
            if value != '':
                joint_positions.append(float(value))

        max_base_displacement = None
        if base_positions:
            initial = base_positions[0]
            max_base_displacement = max(
                math.dist(position, initial) for position in base_positions)
        max_base_orientation_error = None
        if base_quaternions:
            initial = base_quaternions[0]
            errors = []
            for quaternion in base_quaternions:
                dot = abs(sum(a * b for a, b in zip(initial, quaternion)))
                errors.append(2.0 * math.acos(min(1.0, max(-1.0, dot))))
            max_base_orientation_error = max(errors)
        joint_motion = None
        if joint_positions:
            joint_motion = max(joint_positions) - min(joint_positions)

        passed = (
            max_base_displacement is not None
            and max_base_displacement <= self.args.max_base_displacement_m
            and max_base_orientation_error is not None
            and max_base_orientation_error <= self.args.max_base_orientation_error_rad
            and joint_motion is not None
            and joint_motion >= self.args.min_joint_motion_rad)
        print('Phase 4 base plant pulse summary')
        print(f'  samples: {len(rows)}')
        print(f'  max_base_displacement_m: {max_base_displacement}')
        print(f'  max_base_orientation_error_rad: {max_base_orientation_error}')
        print(f'  commanded_joint_motion_rad: {joint_motion}')
        print(f'  base_stable_gate: {str(passed).lower()}')
        print(f'  csv: {self.args.output_csv}')
        return passed


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Phase4.2 joint pulse recorder for fixed/moving base comparison')
    parser.add_argument('--output-csv', default='/tmp/phase4_2_joint_pulse.csv')
    parser.add_argument('--joint-name', default='joint2',
                        choices=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
    parser.add_argument('--velocity', type=float, default=0.02)
    parser.add_argument('--pulse-duration-sec', type=float, default=0.5)
    parser.add_argument('--pre-sec', type=float, default=0.5)
    parser.add_argument('--duration-sec', type=float, default=3.0)
    parser.add_argument('--sample-hz', type=float, default=20.0)
    parser.add_argument('--command-topic', default='/arm_velocity_controller/commands')
    parser.add_argument('--joint-states-topic', default='/joint_states')
    parser.add_argument('--link-states-topic', default='/link_states')
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--base-link-name', default='windylab_arm::base_link')
    parser.add_argument('--ee-link-name', default='windylab_arm::link6')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--max-base-displacement-m', type=float, default=0.001)
    parser.add_argument(
        '--max-base-orientation-error-rad', type=float, default=0.01)
    parser.add_argument('--min-joint-motion-rad', type=float, default=0.005)
    parser.add_argument('--require-base-stable', action='store_true')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args(argv)
    if args.duration_sec <= 0.0:
        raise SystemExit('--duration-sec must be positive')
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')
    if args.max_base_displacement_m <= 0.0:
        raise SystemExit('--max-base-displacement-m must be positive')
    if args.max_base_orientation_error_rad <= 0.0:
        raise SystemExit('--max-base-orientation-error-rad must be positive')
    if args.min_joint_motion_rad <= 0.0:
        raise SystemExit('--min-joint-motion-rad must be positive')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = Phase4JointPulseCheck(args)
    passed = False
    try:
        passed = node.run()
    finally:
        try:
            node.publish_command([0.0] * 6)
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    if args.require_base_stable and not passed:
        raise SystemExit('base plant pulse gate failed')


if __name__ == '__main__':
    main()
