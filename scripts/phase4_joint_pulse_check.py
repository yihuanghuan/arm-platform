#!/usr/bin/env python3

import argparse
import csv
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
            'ee_x': '',
            'ee_y': '',
            'ee_z': '',
            'ee_qx': '',
            'ee_qy': '',
            'ee_qz': '',
            'ee_qw': '',
        }
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
            'ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw',
        ] + [f'{name}_position' for name in self.joint_names]
        joint_index = self.joint_names.index(self.args.joint_name)
        command = [0.0] * len(self.joint_names)
        sample_period = 1.0 / self.args.sample_hz
        start = time.monotonic()
        next_sample = start
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
                    writer.writerow(self.sample_row(phase, command_value))
                    handle.flush()
                    next_sample += sample_period
        self.publish_command([0.0] * len(self.joint_names))


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
    parser.add_argument('--ee-link-name', default='windylab_arm::link6')
    parser.add_argument('--tag-family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    parser.add_argument('--use-sim-time', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = Phase4JointPulseCheck(args)
    try:
        node.run()
    finally:
        try:
            node.publish_command([0.0] * 6)
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
