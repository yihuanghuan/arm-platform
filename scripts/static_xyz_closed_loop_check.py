#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


def parse_vector(value, expected_length, name):
    parts = [float(part) for part in value.replace(',', ' ').split()]
    if len(parts) != expected_length:
        raise argparse.ArgumentTypeError(
            f'{name} must contain {expected_length} values, got {len(parts)}')
    if any(not math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError(f'{name} contains a non-finite value')
    return parts


def norm3(values):
    return math.sqrt(sum(float(value) * float(value) for value in values[:3]))


class StaticXyzClosedLoopCheck(Node):
    def __init__(self, args):
        super().__init__(
            'static_xyz_closed_loop_check',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.latest_error = None
        self.latest_dq = None
        self.latest_target = None
        self.visual_valid = False
        self.samples = []

        self.command_pub = self.create_publisher(
            Float64MultiArray, args.command_topic, 10)
        self.create_subscription(
            Float64MultiArray,
            args.error_topic,
            self.error_callback,
            20)
        self.create_subscription(
            Float64MultiArray,
            args.dq_topic,
            self.dq_callback,
            20)
        self.create_subscription(
            Float64MultiArray,
            args.target_topic,
            self.target_callback,
            20)
        self.create_subscription(
            Bool,
            args.visual_valid_topic,
            self.visual_valid_callback,
            20)

    def error_callback(self, msg):
        if len(msg.data) >= 6:
            self.latest_error = [float(value) for value in msg.data[:6]]

    def dq_callback(self, msg):
        if len(msg.data) >= self.args.joint_count:
            self.latest_dq = [float(value) for value in msg.data[:self.args.joint_count]]

    def target_callback(self, msg):
        if len(msg.data) >= 7:
            self.latest_target = [float(value) for value in msg.data[:7]]

    def visual_valid_callback(self, msg):
        self.visual_valid = bool(msg.data)

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.initial_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.latest_error is not None
                and self.latest_dq is not None
                and self.latest_target_is_locked()
                and self.visual_valid
            ):
                return
        raise RuntimeError('Timed out waiting for visual stabilization outputs')

    def latest_target_is_locked(self):
        if self.latest_target is None:
            return False
        return any(abs(value) > 1e-9 for value in self.latest_target[:3])

    def publish_velocity(self, velocities):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in velocities]
        self.command_pub.publish(msg)

    def publish_pulse(self):
        period = 1.0 / self.args.pulse_rate_hz
        deadline = time.monotonic() + self.args.pulse_duration_sec
        next_publish = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publish_velocity(self.args.pulse_velocity)
                next_publish += period
            rclpy.spin_once(self, timeout_sec=0.002)

        zero = [0.0] * self.args.joint_count
        settle_deadline = time.monotonic() + self.args.zero_after_pulse_sec
        while rclpy.ok() and time.monotonic() < settle_deadline:
            self.publish_velocity(zero)
            rclpy.spin_once(self, timeout_sec=period)

    def monitor(self):
        start = time.monotonic()
        deadline = start + self.args.monitor_duration_sec
        period = 1.0 / self.args.sample_hz
        next_sample = time.monotonic()
        next_command = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            elapsed_sec = now - start
            if self.args.publish_pulse and now >= next_command:
                self.publish_scheduled_pulse_command(elapsed_sec)
                next_command += 1.0 / self.args.pulse_rate_hz
            if now < next_sample:
                rclpy.spin_once(self, timeout_sec=min(0.01, next_sample - now))
                continue
            next_sample += period
            rclpy.spin_once(self, timeout_sec=0.001)
            self.sample_once(elapsed_sec)

    def publish_scheduled_pulse_command(self, elapsed_sec):
        pulse_start = self.args.pre_pulse_monitor_sec
        pulse_end = pulse_start + self.args.pulse_duration_sec
        zero_end = pulse_end + self.args.zero_after_pulse_sec
        if elapsed_sec < pulse_start:
            return
        if elapsed_sec < pulse_end:
            self.publish_velocity(self.args.pulse_velocity)
            return
        if elapsed_sec < zero_end:
            self.publish_velocity([0.0] * self.args.joint_count)

    def sample_once(self, elapsed_sec):
        error = self.latest_error if self.latest_error is not None else [0.0] * 6
        dq = self.latest_dq if self.latest_dq is not None else [0.0] * self.args.joint_count
        row = {
            'elapsed_sec': f'{elapsed_sec:.9f}',
            'visual_valid': self.visual_valid,
            'error_x_m': f'{error[0]:.9f}',
            'error_y_m': f'{error[1]:.9f}',
            'error_z_m': f'{error[2]:.9f}',
            'error_norm_m': f'{norm3(error):.9f}',
            'max_abs_joint_velocity_rad_s': f'{max(abs(value) for value in dq):.9f}',
        }
        for index, value in enumerate(dq, start=1):
            row[f'dq{index}_rad_s'] = f'{value:.9f}'
        self.samples.append(row)

    def run(self):
        self.wait_for_controller()
        if self.args.publish_pulse:
            self.get_logger().info(
                f'Will publish pulse velocity for {self.args.pulse_duration_sec:.3f} s '
                f'after {self.args.pre_pulse_monitor_sec:.3f} s of monitoring')
        self.monitor()

    def write_csv(self):
        if not self.args.output_csv:
            return
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'elapsed_sec',
            'visual_valid',
            'error_x_m',
            'error_y_m',
            'error_z_m',
            'error_norm_m',
            'max_abs_joint_velocity_rad_s',
        ] + [f'dq{index}_rad_s' for index in range(1, self.args.joint_count + 1)]
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)
        print(f'  csv: {self.args.output_csv}')

    def print_summary(self):
        print('Static XYZ closed-loop summary')
        print(f'  samples: {len(self.samples)}')
        if not self.samples:
            return
        errors = [float(sample['error_norm_m']) for sample in self.samples]
        velocities = [
            float(sample['max_abs_joint_velocity_rad_s'])
            for sample in self.samples
        ]
        valid_count = sum(1 for sample in self.samples if sample['visual_valid'])
        tail_count = max(1, int(round(self.args.steady_window_sec * self.args.sample_hz)))
        tail_errors = errors[-tail_count:]
        initial_error = max(errors[:max(1, min(len(errors), tail_count))])
        steady_mean = statistics.mean(tail_errors)
        print(f'  visual_valid_fraction: {valid_count / len(self.samples):.3f}')
        print(f'  initial_error_max_m: {initial_error:.6f}')
        print(f'  error_max_m: {max(errors):.6f}')
        print(f'  steady_error_mean_m: {steady_mean:.6f}')
        print(f'  steady_error_max_m: {max(tail_errors):.6f}')
        print(f'  max_abs_joint_velocity_rad_s: {max(velocities):.6f}')
        passed = (
            steady_mean <= self.args.steady_error_threshold_m
            and max(velocities) <= self.args.max_joint_velocity + 1e-6
            and valid_count == len(self.samples)
        )
        print(f'  pass: {str(passed).lower()}')
        if not passed:
            raise RuntimeError('Static XYZ closed-loop acceptance check failed')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Apply a small joint-velocity pulse and check static XYZ visual closure.')
    parser.add_argument('--command-topic', default='/arm_velocity_controller/commands')
    parser.add_argument('--error-topic', default='/visual_stabilization/error')
    parser.add_argument('--dq-topic', default='/visual_stabilization/dq_limited')
    parser.add_argument('--target-topic', default='/visual_stabilization/target_pose')
    parser.add_argument('--visual-valid-topic', default='/visual_ee_pose_valid')
    parser.add_argument('--joint-count', type=int, default=6)
    parser.add_argument(
        '--pulse-velocity',
        type=lambda value: parse_vector(value, 6, 'pulse velocity'),
        default=parse_vector('0.0 0.10 -0.08 0.0 0.0 0.0', 6, 'pulse velocity'),
        help='Six joint velocity values in rad/s.')
    parser.add_argument('--pulse-duration-sec', type=float, default=0.35)
    parser.add_argument('--pre-pulse-monitor-sec', type=float, default=0.5)
    parser.add_argument('--pulse-rate-hz', type=float, default=100.0)
    parser.add_argument('--zero-after-pulse-sec', type=float, default=0.5)
    parser.add_argument('--monitor-duration-sec', type=float, default=8.0)
    parser.add_argument('--sample-hz', type=float, default=20.0)
    parser.add_argument('--steady-window-sec', type=float, default=2.0)
    parser.add_argument('--steady-error-threshold-m', type=float, default=0.010)
    parser.add_argument('--max-joint-velocity', type=float, default=0.35)
    parser.add_argument('--initial-timeout-sec', type=float, default=12.0)
    parser.add_argument('--output-csv', default='/tmp/windylab_phase3_static_xyz.csv')
    parser.add_argument('--no-pulse', dest='publish_pulse', action='store_false')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.set_defaults(publish_pulse=True)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.joint_count != 6:
        raise SystemExit('--joint-count must be 6 for the current manipulator')
    if args.pulse_duration_sec < 0.0:
        raise SystemExit('--pulse-duration-sec must be non-negative')
    if args.pre_pulse_monitor_sec < 0.0:
        raise SystemExit('--pre-pulse-monitor-sec must be non-negative')
    if args.pulse_rate_hz <= 0.0:
        raise SystemExit('--pulse-rate-hz must be positive')
    if args.zero_after_pulse_sec < 0.0:
        raise SystemExit('--zero-after-pulse-sec must be non-negative')
    if args.monitor_duration_sec <= 0.0:
        raise SystemExit('--monitor-duration-sec must be positive')
    if args.sample_hz <= 0.0:
        raise SystemExit('--sample-hz must be positive')

    rclpy.init()
    node = StaticXyzClosedLoopCheck(args)
    try:
        node.run()
        node.print_summary()
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
