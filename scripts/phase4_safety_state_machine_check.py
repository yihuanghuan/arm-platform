#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


class Phase4SafetyStateMachineCheck(Node):
    def __init__(self, args):
        super().__init__('phase4_safety_state_machine_check')
        self.args = args
        self.joint_names = [f'joint{index}' for index in range(1, 7)]
        self.joint_pub = self.create_publisher(
            JointState, args.joint_state_topic, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, args.visual_pose_topic, 10)
        self.valid_pub = self.create_publisher(
            Bool, args.visual_valid_topic, 10)
        self.create_subscription(
            Float64MultiArray,
            args.command_topic,
            self.command_callback,
            100)
        self.create_subscription(
            String,
            args.status_topic,
            self.status_callback,
            100)
        self.commands = []
        self.statuses = []
        self.status_json_errors = 0
        self.phase_rows = []

    def command_callback(self, message):
        values = [float(value) for value in message.data]
        peak = max((abs(value) for value in values), default=0.0)
        self.commands.append((time.monotonic(), peak, values))

    def status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.status_json_errors += 1
            return
        self.statuses.append((time.monotonic(), payload))

    def publish_joint_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = [0.0] * 6
        message.velocity = [0.0] * 6
        self.joint_pub.publish(message)

    def publish_visual(self, position_x, valid):
        if position_x is not None:
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'world'
            message.pose.position.x = float(position_x)
            message.pose.position.y = 0.00195
            message.pose.position.z = 0.374311
            message.pose.orientation.w = 1.0
            if valid:
                self.pose_pub.publish(message)
                self.valid_pub.publish(Bool(data=True))
            else:
                self.valid_pub.publish(Bool(data=False))
                self.pose_pub.publish(message)
        elif valid is not None:
            self.valid_pub.publish(Bool(data=bool(valid)))

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.controller_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.commands and self.statuses:
                return
        raise RuntimeError(
            'Timed out waiting for controller command and status topics')

    def run_phase(
            self,
            name,
            duration_sec,
            publish_joints,
            visual_x=None,
            visual_valid=None,
            visual_once=False):
        start = time.monotonic()
        end = start + duration_sec
        command_start_index = len(self.commands)
        status_start_index = len(self.statuses)
        next_joint = start
        next_visual = start
        visual_published = False
        joint_period = 1.0 / self.args.joint_publish_hz
        visual_period = 1.0 / self.args.visual_publish_hz

        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            if publish_joints and now >= next_joint:
                self.publish_joint_state()
                next_joint += joint_period
            should_publish_visual = (
                visual_valid is not None
                and now >= next_visual
                and (not visual_once or not visual_published))
            if should_publish_visual:
                self.publish_visual(visual_x, visual_valid)
                visual_published = True
                next_visual += visual_period
            rclpy.spin_once(self, timeout_sec=0.005)

        self.spin_for(0.03)
        phase_end = time.monotonic()
        commands = self.commands[command_start_index:]
        statuses = self.statuses[status_start_index:]
        tail_start = phase_end - min(0.05, duration_sec * 0.4)
        tail_commands = [peak for stamp, peak, _ in commands if stamp >= tail_start]
        all_command_peaks = [peak for _, peak, _ in commands]
        status = statuses[-1][1] if statuses else {}
        safety_state = status.get('safety_state', '')
        state_entry_stamp = next((
            stamp for stamp, payload in statuses
            if payload.get('safety_state', '') == safety_state), None)
        sustained_zero_stamp = None
        if state_entry_stamp is not None:
            commands_after_entry = [
                (stamp, peak) for stamp, peak, _ in commands
                if stamp >= state_entry_stamp]
            for index, (stamp, _) in enumerate(commands_after_entry):
                if all(
                        peak <= self.args.zero_command_threshold_rad_s
                        for _, peak in commands_after_entry[index:]):
                    sustained_zero_stamp = stamp
                    break
        state_entry_to_zero_latency_sec = math.nan
        if sustained_zero_stamp is not None:
            state_entry_to_zero_latency_sec = max(
                0.0, sustained_zero_stamp - state_entry_stamp)
        row = {
            'phase': name,
            'duration_sec': duration_sec,
            'command_samples': len(commands),
            'command_peak_rad_s': max(all_command_peaks, default=math.nan),
            'tail_command_peak_rad_s': max(tail_commands, default=math.nan),
            'status_samples': len(statuses),
            'safety_state': safety_state,
            'status': status.get('status', ''),
            'target_locked': status.get('target_locked'),
            'target_lock_count': status.get('target_lock_count'),
            'target_reset_count': status.get('target_reset_count'),
            'safety_stop': status.get('safety_stop'),
            'safety_stop_reason': status.get('safety_stop_reason', ''),
            'state_entry_to_zero_latency_sec': state_entry_to_zero_latency_sec,
        }
        self.phase_rows.append(row)
        return row

    @staticmethod
    def is_zero(row, threshold):
        peak = row['tail_command_peak_rad_s']
        return math.isfinite(peak) and peak <= threshold

    @staticmethod
    def is_nonzero(row, threshold):
        peak = row['command_peak_rad_s']
        return math.isfinite(peak) and peak >= threshold

    @staticmethod
    def hard_zero_within(row, timeout_sec=0.05):
        latency = row['state_entry_to_zero_latency_sec']
        return math.isfinite(latency) and latency <= timeout_sec

    def execute(self):
        self.wait_for_controller()
        zero = self.args.zero_command_threshold_rad_s
        nonzero = self.args.nonzero_command_threshold_rad_s
        checks = []

        waiting = self.run_phase('waiting_no_inputs', 0.25, False)
        checks.append((
            'waiting_no_inputs',
            self.is_zero(waiting, zero)
            and waiting['safety_state'] == 'WAITING_FOR_INPUTS'))

        joint_only = self.run_phase('waiting_joint_only', 0.25, True)
        checks.append((
            'waiting_joint_only',
            self.is_zero(joint_only, zero)
            and joint_only['safety_state'] == 'WAITING_FOR_INPUTS'))

        warm_1 = self.run_phase(
            'warmup_1', 0.12, True, self.args.initial_x, True, True)
        warm_2 = self.run_phase(
            'warmup_2', 0.12, True, self.args.initial_x, True, True)
        checks.append((
            'warmup_requires_three',
            self.is_zero(warm_1, zero)
            and self.is_zero(warm_2, zero)
            and warm_2['safety_state'] == 'WARMING_UP'
            and warm_2['target_lock_count'] == 0))

        locked = self.run_phase(
            'lock_third_pose', 0.15, True, self.args.initial_x, True, True)
        checks.append((
            'lock_on_third_pose',
            self.is_zero(locked, zero)
            and locked['safety_state'] == 'TRACKING'
            and locked['target_lock_count'] == 1
            and locked['target_reset_count'] == 0))

        tracking = self.run_phase(
            'tracking_error',
            0.45,
            True,
            self.args.initial_x + self.args.tracking_error_m,
            True)
        checks.append((
            'tracking_nonzero',
            self.is_nonzero(tracking, nonzero)
            and tracking['safety_state'] == 'TRACKING'
            and not tracking['safety_stop']))

        invalid = self.run_phase(
            'short_visual_invalid_with_pose',
            0.20,
            True,
            self.args.initial_x + self.args.large_error_m,
            False)
        checks.append((
            'invalid_pose_hard_zero_preserves_target',
            self.is_zero(invalid, zero)
            and self.hard_zero_within(invalid)
            and invalid['safety_state'] == 'HOLDING_INPUT_LOSS'
            and invalid['target_locked']
            and invalid['target_lock_count'] == 1
            and invalid['target_reset_count'] == 0
            and not invalid['safety_stop']))

        recovered = self.run_phase(
            'short_visual_recovery',
            0.40,
            True,
            self.args.initial_x + self.args.tracking_error_m,
            True)
        checks.append((
            'short_visual_recovery',
            self.is_nonzero(recovered, nonzero)
            and recovered['safety_state'] == 'TRACKING'
            and recovered['target_lock_count'] == 1
            and recovered['target_reset_count'] == 0))

        joint_timeout = self.run_phase(
            'joint_state_timeout',
            0.35,
            False,
            self.args.initial_x + self.args.tracking_error_m,
            True)
        checks.append((
            'joint_timeout_hard_zero_preserves_target',
            self.is_zero(joint_timeout, zero)
            and self.hard_zero_within(joint_timeout)
            and joint_timeout['safety_state'] == 'HOLDING_INPUT_LOSS'
            and joint_timeout['target_locked']
            and not joint_timeout['safety_stop']))

        joint_recovered = self.run_phase(
            'joint_state_recovery',
            0.40,
            True,
            self.args.initial_x + self.args.tracking_error_m,
            True)
        checks.append((
            'joint_state_recovery',
            self.is_nonzero(joint_recovered, nonzero)
            and joint_recovered['safety_state'] == 'TRACKING'
            and joint_recovered['target_lock_count'] == 1))

        visual_timeout = self.run_phase(
            'visual_pose_timeout', 0.38, True)
        checks.append((
            'visual_timeout_hard_zero_preserves_target',
            self.is_zero(visual_timeout, zero)
            and self.hard_zero_within(visual_timeout)
            and visual_timeout['safety_state'] == 'HOLDING_INPUT_LOSS'
            and visual_timeout['target_locked']
            and visual_timeout['target_lock_count'] == 1
            and visual_timeout['target_reset_count'] == 0
            and not visual_timeout['safety_stop']))

        visual_timeout_recovered = self.run_phase(
            'visual_pose_timeout_recovery',
            0.40,
            True,
            self.args.initial_x + self.args.tracking_error_m,
            True)
        checks.append((
            'visual_pose_timeout_recovery',
            self.is_nonzero(visual_timeout_recovered, nonzero)
            and visual_timeout_recovered['safety_state'] == 'TRACKING'
            and visual_timeout_recovered['target_lock_count'] == 1
            and visual_timeout_recovered['target_reset_count'] == 0))

        long_loss = self.run_phase(
            'long_visual_loss', 0.45, True, None, False)
        checks.append((
            'long_visual_loss_resets_target',
            self.is_zero(long_loss, zero)
            and self.hard_zero_within(long_loss)
            and long_loss['safety_state'] == 'HOLDING_INPUT_LOSS'
            and not long_loss['target_locked']
            and long_loss['target_lock_count'] == 1
            and long_loss['target_reset_count'] == 1
            and not long_loss['safety_stop']))

        relock_x = self.args.initial_x + self.args.relock_offset_m
        relock_1 = self.run_phase(
            'relock_warmup_1', 0.12, True, relock_x, True, True)
        relock_2 = self.run_phase(
            'relock_warmup_2', 0.12, True, relock_x, True, True)
        relocked = self.run_phase(
            'relock_third_pose', 0.15, True, relock_x, True, True)
        checks.append((
            'relock_requires_three',
            self.is_zero(relock_1, zero)
            and self.is_zero(relock_2, zero)
            and relock_2['safety_state'] == 'WARMING_UP'
            and relocked['safety_state'] == 'TRACKING'
            and relocked['target_lock_count'] == 2
            and relocked['target_reset_count'] == 1))

        before_fault = self.run_phase(
            'tracking_before_fault',
            0.35,
            True,
            relock_x + self.args.tracking_error_m,
            True)
        checks.append((
            'tracking_before_fault', self.is_nonzero(before_fault, nonzero)))

        fault = self.run_phase(
            'large_error_fault',
            0.20,
            True,
            relock_x + self.args.large_error_m,
            True)
        checks.append((
            'large_error_fault_latched_hard_zero',
            self.is_zero(fault, zero)
            and self.hard_zero_within(fault)
            and fault['safety_state'] == 'FAULT_LATCHED'
            and fault['safety_stop']))

        fault_recovery = self.run_phase(
            'fault_recovery_rejected', 0.30, True, relock_x, True)
        checks.append((
            'fault_requires_restart',
            self.is_zero(fault_recovery, zero)
            and self.hard_zero_within(fault_recovery)
            and fault_recovery['safety_state'] == 'FAULT_LATCHED'
            and fault_recovery['safety_stop']
            and fault_recovery['target_lock_count'] == 2))

        checks.append(('status_json_valid', self.status_json_errors == 0))
        return checks

    def write_csv(self):
        output_dir = os.path.dirname(os.path.abspath(self.args.output_csv))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        fieldnames = [
            'phase', 'duration_sec', 'command_samples', 'command_peak_rad_s',
            'tail_command_peak_rad_s', 'status_samples', 'safety_state', 'status',
            'target_locked', 'target_lock_count', 'target_reset_count',
            'safety_stop', 'safety_stop_reason',
            'state_entry_to_zero_latency_sec',
        ]
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.phase_rows:
                writer.writerow(row)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Deterministic phase 4 visual-controller safety state-machine check.')
    parser.add_argument(
        '--output-csv', default='/tmp/phase4_safety_state_machine.csv')
    parser.add_argument('--joint-state-topic', default='/phase4_safety/joint_states')
    parser.add_argument('--visual-pose-topic', default='/phase4_safety/visual_pose')
    parser.add_argument('--visual-valid-topic', default='/phase4_safety/visual_valid')
    parser.add_argument('--command-topic', default='/phase4_safety/commands')
    parser.add_argument('--status-topic', default='/visual_stabilization/status')
    parser.add_argument('--controller-timeout-sec', type=float, default=5.0)
    parser.add_argument('--joint-publish-hz', type=float, default=50.0)
    parser.add_argument('--visual-publish-hz', type=float, default=20.0)
    parser.add_argument('--initial-x', type=float, default=0.353286)
    parser.add_argument('--tracking-error-m', type=float, default=0.03)
    parser.add_argument('--relock-offset-m', type=float, default=0.05)
    parser.add_argument('--large-error-m', type=float, default=0.25)
    parser.add_argument('--zero-command-threshold-rad-s', type=float, default=1e-9)
    parser.add_argument('--nonzero-command-threshold-rad-s', type=float, default=1e-3)
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args(argv)
    if args.controller_timeout_sec <= 0.0:
        raise SystemExit('--controller-timeout-sec must be positive')
    if args.joint_publish_hz <= 0.0 or args.visual_publish_hz <= 0.0:
        raise SystemExit('publish rates must be positive')
    if args.zero_command_threshold_rad_s < 0.0:
        raise SystemExit('--zero-command-threshold-rad-s must be non-negative')
    if args.nonzero_command_threshold_rad_s <= args.zero_command_threshold_rad_s:
        raise SystemExit('nonzero threshold must exceed zero threshold')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = Phase4SafetyStateMachineCheck(args)
    checks = []
    try:
        checks = node.execute()
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('Phase 4 safety state-machine checks')
    for name, passed in checks:
        print(f'  {name}: {str(passed).lower()}')
    passed = bool(checks) and all(result for _, result in checks)
    print(f'  overall_passed: {str(passed).lower()}')
    print(f'  csv: {args.output_csv}')
    if args.require_pass and not passed:
        raise SystemExit('safety state-machine gate failed')


if __name__ == '__main__':
    main()
