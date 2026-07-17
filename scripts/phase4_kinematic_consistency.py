#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time

from gazebo_msgs.msg import LinkStates
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [f'joint{index}' for index in range(1, 7)]


def default_urdf_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(
            os.path.dirname(os.path.dirname(script_dir)),
            'share',
            'manipulator',
            'arm.urdf'),
        os.path.join(os.path.dirname(script_dir), 'config', 'arm.urdf'),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


def parse_vector(value):
    try:
        result = [
            float(part)
            for part in value.replace(',', ' ').replace(';', ' ').split()
        ]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    if len(result) != 6 or not all(math.isfinite(item) for item in result):
        raise argparse.ArgumentTypeError('configuration must contain six finite values')
    return result


def normalize_quaternion_xyzw(values):
    quaternion = np.asarray(values, dtype=float)
    magnitude = float(np.linalg.norm(quaternion))
    if magnitude <= 1e-12:
        raise ValueError('zero-norm quaternion')
    return quaternion / magnitude


def pose_matrix(pose):
    qx, qy, qz, qw = normalize_quaternion_xyzw([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ])
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = pin.Quaternion(qw, qx, qy, qz).toRotationMatrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def rotation_angle(rotation):
    cosine = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.acos(min(1.0, max(-1.0, cosine)))


def serialize(values):
    return ';'.join(f'{float(value):.9g}' for value in values)


class KinematicConsistencyCheck(Node):
    def __init__(self, args):
        super().__init__(
            'phase4_kinematic_consistency',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.model = pin.buildModelFromUrdf(args.urdf_path)
        self.data = self.model.createData()
        if not self.model.existFrame(args.ee_frame):
            raise ValueError(f'End-effector frame not found: {args.ee_frame}')
        self.ee_frame_id = self.model.getFrameId(args.ee_frame)
        self.q_indices = []
        for name in JOINT_NAMES:
            if not self.model.existJointName(name):
                raise ValueError(f'Joint not found in URDF: {name}')
            joint_id = self.model.getJointId(name)
            self.q_indices.append(self.model.joints[joint_id].idx_q)

        self.q_model = pin.neutral(self.model)
        self.joint_positions = None
        self.joint_state_received_wall = None
        self.base_to_ee = None
        self.link_state_sequence = 0
        self.command_pub = self.create_publisher(
            Float64MultiArray, args.command_topic, 20)
        self.create_subscription(
            JointState, args.joint_states_topic, self.joint_state_callback, 1)
        self.create_subscription(
            LinkStates, args.link_states_topic, self.link_states_callback, 1)

    def joint_state_callback(self, message):
        position_by_name = dict(zip(message.name, message.position))
        if not all(name in position_by_name for name in JOINT_NAMES):
            return
        values = np.array(
            [float(position_by_name[name]) for name in JOINT_NAMES], dtype=float)
        if not np.all(np.isfinite(values)):
            return
        self.joint_positions = values
        self.joint_state_received_wall = time.monotonic()

    def link_states_callback(self, message):
        try:
            base_index = message.name.index(self.args.base_link_name)
            ee_index = message.name.index(self.args.ee_link_name)
        except ValueError:
            return
        if base_index >= len(message.pose) or ee_index >= len(message.pose):
            return
        world_to_base = pose_matrix(message.pose[base_index])
        world_to_ee = pose_matrix(message.pose[ee_index])
        self.base_to_ee = np.linalg.inv(world_to_base) @ world_to_ee
        self.link_state_sequence += 1

    def publish_command(self, values):
        message = Float64MultiArray()
        message.data = [float(value) for value in values]
        self.command_pub.publish(message)

    def sim_time_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                    self.sim_time_sec() > 0.0
                    and self.joint_positions is not None
                    and self.base_to_ee is not None):
                return
        raise RuntimeError('Timed out waiting for joint and Gazebo link states')

    def run_for_sim_duration(self, command, duration_sec):
        start_sim = self.sim_time_sec()
        deadline = time.monotonic() + self.args.operation_timeout_sec
        while rclpy.ok() and self.sim_time_sec() - start_sim < duration_sec:
            if time.monotonic() >= deadline:
                raise RuntimeError('Simulation time did not advance during command')
            self.publish_command(command)
            rclpy.spin_once(self, timeout_sec=0.002)

    def settle(self):
        self.run_for_sim_duration(np.zeros(6, dtype=float), self.args.settle_sec)

    def stop(self):
        deadline = time.monotonic() + self.args.stop_publish_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(np.zeros(6, dtype=float))
            rclpy.spin_once(self, timeout_sec=0.002)

    def drive_to_configuration(self, target):
        target = np.asarray(target, dtype=float)
        start_sim = self.sim_time_sec()
        progress_deadline = time.monotonic() + self.args.operation_timeout_sec
        previous_sim = start_sim
        stable_updates = 0
        previous_sequence = self.link_state_sequence
        while (
                rclpy.ok()
                and self.sim_time_sec() - start_sim < self.args.drive_timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.002)
            current_sim = self.sim_time_sec()
            if current_sim > previous_sim:
                progress_deadline = (
                    time.monotonic() + self.args.operation_timeout_sec)
                previous_sim = current_sim
            elif time.monotonic() >= progress_deadline:
                self.stop()
                raise RuntimeError(
                    'Simulation time did not advance while driving to configuration')
            if self.joint_positions is None:
                continue
            if (
                    self.joint_state_received_wall is None
                    or time.monotonic() - self.joint_state_received_wall
                    > self.args.max_joint_state_wall_age_sec):
                self.publish_command(np.zeros(6, dtype=float))
                continue
            error = target - self.joint_positions
            if float(np.max(np.abs(error))) <= self.args.configuration_tolerance_rad:
                if self.link_state_sequence != previous_sequence:
                    stable_updates += 1
                    previous_sequence = self.link_state_sequence
                self.publish_command(np.zeros(6, dtype=float))
                if stable_updates >= 3:
                    self.settle()
                    return
                continue
            stable_updates = 0
            command = np.clip(
                self.args.drive_gain * error,
                -self.args.drive_max_velocity,
                self.args.drive_max_velocity)
            self.publish_command(command)
        self.stop()
        raise RuntimeError(
            'Timed out in simulation time while driving to configuration; max error='
            f'{float(np.max(np.abs(target - self.joint_positions))):.6f} rad')

    def pin_base_to_ee(self, joint_positions):
        q = pin.neutral(self.model)
        for value, q_index in zip(joint_positions, self.q_indices):
            q[q_index] = float(value)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.ee_frame_id]
        result = np.eye(4, dtype=float)
        result[:3, :3] = placement.rotation
        result[:3, 3] = placement.translation
        return result

    def snapshot(self):
        if self.joint_positions is None or self.base_to_ee is None:
            raise RuntimeError('State unavailable during snapshot')
        return self.joint_positions.copy(), self.base_to_ee.copy()

    def evaluate_pulse(self, configuration_index, joint_index, sign, target):
        self.drive_to_configuration(target)
        before_q, before_gazebo = self.snapshot()
        command = np.zeros(6, dtype=float)
        command[joint_index] = sign * self.args.pulse_velocity
        self.run_for_sim_duration(command, self.args.pulse_duration_sec)
        self.settle()
        after_q, after_gazebo = self.snapshot()

        before_pin = self.pin_base_to_ee(before_q)
        after_pin = self.pin_base_to_ee(after_q)
        joint_delta = float(after_q[joint_index] - before_q[joint_index])
        pin_delta = after_pin[:3, 3] - before_pin[:3, 3]
        gazebo_delta = after_gazebo[:3, 3] - before_gazebo[:3, 3]
        position_delta_error = float(np.linalg.norm(gazebo_delta - pin_delta))
        before_position_error = float(
            np.linalg.norm(before_gazebo[:3, 3] - before_pin[:3, 3]))
        after_position_error = float(
            np.linalg.norm(after_gazebo[:3, 3] - after_pin[:3, 3]))
        before_orientation_error = rotation_angle(
            before_pin[:3, :3].T @ before_gazebo[:3, :3])
        after_orientation_error = rotation_angle(
            after_pin[:3, :3].T @ after_gazebo[:3, :3])
        pin_rotation_delta = before_pin[:3, :3].T @ after_pin[:3, :3]
        gazebo_rotation_delta = before_gazebo[:3, :3].T @ after_gazebo[:3, :3]
        orientation_delta_error = rotation_angle(
            pin_rotation_delta.T @ gazebo_rotation_delta)

        pin_delta_norm = float(np.linalg.norm(pin_delta))
        gazebo_delta_norm = float(np.linalg.norm(gazebo_delta))
        direction_cosine = None
        scale_ratio = None
        if pin_delta_norm >= self.args.translation_direction_min_m:
            direction_cosine = float(
                np.dot(pin_delta, gazebo_delta)
                / max(pin_delta_norm * gazebo_delta_norm, 1e-15))
            scale_ratio = gazebo_delta_norm / pin_delta_norm

        passed = (
            sign * joint_delta >= self.args.min_joint_motion_rad
            and before_position_error <= self.args.max_absolute_position_error_m
            and after_position_error <= self.args.max_absolute_position_error_m
            and before_orientation_error <= self.args.max_absolute_orientation_error_rad
            and after_orientation_error <= self.args.max_absolute_orientation_error_rad
            and position_delta_error <= self.args.max_delta_position_error_m
            and orientation_delta_error <= self.args.max_delta_orientation_error_rad
            and (
                direction_cosine is None
                or direction_cosine >= self.args.min_translation_direction_cosine)
            and (
                scale_ratio is None
                or abs(scale_ratio - 1.0) <= self.args.max_translation_scale_error))

        return {
            'configuration': configuration_index,
            'joint': JOINT_NAMES[joint_index],
            'sign': '+' if sign > 0.0 else '-',
            'before_q': serialize(before_q),
            'after_q': serialize(after_q),
            'joint_delta_rad': joint_delta,
            'pin_delta_xyz': serialize(pin_delta),
            'gazebo_delta_xyz': serialize(gazebo_delta),
            'pin_delta_norm_m': pin_delta_norm,
            'gazebo_delta_norm_m': gazebo_delta_norm,
            'position_delta_error_m': position_delta_error,
            'orientation_delta_error_rad': orientation_delta_error,
            'before_position_error_m': before_position_error,
            'after_position_error_m': after_position_error,
            'before_orientation_error_rad': before_orientation_error,
            'after_orientation_error_rad': after_orientation_error,
            'translation_direction_cosine': direction_cosine,
            'translation_scale_ratio': scale_ratio,
            'pass': passed,
        }

    def run(self):
        self.wait_ready()
        rows = []
        for configuration_index, target in enumerate(self.args.configurations):
            for joint_index in range(6):
                for sign in (1.0, -1.0):
                    row = self.evaluate_pulse(
                        configuration_index, joint_index, sign, target)
                    rows.append(row)
                    self.write_csv(rows)
                    print(
                        f'config={configuration_index} joint={row["joint"]} '
                        f'sign={row["sign"]} pass={str(row["pass"]).lower()} '
                        f'joint_delta_rad={row["joint_delta_rad"]:.6g} '
                        f'position_delta_error_m={row["position_delta_error_m"]:.6g} '
                        'orientation_delta_error_rad='
                        f'{row["orientation_delta_error_rad"]:.6g} '
                        'direction_cosine='
                        f'{row["translation_direction_cosine"]} '
                        f'scale_ratio={row["translation_scale_ratio"]}')
        self.stop()
        self.write_csv(rows)
        passed_count = sum(1 for row in rows if row['pass'])
        print('Phase 4 Pinocchio/Gazebo kinematic consistency summary')
        print(f'  cases: {len(rows)}')
        print(f'  passed: {passed_count}')
        print(f'  failed: {len(rows) - passed_count}')
        print(f'  csv: {self.args.output_csv}')
        return passed_count == len(rows)

    def write_csv(self, rows):
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'configuration',
            'joint',
            'sign',
            'before_q',
            'after_q',
            'joint_delta_rad',
            'pin_delta_xyz',
            'gazebo_delta_xyz',
            'pin_delta_norm_m',
            'gazebo_delta_norm_m',
            'position_delta_error_m',
            'orientation_delta_error_rad',
            'before_position_error_m',
            'after_position_error_m',
            'before_orientation_error_rad',
            'after_orientation_error_rad',
            'translation_direction_cosine',
            'translation_scale_ratio',
            'pass',
        ]
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Compare Pinocchio FK with Gazebo base-relative link6 motion for '
            'positive and negative single-joint pulses.'))
    parser.add_argument('--output-csv', default='/tmp/phase4_kinematic_consistency.csv')
    parser.add_argument('--urdf-path', default=default_urdf_path())
    parser.add_argument('--ee-frame', default='link6')
    parser.add_argument('--base-link-name', default='windylab_arm::base_link')
    parser.add_argument('--ee-link-name', default='windylab_arm::link6')
    parser.add_argument('--joint-states-topic', default='/joint_states')
    parser.add_argument('--link-states-topic', default='/link_states')
    parser.add_argument('--command-topic', default='/arm_velocity_controller/commands')
    parser.add_argument(
        '--configuration', dest='configurations', action='append',
        type=parse_vector)
    parser.add_argument('--pulse-velocity', type=float, default=0.03)
    parser.add_argument('--pulse-duration-sec', type=float, default=1.80)
    parser.add_argument('--settle-sec', type=float, default=0.15)
    parser.add_argument('--drive-gain', type=float, default=5.0)
    parser.add_argument('--drive-max-velocity', type=float, default=0.15)
    parser.add_argument('--configuration-tolerance-rad', type=float, default=0.005)
    parser.add_argument('--ready-timeout-sec', type=float, default=15.0)
    parser.add_argument('--drive-timeout-sec', type=float, default=30.0)
    parser.add_argument('--operation-timeout-sec', type=float, default=5.0)
    parser.add_argument(
        '--max-joint-state-wall-age-sec', type=float, default=0.10)
    parser.add_argument('--stop-publish-sec', type=float, default=0.25)
    parser.add_argument('--min-joint-motion-rad', type=float, default=0.004)
    parser.add_argument('--max-absolute-position-error-m', type=float, default=0.001)
    parser.add_argument(
        '--max-absolute-orientation-error-rad', type=float, default=0.005)
    parser.add_argument('--max-delta-position-error-m', type=float, default=0.0005)
    parser.add_argument(
        '--max-delta-orientation-error-rad', type=float, default=0.005)
    parser.add_argument('--translation-direction-min-m', type=float, default=1e-4)
    parser.add_argument(
        '--min-translation-direction-cosine', type=float, default=0.98)
    parser.add_argument('--max-translation-scale-error', type=float, default=0.10)
    parser.add_argument('--require-pass', action='store_true')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args(argv)
    if args.configurations is None:
        args.configurations = [
            [0.0] * 6,
            [0.20, -0.25, 0.30, -0.15, 0.10, -0.10],
        ]
    positive_names = (
        'pulse_velocity',
        'pulse_duration_sec',
        'settle_sec',
        'drive_gain',
        'drive_max_velocity',
        'configuration_tolerance_rad',
        'ready_timeout_sec',
        'drive_timeout_sec',
        'operation_timeout_sec',
        'max_joint_state_wall_age_sec',
        'stop_publish_sec',
        'min_joint_motion_rad',
        'max_absolute_position_error_m',
        'max_absolute_orientation_error_rad',
        'max_delta_position_error_m',
        'max_delta_orientation_error_rad',
        'translation_direction_min_m',
        'max_translation_scale_error',
    )
    for name in positive_names:
        if getattr(args, name) <= 0.0:
            raise SystemExit(f'--{name.replace("_", "-")} must be positive')
    if not -1.0 <= args.min_translation_direction_cosine <= 1.0:
        raise SystemExit('--min-translation-direction-cosine must be in [-1, 1]')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = KinematicConsistencyCheck(args)
    passed = False
    try:
        passed = node.run()
    finally:
        try:
            node.stop()
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    if args.require_pass and not passed:
        raise SystemExit('Pinocchio/Gazebo kinematic consistency gate failed')


if __name__ == '__main__':
    main()
