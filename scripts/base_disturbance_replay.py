#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
import sys
import time

from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


TRAJECTORY_FIELDS = [
    'time_sec',
    'x',
    'y',
    'z',
    'qx',
    'qy',
    'qz',
    'qw',
    'linear_x',
    'linear_y',
    'linear_z',
    'angular_x',
    'angular_y',
    'angular_z',
]


def pose_position(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def pose_quaternion(pose):
    return (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def position_error(a_pose, b_pose):
    ax, ay, az = pose_position(a_pose)
    bx, by, bz = pose_position(b_pose)
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def quaternion_angle_error(a_pose, b_pose):
    aq = pose_quaternion(a_pose)
    bq = pose_quaternion(b_pose)
    dot = abs(sum(a * b for a, b in zip(aq, bq)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def make_pose(row):
    pose = Pose()
    pose.position.x = float(row['x'])
    pose.position.y = float(row['y'])
    pose.position.z = float(row['z'])
    qx, qy, qz, qw = normalize_quaternion(
        float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw']))
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def make_twist(row):
    twist = Twist()
    twist.linear.x = float(row['linear_x'])
    twist.linear.y = float(row['linear_y'])
    twist.linear.z = float(row['linear_z'])
    twist.angular.x = float(row['angular_x'])
    twist.angular.y = float(row['angular_y'])
    twist.angular.z = float(row['angular_z'])
    return twist


def serialize_pose(prefix, pose):
    if pose is None:
        return {
            f'{prefix}_x': '',
            f'{prefix}_y': '',
            f'{prefix}_z': '',
            f'{prefix}_qx': '',
            f'{prefix}_qy': '',
            f'{prefix}_qz': '',
            f'{prefix}_qw': '',
        }
    return {
        f'{prefix}_x': f'{pose.position.x:.9f}',
        f'{prefix}_y': f'{pose.position.y:.9f}',
        f'{prefix}_z': f'{pose.position.z:.9f}',
        f'{prefix}_qx': f'{pose.orientation.x:.9f}',
        f'{prefix}_qy': f'{pose.orientation.y:.9f}',
        f'{prefix}_qz': f'{pose.orientation.z:.9f}',
        f'{prefix}_qw': f'{pose.orientation.w:.9f}',
    }


def serialize_values(values):
    if values is None:
        return ''
    return ';'.join(f'{value:.9g}' for value in values)


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def pose_is_physical(pose, max_abs_position_m):
    values = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    if not all(math.isfinite(value) for value in values):
        return False
    if max(
            abs(pose.position.x),
            abs(pose.position.y),
            abs(pose.position.z)) > max_abs_position_m:
        return False
    q_norm = math.sqrt(
        pose.orientation.x * pose.orientation.x
        + pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z
        + pose.orientation.w * pose.orientation.w)
    return q_norm > 1e-6


def load_trajectory(path):
    with open(path, 'r', encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        missing = [field for field in TRAJECTORY_FIELDS if field not in reader.fieldnames]
        if missing:
            raise ValueError(f'Trajectory CSV is missing fields: {missing}')
        rows = []
        for row in reader:
            rows.append({field: float(row[field]) for field in TRAJECTORY_FIELDS})
    if len(rows) < 2:
        raise ValueError('Trajectory CSV must contain at least two samples')
    return rows


class BaseDisturbanceReplay(Node):
    def __init__(self, args):
        super().__init__(
            'base_disturbance_replay',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.set_client = self.create_client(SetEntityState, args.set_entity_state_service)
        self.get_client = self.create_client(GetEntityState, args.get_entity_state_service)
        self.latest_joint_positions = None
        self.initial_joint_positions = None
        self.previous_joint_positions = None
        self.max_joint_step = 0.0
        self.latest_velocity_command = None
        self.latest_visual_valid = None
        self.latest_visual_pose_stamp_ns = 0
        self.latest_visual_error = None
        self.latest_visual_dq_limited = None
        self.latest_model_pose = None
        self.latest_base_pose = None
        self.latest_ee_pose = None
        self.latest_camera_pose = None
        self.model_poses = {}
        self.link_poses = {}
        self.entity_stable_count = 0
        self.rows = []
        self.wall_stamps = []
        self.output_rows = []
        self.pre_roll_hold_stamps = []
        self.pre_roll_hold_failures = 0

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
        self.create_subscription(
            Bool,
            args.visual_valid_topic,
            self.visual_valid_callback,
            50)
        self.create_subscription(
            PoseStamped,
            args.visual_pose_topic,
            self.visual_pose_callback,
            50)
        self.create_subscription(
            Float64MultiArray,
            args.visual_error_topic,
            self.visual_error_callback,
            50)
        self.create_subscription(
            Float64MultiArray,
            args.visual_dq_topic,
            self.visual_dq_callback,
            50)
        self.create_subscription(
            ModelStates,
            args.model_states_topic,
            self.model_states_callback,
            10)
        self.create_subscription(
            LinkStates,
            args.link_states_topic,
            self.link_states_callback,
            10)

    def joint_state_callback(self, msg):
        if not msg.name or len(msg.position) < len(msg.name):
            return
        by_name = dict(zip(msg.name, msg.position))
        missing = [name for name in self.args.joint_names if name not in by_name]
        if missing:
            return
        positions = [by_name[name] for name in self.args.joint_names]
        if self.initial_joint_positions is None:
            self.initial_joint_positions = list(positions)
        if self.previous_joint_positions is not None:
            step = max(
                abs(current - previous)
                for current, previous in zip(positions, self.previous_joint_positions))
            self.max_joint_step = max(self.max_joint_step, step)
        self.previous_joint_positions = list(positions)
        self.latest_joint_positions = list(positions)

    def velocity_command_callback(self, msg):
        self.latest_velocity_command = list(msg.data)

    def visual_valid_callback(self, msg):
        self.latest_visual_valid = bool(msg.data)

    def visual_pose_callback(self, msg):
        self.latest_visual_pose_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)

    def visual_error_callback(self, msg):
        self.latest_visual_error = list(msg.data)

    def visual_dq_callback(self, msg):
        self.latest_visual_dq_limited = list(msg.data)

    def model_states_callback(self, msg):
        self.model_poses = {
            name: pose for name, pose in zip(msg.name, msg.pose)
            if pose_is_physical(pose, self.args.gt_max_abs_position_m)
        }
        pose = self.model_poses.get(self.args.entity_name)
        if pose is not None:
            self.latest_model_pose = pose

    def link_states_callback(self, msg):
        self.link_poses = {
            name: pose for name, pose in zip(msg.name, msg.pose)
            if pose_is_physical(pose, self.args.gt_max_abs_position_m)
        }
        self.refresh_latest_entity_poses()

    def refresh_latest_entity_poses(self):
        self.latest_model_pose = self.model_poses.get(
            self.args.entity_name, self.latest_model_pose)
        self.latest_base_pose = self.link_poses.get(
            self.args.base_entity_name, self.latest_base_pose)
        self.latest_ee_pose = self.link_poses.get(
            self.args.ee_entity_name, self.latest_ee_pose)
        self.latest_camera_pose = self.link_poses.get(
            self.args.camera_entity_name, self.latest_camera_pose)

    def cached_pose_for_entity(self, entity_name):
        if entity_name in self.model_poses:
            return self.model_poses[entity_name]
        if entity_name in self.link_poses:
            return self.link_poses[entity_name]
        return None

    def wait_for_services(self):
        if not self.set_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f'Gazebo service not available: {self.args.set_entity_state_service}')
        self.get_client.wait_for_service(timeout_sec=0.2)

    def wait_for_entities(self):
        required = [
            self.args.entity_name,
            self.args.base_entity_name,
            self.args.ee_entity_name,
            self.args.camera_entity_name,
        ]
        deadline = time.monotonic() + self.args.entity_ready_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.refresh_latest_entity_poses()
            missing = [
                entity_name for entity_name in required
                if self.cached_pose_for_entity(entity_name) is None
            ]
            if missing and self.get_client.service_is_ready():
                still_missing = []
                for entity_name in missing:
                    state = self.call_get_state(entity_name)
                    if (
                            state is None
                            or not pose_is_physical(
                                state.pose, self.args.gt_max_abs_position_m)):
                        still_missing.append(entity_name)
                        continue
                    if entity_name == self.args.entity_name:
                        self.latest_model_pose = state.pose
                    elif entity_name == self.args.base_entity_name:
                        self.latest_base_pose = state.pose
                    elif entity_name == self.args.ee_entity_name:
                        self.latest_ee_pose = state.pose
                    elif entity_name == self.args.camera_entity_name:
                        self.latest_camera_pose = state.pose
                missing = still_missing
            if not missing:
                self.entity_stable_count += 1
                if self.entity_stable_count >= self.args.entity_stable_samples:
                    return
            else:
                self.entity_stable_count = 0
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        raise RuntimeError(
            'Timed out waiting for Gazebo entities: ' + ', '.join(required))

    def call_set_state(self, row):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.args.entity_name
        request.state.reference_frame = self.args.world_frame
        request.state.pose = make_pose(row)
        request.state.twist = make_twist(row)
        future = self.set_client.call_async(request)
        return self.wait_for_future(future, self.args.service_call_timeout)

    def call_get_state(self, entity_name):
        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = self.args.world_frame
        future = self.get_client.call_async(request)
        response = self.wait_for_future(future, self.args.service_call_timeout)
        if response is None or not response.success:
            return None
        return response.state

    def wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.001)
        if not future.done():
            return None
        return future.result()

    def hold_initial_state_until(self, first_row, start_time):
        if not self.args.hold_initial_state_during_start_delay:
            while rclpy.ok() and time.monotonic() < start_time:
                rclpy.spin_once(self, timeout_sec=0.01)
            return

        self.get_logger().info(
            'Holding the initial commanded base state during the replay start delay')
        period = 1.0 / self.args.rate_hz
        index = 0
        while rclpy.ok():
            target_time = start_time - self.args.start_delay_sec + index * period
            now = time.monotonic()
            if now >= start_time:
                break
            if now < target_time:
                rclpy.spin_once(
                    self, timeout_sec=min(0.002, target_time - now))
                continue

            response = self.call_set_state(first_row)
            self.pre_roll_hold_stamps.append(time.monotonic())
            if response is None or not response.success:
                self.pre_roll_hold_failures += 1
            index += 1

    def pre_roll_hold_frequency_hz(self):
        if len(self.pre_roll_hold_stamps) < 2:
            return 0.0
        elapsed = self.pre_roll_hold_stamps[-1] - self.pre_roll_hold_stamps[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self.pre_roll_hold_stamps) - 1) / elapsed

    def run(self, rows):
        self.rows = rows
        self.wait_for_services()
        self.wait_for_entities()
        self.get_logger().info(
            f'Replaying {len(rows)} samples to {self.args.entity_name} at '
            f'{self.args.rate_hz:.1f} Hz target')
        self.get_logger().info(
            'Sampling Gazebo state from /model_states and /link_states caches; '
            'get_entity_state is only used during startup readiness checks')

        start = time.monotonic() + self.args.start_delay_sec
        self.hold_initial_state_until(rows[0], start)

        period = 1.0 / self.args.rate_hz
        for index, row in enumerate(rows):
            if not rclpy.ok():
                break
            target_time = start + index * period
            while rclpy.ok():
                now = time.monotonic()
                if now >= target_time:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.002, target_time - now))

            before_call = time.monotonic()
            set_response = self.call_set_state(row)
            after_call = time.monotonic()
            self.wall_stamps.append(after_call)

            command_pose = make_pose(row)
            self.refresh_latest_entity_poses()

            model_pose = self.latest_model_pose
            base_pose = self.latest_base_pose
            ee_pose = self.latest_ee_pose
            camera_pose = self.latest_camera_pose
            tracking_pose = base_pose if base_pose is not None else model_pose
            latest_visual_pose_age_sec = ''
            now_ns = self.get_clock().now().nanoseconds
            if self.latest_visual_pose_stamp_ns > 0 and now_ns > self.latest_visual_pose_stamp_ns:
                latest_visual_pose_age_sec = (
                    f'{(now_ns - self.latest_visual_pose_stamp_ns) * 1e-9:.9f}')
            row_out = {
                'trajectory_time_sec': f'{float(row["time_sec"]):.9f}',
                'wall_time_sec': f'{after_call - start:.9f}',
                'pre_roll_hold_enabled': str(
                    bool(self.args.hold_initial_state_during_start_delay)).lower(),
                'pre_roll_hold_attempts': len(self.pre_roll_hold_stamps),
                'pre_roll_hold_failures': self.pre_roll_hold_failures,
                'pre_roll_hold_rate_hz': f'{self.pre_roll_hold_frequency_hz():.9f}',
                'set_success': bool(set_response.success) if set_response else False,
                'set_call_sec': f'{after_call - before_call:.9f}',
                'pose_tracking_error_m': (
                    f'{position_error(command_pose, tracking_pose):.9f}'
                    if tracking_pose is not None else ''),
                'orientation_tracking_error_rad': (
                    f'{quaternion_angle_error(command_pose, tracking_pose):.9f}'
                    if tracking_pose is not None else ''),
                'joint_max_step_rad': f'{self.max_joint_step:.9f}',
                'joint_positions': serialize_values(self.latest_joint_positions),
                'velocity_command': serialize_values(self.latest_velocity_command),
                'visual_valid': (
                    '' if self.latest_visual_valid is None
                    else str(bool(self.latest_visual_valid)).lower()),
                'latest_visual_pose_age_sec': latest_visual_pose_age_sec,
                'visual_error': serialize_values(self.latest_visual_error),
                'visual_dq_limited': serialize_values(self.latest_visual_dq_limited),
            }
            row_out.update(serialize_pose('command', command_pose))
            row_out.update(serialize_pose('actual_model', model_pose))
            row_out.update(serialize_pose('actual_base', base_pose))
            row_out.update(serialize_pose('actual_link6', ee_pose))
            row_out.update(serialize_pose('actual_camera', camera_pose))
            self.output_rows.append(row_out)

        self.write_csv()
        self.print_summary()

    def actual_frequency_hz(self):
        if len(self.wall_stamps) < 2:
            return 0.0
        elapsed = self.wall_stamps[-1] - self.wall_stamps[0]
        return (len(self.wall_stamps) - 1) / elapsed if elapsed > 0.0 else 0.0

    def joint_discontinuity_from_initial(self):
        if self.initial_joint_positions is None or self.latest_joint_positions is None:
            return 0.0
        return max(
            abs(current - initial)
            for current, initial in zip(self.latest_joint_positions, self.initial_joint_positions))

    def print_summary(self):
        errors = [
            float(row['pose_tracking_error_m'])
            for row in self.output_rows
            if row['pose_tracking_error_m'] != ''
        ]
        set_failures = sum(1 for row in self.output_rows if not row['set_success'])
        print('Base disturbance replay summary')
        print(f'  samples: {len(self.output_rows)}')
        print(f'  target_rate_hz: {self.args.rate_hz:.3f}')
        print(f'  actual_rate_hz: {self.actual_frequency_hz():.3f}')
        print(
            '  pre_roll_hold_enabled: '
            f'{str(bool(self.args.hold_initial_state_during_start_delay)).lower()}')
        print(f'  pre_roll_hold_attempts: {len(self.pre_roll_hold_stamps)}')
        print(f'  pre_roll_hold_failures: {self.pre_roll_hold_failures}')
        print(f'  pre_roll_hold_rate_hz: {self.pre_roll_hold_frequency_hz():.3f}')
        print(f'  set_failures: {set_failures}')
        print(f'  joint_max_step_rad: {self.max_joint_step:.9f}')
        print(f'  joint_final_discontinuity_rad: {self.joint_discontinuity_from_initial():.9f}')
        if errors:
            print(f'  pose_tracking_error_mean_m: {statistics.mean(errors):.9f}')
            print(f'  pose_tracking_error_max_m: {max(errors):.9f}')
        print(f'  csv: {self.args.output_csv}')

    def write_csv(self):
        if not self.args.output_csv:
            return
        directory = os.path.dirname(os.path.abspath(self.args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = [
            'trajectory_time_sec',
            'wall_time_sec',
            'pre_roll_hold_enabled',
            'pre_roll_hold_attempts',
            'pre_roll_hold_failures',
            'pre_roll_hold_rate_hz',
            'set_success',
            'set_call_sec',
            'pose_tracking_error_m',
            'orientation_tracking_error_rad',
            'joint_max_step_rad',
            'joint_positions',
            'velocity_command',
            'visual_valid',
            'latest_visual_pose_age_sec',
            'visual_error',
            'visual_dq_limited',
        ]
        for prefix in ('command', 'actual_model', 'actual_base', 'actual_link6', 'actual_camera'):
            fieldnames.extend([
                f'{prefix}_x',
                f'{prefix}_y',
                f'{prefix}_z',
                f'{prefix}_qx',
                f'{prefix}_qy',
                f'{prefix}_qz',
                f'{prefix}_qw',
            ])
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.output_rows)


def parse_joint_names(value):
    return [part for part in value.replace(',', ' ').split() if part]


def parse_bool(value):
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError(f'invalid boolean value: {value}')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Replay a base disturbance trajectory through Gazebo set_entity_state.')
    parser.add_argument('--trajectory-csv', required=True)
    parser.add_argument('--output-csv', default='/tmp/base_disturbance_replay.csv')
    parser.add_argument('--entity-name', default='windylab_arm')
    parser.add_argument('--base-entity-name', default='windylab_arm::base_link')
    parser.add_argument('--ee-entity-name', default='windylab_arm::link6')
    parser.add_argument(
        '--camera-entity-name',
        default='windylab_arm::link6')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--set-entity-state-service', default='/set_entity_state')
    parser.add_argument('--get-entity-state-service', default='/get_entity_state')
    parser.add_argument('--model-states-topic', default='/model_states')
    parser.add_argument('--link-states-topic', default='/link_states')
    parser.add_argument('--joint-state-topic', default='/joint_states')
    parser.add_argument('--velocity-command-topic', default='/arm_velocity_controller/commands')
    parser.add_argument('--visual-valid-topic', default='/visual_ee_pose_valid')
    parser.add_argument('--visual-pose-topic', default='/visual_ee_pose')
    parser.add_argument('--visual-error-topic', default='/visual_stabilization/error')
    parser.add_argument('--visual-dq-topic', default='/visual_stabilization/dq_limited')
    parser.add_argument('--joint-names', type=parse_joint_names, default=parse_joint_names(
        'joint1 joint2 joint3 joint4 joint5 joint6'))
    parser.add_argument('--rate-hz', type=float, default=100.0)
    parser.add_argument('--state-sample-stride', type=int, default=1)
    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--service-call-timeout', type=float, default=0.05)
    parser.add_argument('--entity-ready-timeout', type=float, default=20.0)
    parser.add_argument('--entity-stable-samples', type=int, default=3)
    parser.add_argument('--gt-max-abs-position-m', type=float, default=5.0)
    parser.add_argument('--start-delay-sec', type=float, default=2.0)
    parser.add_argument(
        '--hold-initial-state-during-start-delay',
        type=parse_bool,
        default=True,
        help='Continuously prescribe trajectory row 0 while waiting to start replay.')
    parser.add_argument('--use-sim-time', action='store_true')
    args, _ = parser.parse_known_args(argv)
    if args.rate_hz <= 0.0:
        raise SystemExit('--rate-hz must be positive')
    if args.state_sample_stride <= 0:
        raise SystemExit('--state-sample-stride must be positive')
    if args.entity_stable_samples <= 0:
        raise SystemExit('--entity-stable-samples must be positive')
    if args.gt_max_abs_position_m <= 0.0:
        raise SystemExit('--gt-max-abs-position-m must be positive')
    if args.start_delay_sec < 0.0:
        raise SystemExit('--start-delay-sec must be non-negative')
    if not args.joint_names:
        raise SystemExit('--joint-names must not be empty')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rows = load_trajectory(args.trajectory_csv)
    rclpy.init()
    node = BaseDisturbanceReplay(args)
    try:
        node.run(rows)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
