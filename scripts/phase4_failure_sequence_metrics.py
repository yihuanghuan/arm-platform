#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    '/model_states',
    '/joint_states',
    '/arm_velocity_controller/commands',
    '/visual_stabilization/status',
)


def parse_bag(value):
    if '=' not in value:
        raise argparse.ArgumentTypeError('bag must be LABEL=PATH')
    label, path = value.split('=', 1)
    if not label or not path:
        raise argparse.ArgumentTypeError('bag must be LABEL=PATH')
    return label, path


def elapsed_sec(stamp, start_stamp):
    return (stamp - start_stamp) * 1e-9


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def analyze_bag(label, path, args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing_topics = [topic for topic in TOPICS if topic not in topic_types]

    first_stamp = None
    first_model_position = None
    first_joint_positions = None
    target_lock_sec = None
    first_nonzero_command_sec = None
    base_over_1cm_sec = None
    base_over_1m_sec = None
    safety_stop_sec = None
    max_base_displacement = 0.0
    max_joint_deviation = 0.0
    max_command = 0.0
    max_visual_error = 0.0
    command_count_after_safety = 0
    nonzero_command_count_after_safety = 0
    status_json_errors = 0

    while reader.has_next():
        topic, serialized, stamp = reader.read_next()
        if topic not in TOPICS:
            continue
        if first_stamp is None:
            first_stamp = stamp
        time_sec = elapsed_sec(stamp, first_stamp)
        message = deserialize_message(serialized, get_message(topic_types[topic]))

        if topic == '/model_states':
            if args.model_name not in message.name:
                continue
            pose = message.pose[message.name.index(args.model_name)]
            position = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            if first_model_position is None:
                first_model_position = position
            displacement = math.dist(position, first_model_position)
            max_base_displacement = max(max_base_displacement, displacement)
            if displacement > args.base_small_threshold_m and base_over_1cm_sec is None:
                base_over_1cm_sec = time_sec
            if displacement > args.base_large_threshold_m and base_over_1m_sec is None:
                base_over_1m_sec = time_sec

        elif topic == '/joint_states':
            position_by_name = dict(zip(message.name, message.position))
            if not all(name in position_by_name for name in args.joint_names):
                continue
            positions = tuple(
                float(position_by_name[name]) for name in args.joint_names)
            if first_joint_positions is None:
                first_joint_positions = positions
            max_joint_deviation = max(
                max(
                    abs(current - initial)
                    for current, initial in zip(positions, first_joint_positions)),
                max_joint_deviation)

        elif topic == '/arm_velocity_controller/commands':
            peak = max((abs(float(value)) for value in message.data), default=0.0)
            max_command = max(max_command, peak)
            if peak > args.command_threshold_rad_s and first_nonzero_command_sec is None:
                first_nonzero_command_sec = time_sec
            if safety_stop_sec is not None:
                command_count_after_safety += 1
                if peak > args.command_threshold_rad_s:
                    nonzero_command_count_after_safety += 1

        elif topic == '/visual_stabilization/status':
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                status_json_errors += 1
                continue
            if payload.get('target_locked') and target_lock_sec is None:
                target_lock_sec = time_sec
            position_error = payload.get('position_error')
            if position_error:
                max_visual_error = max(
                    max_visual_error,
                    vector_norm(float(value) for value in position_error))
            if payload.get('safety_stop') and safety_stop_sec is None:
                safety_stop_sec = time_sec

    invalid_reasons = []
    if missing_topics:
        invalid_reasons.append('missing_topics:' + ','.join(missing_topics))
    if first_stamp is None:
        invalid_reasons.append('no_required_topic_messages')
    if target_lock_sec is None:
        invalid_reasons.append('target_never_locked')
    if first_nonzero_command_sec is None:
        invalid_reasons.append('no_nonzero_command')
    if base_over_1cm_sec is None:
        invalid_reasons.append('base_never_crossed_small_threshold')
    if base_over_1m_sec is None:
        invalid_reasons.append('base_never_crossed_large_threshold')
    if (
            first_nonzero_command_sec is not None
            and base_over_1cm_sec is not None
            and base_over_1cm_sec <= first_nonzero_command_sec):
        invalid_reasons.append('base_small_motion_not_after_command')
    if (
            first_nonzero_command_sec is not None
            and base_over_1m_sec is not None
            and base_over_1m_sec <= first_nonzero_command_sec):
        invalid_reasons.append('base_large_motion_not_after_command')
    if max_joint_deviation <= args.joint_failure_threshold_rad:
        invalid_reasons.append('joint_deviation_below_failure_threshold')

    return {
        'label': label,
        'bag': path,
        'sequence_valid': not invalid_reasons,
        'invalid_reason': ';'.join(invalid_reasons),
        'target_lock_sec': target_lock_sec,
        'first_nonzero_command_sec': first_nonzero_command_sec,
        'base_over_1cm_sec': base_over_1cm_sec,
        'base_over_1m_sec': base_over_1m_sec,
        'safety_stop_sec': safety_stop_sec,
        'max_base_displacement_m': max_base_displacement,
        'max_joint_deviation_rad': max_joint_deviation,
        'max_command_rad_s': max_command,
        'max_visual_error_m': max_visual_error,
        'nonzero_command_ratio_after_safety': (
            nonzero_command_count_after_safety / command_count_after_safety
            if command_count_after_safety else None),
        'status_json_errors': status_json_errors,
    }


def write_summary(path, summaries):
    fieldnames = [
        'label',
        'bag',
        'sequence_valid',
        'invalid_reason',
        'target_lock_sec',
        'first_nonzero_command_sec',
        'base_over_1cm_sec',
        'base_over_1m_sec',
        'safety_stop_sec',
        'max_base_displacement_m',
        'max_joint_deviation_rad',
        'max_command_rad_s',
        'max_visual_error_m',
        'nonzero_command_ratio_after_safety',
        'status_json_errors',
    ]
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)


def print_summary(summaries, output_csv):
    print('Phase 4 static visual failure-sequence metrics')
    for summary in summaries:
        print(f'  {summary["label"]}:')
        for key, value in summary.items():
            if key in ('label', 'bag'):
                continue
            if isinstance(value, float):
                value = f'{value:.9g}'
            elif value is None:
                value = 'n/a'
            print(f'    {key}: {value}')
    print(f'  summary_csv: {output_csv}')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Verify the command-before-free-base-motion failure sequence in phase 4 '
            'static visual rosbag captures.'))
    parser.add_argument('--bag', action='append', type=parse_bag, required=True)
    parser.add_argument(
        '--output-csv',
        default='/tmp/windylab_phase4_failure_sequence_summary.csv')
    parser.add_argument('--model-name', default='windylab_arm')
    parser.add_argument(
        '--joint-names', nargs='+',
        default=[f'joint{index}' for index in range(1, 7)])
    parser.add_argument('--command-threshold-rad-s', type=float, default=1e-4)
    parser.add_argument('--base-small-threshold-m', type=float, default=0.01)
    parser.add_argument('--base-large-threshold-m', type=float, default=1.0)
    parser.add_argument('--joint-failure-threshold-rad', type=float, default=0.1)
    parser.add_argument(
        '--require-sequence', action='store_true',
        help='Exit nonzero if any bag does not contain the expected failure sequence.')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.command_threshold_rad_s < 0.0:
        raise SystemExit('--command-threshold-rad-s must be non-negative')
    if args.base_small_threshold_m <= 0.0:
        raise SystemExit('--base-small-threshold-m must be positive')
    if args.base_large_threshold_m <= args.base_small_threshold_m:
        raise SystemExit('--base-large-threshold-m must exceed the small threshold')
    if args.joint_failure_threshold_rad <= 0.0:
        raise SystemExit('--joint-failure-threshold-rad must be positive')

    summaries = [analyze_bag(label, path, args) for label, path in args.bag]
    write_summary(args.output_csv, summaries)
    print_summary(summaries, args.output_csv)
    if args.require_sequence and not all(
            summary['sequence_valid'] for summary in summaries):
        raise SystemExit('one or more bags failed the expected failure-sequence gate')


if __name__ == '__main__':
    main()
