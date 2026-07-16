#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
import sys


def parse_run(value):
    if '=' not in value:
        raise argparse.ArgumentTypeError('run must be LABEL=CSV')
    label, path = value.split('=', 1)
    if not label or not path:
        raise argparse.ArgumentTypeError('run must be LABEL=CSV')
    return label, path


def parse_float(value):
    if value in (None, ''):
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def parse_bool(value):
    if value == '':
        return None
    return str(value).strip().lower() in ('1', 'true', 'yes')


def parse_vector(value):
    if value in (None, ''):
        return []
    result = []
    for part in value.split(';'):
        item = parse_float(part)
        if item is None:
            return []
        result.append(item)
    return result


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def rms(values):
    if not values:
        return None
    return math.sqrt(statistics.mean(value * value for value in values))


def load_rows(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def first_position(rows):
    for row in rows:
        position = actual_link6_position(row)
        if position is not None:
            return position
    return None


def actual_link6_position(row):
    values = [
        parse_float(row.get('actual_link6_x')),
        parse_float(row.get('actual_link6_y')),
        parse_float(row.get('actual_link6_z')),
    ]
    if any(value is None for value in values):
        return None
    return values


def position_is_physical(position, max_abs_position_m):
    return (
        position is not None
        and all(math.isfinite(value) for value in position)
        and max(abs(value) for value in position) <= max_abs_position_m)


def physical_actual_link6_position(row, max_abs_position_m):
    position = actual_link6_position(row)
    if not position_is_physical(position, max_abs_position_m):
        return None
    return position


def summarize(label, path, args):
    rows = load_rows(path)
    origin = None
    for row in rows:
        origin = physical_actual_link6_position(row, args.gt_max_abs_position_m)
        if origin is not None:
            break
    errors = []
    wall_times = []
    velocity_peaks = []
    visual_valid_values = []
    visual_pose_ages = []
    visual_error_norms = []
    saturated_count = 0
    command_count = 0
    set_failures = 0
    invalid_gt_samples = 0
    max_joint_step = 0.0

    for row in rows:
        if str(row.get('set_success', '')).lower() != 'true':
            set_failures += 1

        joint_step = parse_float(row.get('joint_max_step_rad'))
        if joint_step is not None:
            max_joint_step = max(max_joint_step, joint_step)

        raw_position = actual_link6_position(row)
        position = physical_actual_link6_position(row, args.gt_max_abs_position_m)
        if raw_position is not None and position is None:
            invalid_gt_samples += 1
        if origin is not None and position is not None:
            errors.append(norm([current - start for current, start in zip(position, origin)]))
            wall_time = parse_float(row.get('wall_time_sec'))
            wall_times.append(wall_time if wall_time is not None else 0.0)

        command = parse_vector(row.get('velocity_command'))
        if command:
            peak = max(abs(value) for value in command)
            velocity_peaks.append(peak)
            command_count += 1
            if peak >= args.max_joint_velocity - args.saturation_epsilon:
                saturated_count += 1

        valid = parse_bool(row.get('visual_valid', ''))
        if valid is not None:
            visual_valid_values.append(valid)

        age = parse_float(row.get('latest_visual_pose_age_sec'))
        if age is not None:
            visual_pose_ages.append(age)

        visual_error = parse_vector(row.get('visual_error'))
        if len(visual_error) >= 3:
            visual_error_norms.append(norm(visual_error[:3]))

    steady_errors = []
    if errors and wall_times:
        final_time = wall_times[-1]
        steady_errors = [
            error for error, wall_time in zip(errors, wall_times)
            if wall_time >= final_time - args.steady_window_sec
        ]

    visual_valid_count = sum(1 for value in visual_valid_values if value)
    result = {
        'label': label,
        'csv': path,
        'samples': len(rows),
        'valid_gt_samples': len(errors),
        'invalid_gt_samples': invalid_gt_samples,
        'set_failures': set_failures,
        'xyz_rms_m': rms(errors),
        'xyz_max_m': max(errors) if errors else None,
        'steady_xyz_mean_m': statistics.mean(steady_errors) if steady_errors else None,
        'steady_xyz_max_m': max(steady_errors) if steady_errors else None,
        'joint_velocity_peak_rad_s': max(velocity_peaks) if velocity_peaks else None,
        'joint_max_step_rad': max_joint_step,
        'command_saturation_ratio': (
            saturated_count / command_count if command_count else None),
        'visual_loss_rate': (
            1.0 - visual_valid_count / len(visual_valid_values)
            if visual_valid_values else None),
        'visual_pose_age_mean_sec': (
            statistics.mean(visual_pose_ages) if visual_pose_ages else None),
        'visual_pose_age_max_sec': max(visual_pose_ages) if visual_pose_ages else None,
        'visual_error_rms_m': rms(visual_error_norms),
        'visual_error_max_m': max(visual_error_norms) if visual_error_norms else None,
    }
    return result


def format_value(value):
    if value is None:
        return 'n/a'
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f'{value:.6g}'
    return str(value)


def write_summary(path, summaries):
    if not path:
        return
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    fieldnames = [
        'label',
        'csv',
        'samples',
        'valid_gt_samples',
        'invalid_gt_samples',
        'set_failures',
        'xyz_rms_m',
        'xyz_max_m',
        'steady_xyz_mean_m',
        'steady_xyz_max_m',
        'joint_velocity_peak_rad_s',
        'joint_max_step_rad',
        'command_saturation_ratio',
        'visual_loss_rate',
        'visual_pose_age_mean_sec',
        'visual_pose_age_max_sec',
        'visual_error_rms_m',
        'visual_error_max_m',
    ]
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            writer.writerow({
                key: '' if summary.get(key) is None else summary.get(key)
                for key in fieldnames
            })


def print_summary(summaries, output_csv):
    print('Phase 4 dynamic XYZ metrics')
    for summary in summaries:
        print(f'  {summary["label"]}:')
        for key in (
                'samples',
                'valid_gt_samples',
                'invalid_gt_samples',
                'set_failures',
                'xyz_rms_m',
                'xyz_max_m',
                'steady_xyz_mean_m',
                'steady_xyz_max_m',
                'joint_velocity_peak_rad_s',
                'joint_max_step_rad',
                'command_saturation_ratio',
                'visual_loss_rate',
                'visual_pose_age_mean_sec',
                'visual_pose_age_max_sec',
                'visual_error_rms_m',
                'visual_error_max_m'):
            print(f'    {key}: {format_value(summary.get(key))}')
    if output_csv:
        print(f'  summary_csv: {output_csv}')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Summarize dynamic base-disturbance XYZ stabilization replay CSVs.')
    parser.add_argument(
        '--run',
        action='append',
        type=parse_run,
        required=True,
        help='Run label and replay CSV path, formatted as LABEL=CSV.')
    parser.add_argument('--output-csv', default='/tmp/windylab_phase4_summary.csv')
    parser.add_argument('--steady-window-sec', type=float, default=5.0)
    parser.add_argument('--max-joint-velocity', type=float, default=0.35)
    parser.add_argument('--saturation-epsilon', type=float, default=1e-4)
    parser.add_argument('--gt-max-abs-position-m', type=float, default=5.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.steady_window_sec <= 0.0:
        raise SystemExit('--steady-window-sec must be positive')
    if args.max_joint_velocity <= 0.0:
        raise SystemExit('--max-joint-velocity must be positive')
    if args.gt_max_abs_position_m <= 0.0:
        raise SystemExit('--gt-max-abs-position-m must be positive')

    summaries = [summarize(label, path, args) for label, path in args.run]
    write_summary(args.output_csv, summaries)
    print_summary(summaries, args.output_csv)


if __name__ == '__main__':
    main()
