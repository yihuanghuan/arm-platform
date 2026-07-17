#!/usr/bin/env python3

import argparse
import bisect
import csv
import json
import math
import os
import sys


RATE_FIELDS = (
    'image_rate_hz',
    'camera_info_rate_hz',
    'target_detection_rate_hz',
    'tag_tf_update_rate_hz',
    'visual_pose_rate_hz',
)


def parse_float(value):
    if value in (None, ''):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def parse_bool(value):
    if value in (None, ''):
        return None
    return str(value).strip().lower() in ('1', 'true', 'yes')


def load_rows(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def percentile(values, fraction):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    index = fraction * (len(values) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return values[lower]
    weight = index - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def parse_semicolon_vector(value):
    if not value:
        return []
    result = []
    for part in str(value).split(';'):
        parsed = parse_float(part)
        if parsed is None:
            return []
        result.append(parsed)
    return result


def vector_from_row(row, prefix):
    values = [
        parse_float(row.get(f'{prefix}_x')),
        parse_float(row.get(f'{prefix}_y')),
        parse_float(row.get(f'{prefix}_z')),
    ]
    return None if any(value is None for value in values) else values


def interpolate(times, vectors, stamp):
    index = bisect.bisect_left(times, stamp)
    if index <= 0:
        return vectors[0] if times and stamp == times[0] else None
    if index >= len(times):
        return vectors[-1] if times and stamp == times[-1] else None
    left_time = times[index - 1]
    right_time = times[index]
    if right_time <= left_time:
        return None
    ratio = (stamp - left_time) / (right_time - left_time)
    return [
        vectors[index - 1][axis] * (1.0 - ratio) + vectors[index][axis] * ratio
        for axis in range(3)
    ]


def correlation_and_slope(left, right):
    if len(left) < 2 or len(left) != len(right):
        return None, None
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    covariance = sum(
        (x - left_mean) * (y - right_mean)
        for x, y in zip(left, right))
    left_variance = sum((value - left_mean) ** 2 for value in left)
    right_variance = sum((value - right_mean) ** 2 for value in right)
    if left_variance <= 0.0 or right_variance <= 0.0:
        return None, None
    correlation = covariance / math.sqrt(left_variance * right_variance)
    return correlation, covariance / left_variance


def compute_metrics(args):
    visual_rows = load_rows(args.visual_chain_csv)
    transform_rows = load_rows(args.transform_csv)
    replay_rows = load_rows(args.replay_csv)

    steady_visual_rows = [
        row for row in visual_rows
        if (parse_float(row.get('wall_time_sec')) or 0.0) >= args.steady_start_wall_sec
    ]
    metrics = {
        'steady_visual_rows': len(steady_visual_rows),
    }
    for field in RATE_FIELDS:
        values = [parse_float(row.get(field)) for row in steady_visual_rows]
        values = [value for value in values if value is not None]
        metrics[f'{field}_p05'] = percentile(values, 0.05)
        metrics[f'{field}_mean'] = sum(values) / len(values) if values else None

    valid_values = [parse_bool(row.get('visual_valid')) for row in steady_visual_rows]
    valid_values = [value for value in valid_values if value is not None]
    metrics['visual_valid_fraction'] = (
        sum(1 for value in valid_values if value) / len(valid_values)
        if valid_values else None)
    ages = [parse_float(row.get('visual_pose_age_sec')) for row in steady_visual_rows]
    ages = [value for value in ages if value is not None]
    metrics['visual_pose_age_max_sec'] = max(ages) if ages else None
    metrics['visual_pose_age_p95_sec'] = percentile(ages, 0.95)

    replay_times = []
    replay_positions = []
    command_peak = 0.0
    base_tracking_max = 0.0
    for row in replay_rows:
        stamp = parse_float(row.get('sim_time_sec'))
        position = vector_from_row(row, 'actual_link6')
        if stamp is not None and position is not None:
            replay_times.append(stamp)
            replay_positions.append(position)
        command = parse_semicolon_vector(row.get('velocity_command'))
        command_peak = max(
            command_peak,
            max((abs(value) for value in command), default=0.0))
        tracking_error = parse_float(row.get('pose_tracking_error_m'))
        if tracking_error is not None:
            base_tracking_max = max(base_tracking_max, tracking_error)
    metrics['joint_velocity_command_peak_rad_s'] = command_peak
    metrics['base_tracking_error_max_m'] = base_tracking_max

    errors = []
    gt_x = []
    visual_x = []
    previous_visual_stamp = None
    for row in transform_rows:
        stamp = parse_float(row.get('visual_pose_stamp_sec'))
        visual_position = vector_from_row(row, 'visual_ee')
        if (
                stamp is None
                or visual_position is None
                or stamp == previous_visual_stamp):
            continue
        previous_visual_stamp = stamp
        gt_position = interpolate(replay_times, replay_positions, stamp)
        if gt_position is None:
            continue
        delta = [
            visual_position[axis] - gt_position[axis]
            for axis in range(3)
        ]
        errors.append(math.sqrt(sum(value * value for value in delta)))
        gt_x.append(gt_position[0])
        visual_x.append(visual_position[0])

    correlation, slope = correlation_and_slope(gt_x, visual_x)
    metrics['aligned_visual_samples'] = len(errors)
    metrics['aligned_position_rms_m'] = (
        math.sqrt(sum(value * value for value in errors) / len(errors))
        if errors else None)
    metrics['aligned_position_max_m'] = max(errors) if errors else None
    metrics['aligned_x_correlation'] = correlation
    metrics['aligned_x_slope'] = slope
    return metrics


def evaluate(metrics, args):
    checks = {
        'enough_steady_rows': metrics['steady_visual_rows'] >= 100,
        'image_rate': metrics['image_rate_hz_p05'] >= args.min_image_rate_hz,
        'camera_info_rate': (
            metrics['camera_info_rate_hz_p05'] >= args.min_image_rate_hz),
        'target_detection_rate': (
            metrics['target_detection_rate_hz_p05'] >= args.min_visual_rate_hz),
        'tag_tf_update_rate': (
            metrics['tag_tf_update_rate_hz_p05'] >= args.min_visual_rate_hz),
        'visual_pose_rate': (
            metrics['visual_pose_rate_hz_p05'] >= args.min_visual_rate_hz),
        'visual_valid_fraction': (
            metrics['visual_valid_fraction'] >= args.min_visual_valid_fraction),
        'visual_pose_age': (
            metrics['visual_pose_age_max_sec'] <= args.max_visual_pose_age_sec),
        'open_loop_zero_command': (
            metrics['joint_velocity_command_peak_rad_s']
            <= args.max_joint_velocity_command_rad_s),
        'base_tracking': (
            metrics['base_tracking_error_max_m'] <= args.max_base_tracking_error_m),
        'aligned_position_rms': (
            metrics['aligned_position_rms_m'] <= args.max_position_rms_m),
        'aligned_position_max': (
            metrics['aligned_position_max_m'] <= args.max_position_max_m),
        'x_correlation': (
            metrics['aligned_x_correlation'] >= args.min_x_correlation),
        'x_slope': (
            args.min_x_slope
            <= metrics['aligned_x_slope']
            <= args.max_x_slope),
    }
    return checks


def write_outputs(metrics, checks, args):
    payload = {
        'metrics': metrics,
        'checks': checks,
        'overall_passed': all(checks.values()),
    }
    if args.output_json:
        directory = os.path.dirname(os.path.abspath(args.output_json))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(args.output_json, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
    if args.output_csv:
        directory = os.path.dirname(os.path.abspath(args.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        row = dict(metrics)
        row.update({f'check_{key}': value for key, value in checks.items()})
        row['overall_passed'] = payload['overall_passed']
        with open(args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
            writer.writeheader()
            writer.writerow(row)
    return payload


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Evaluate phase 4 visual open-loop frequency, age, and geometry.')
    parser.add_argument('--visual-chain-csv', required=True)
    parser.add_argument('--transform-csv', required=True)
    parser.add_argument('--replay-csv', required=True)
    parser.add_argument('--output-json', default='')
    parser.add_argument('--output-csv', default='')
    parser.add_argument('--steady-start-wall-sec', type=float, default=12.0)
    parser.add_argument('--min-image-rate-hz', type=float, default=12.0)
    parser.add_argument('--min-visual-rate-hz', type=float, default=10.0)
    parser.add_argument('--min-visual-valid-fraction', type=float, default=0.95)
    parser.add_argument('--max-visual-pose-age-sec', type=float, default=0.20)
    parser.add_argument('--max-joint-velocity-command-rad-s', type=float, default=1e-9)
    parser.add_argument('--max-base-tracking-error-m', type=float, default=0.0021)
    parser.add_argument('--max-position-rms-m', type=float, default=0.010)
    parser.add_argument('--max-position-max-m', type=float, default=0.020)
    parser.add_argument('--min-x-correlation', type=float, default=0.8)
    parser.add_argument('--min-x-slope', type=float, default=0.8)
    parser.add_argument('--max-x-slope', type=float, default=1.2)
    parser.add_argument('--require-pass', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    metrics = compute_metrics(args)
    checks = evaluate(metrics, args)
    payload = write_outputs(metrics, checks, args)
    print(json.dumps(payload, indent=2, sort_keys=True))
    if args.require_pass and not payload['overall_passed']:
        raise SystemExit('phase 4 visual open-loop gate failed')


if __name__ == '__main__':
    main()
