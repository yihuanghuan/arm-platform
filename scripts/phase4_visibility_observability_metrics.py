#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys


RATE_FIELDS = (
    'image_rate_hz',
    'camera_info_rate_hz',
    'apriltag_msg_rate_hz',
    'target_detection_rate_hz',
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
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes'):
        return True
    if normalized in ('0', 'false', 'no'):
        return False
    return None


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


def finite_values(rows, field):
    return [
        value for value in (parse_float(row.get(field)) for row in rows)
        if value is not None
    ]


def min_or_none(values):
    return min(values) if values else None


def max_or_none(values):
    return max(values) if values else None


def at_least(value, threshold):
    return value is not None and value >= threshold


def at_most(value, threshold):
    return value is not None and value <= threshold


def in_range(minimum, maximum, lower, upper):
    return (
        minimum is not None
        and maximum is not None
        and minimum >= lower
        and maximum <= upper)


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


def quaternion_from_row(row, prefix):
    values = [
        parse_float(row.get(f'{prefix}_qx')),
        parse_float(row.get(f'{prefix}_qy')),
        parse_float(row.get(f'{prefix}_qz')),
        parse_float(row.get(f'{prefix}_qw')),
    ]
    if any(value is None for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        return None
    return [value / norm for value in values]


def quaternion_angle(a_quaternion, b_quaternion):
    dot = abs(sum(
        left * right for left, right in zip(a_quaternion, b_quaternion)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def compute_metrics(args):
    visual_rows = load_rows(args.visual_chain_csv)
    replay_rows = load_rows(args.replay_csv)
    steady_rows = [
        row for row in visual_rows
        if (parse_float(row.get('wall_time_sec')) or 0.0)
        >= args.steady_start_wall_sec
    ]

    metrics = {
        'label': args.label,
        'steady_visual_rows': len(steady_rows),
    }
    for field in RATE_FIELDS:
        values = finite_values(steady_rows, field)
        metrics[f'{field}_p05'] = percentile(values, 0.05)
        metrics[f'{field}_mean'] = (
            sum(values) / len(values) if values else None)

    target_states = [parse_bool(row.get('target_detected')) for row in steady_rows]
    target_states = [value for value in target_states if value is not None]
    metrics['target_frame_coverage'] = (
        sum(1 for value in target_states if value) / len(target_states)
        if target_states else None)

    family_counts = finite_values(steady_rows, 'family_detection_count')
    metrics['family_detection_count_min'] = min_or_none(family_counts)
    decision_margins = finite_values(steady_rows, 'target_decision_margin')
    metrics['target_decision_margin_p05'] = percentile(decision_margins, 0.05)
    corner_margins = finite_values(steady_rows, 'target_corner_margin_px')
    metrics['target_corner_margin_min_px'] = min_or_none(corner_margins)
    edge_lengths = finite_values(steady_rows, 'target_min_edge_px')
    metrics['target_min_edge_min_px'] = min_or_none(edge_lengths)
    polygon_areas = finite_values(steady_rows, 'target_polygon_area_px2')
    metrics['target_polygon_area_min_px2'] = min_or_none(polygon_areas)

    tag_ranges = finite_values(steady_rows, 'tag_range_m')
    metrics['tag_range_min_m'] = min_or_none(tag_ranges)
    metrics['tag_range_max_m'] = max_or_none(tag_ranges)
    optical_angles = finite_values(steady_rows, 'tag_optical_axis_angle_rad')
    metrics['tag_optical_axis_angle_max_rad'] = max_or_none(optical_angles)
    incidence_angles = finite_values(steady_rows, 'tag_incidence_angle_rad')
    metrics['tag_incidence_angle_max_rad'] = max_or_none(incidence_angles)

    widths = finite_values(steady_rows, 'camera_width_px')
    heights = finite_values(steady_rows, 'camera_height_px')
    metrics['camera_width_min_px'] = min_or_none(widths)
    metrics['camera_width_max_px'] = max_or_none(widths)
    metrics['camera_height_min_px'] = min_or_none(heights)
    metrics['camera_height_max_px'] = max_or_none(heights)

    valid_states = [parse_bool(row.get('visual_valid')) for row in steady_rows]
    valid_states = [value for value in valid_states if value is not None]
    metrics['visual_valid_fraction'] = (
        sum(1 for value in valid_states if value) / len(valid_states)
        if valid_states else None)
    visual_ages = finite_values(steady_rows, 'visual_pose_age_sec')
    metrics['visual_pose_age_max_sec'] = max_or_none(visual_ages)

    metrics['replay_samples'] = len(replay_rows)
    set_states = [parse_bool(row.get('set_success')) for row in replay_rows]
    metrics['set_failures'] = sum(1 for value in set_states if value is not True)
    raw_tracking_errors = finite_values(replay_rows, 'pose_tracking_error_m')
    evaluable_tracking_errors = []
    sim_times = []
    previous_sim_time = None
    duplicate_sim_time_rows = 0
    late_schedule_rows = 0
    for row in replay_rows:
        sim_time = parse_float(row.get('sim_time_sec'))
        schedule_error = parse_float(row.get('schedule_error_sec'))
        if sim_time is not None:
            sim_times.append(sim_time)
        sim_time_advanced = (
            previous_sim_time is None
            or sim_time is None
            or sim_time > previous_sim_time)
        schedule_aligned = (
            schedule_error is not None and abs(schedule_error) <= 1e-9)
        if not sim_time_advanced:
            duplicate_sim_time_rows += 1
        if not schedule_aligned:
            late_schedule_rows += 1
        if sim_time_advanced and schedule_aligned:
            tracking_error = parse_float(row.get('pose_tracking_error_m'))
            if tracking_error is not None and math.isfinite(tracking_error):
                evaluable_tracking_errors.append(tracking_error)
        if sim_time is not None:
            previous_sim_time = sim_time
    schedule_errors = [
        abs(value) for value in finite_values(replay_rows, 'schedule_error_sec')]
    sim_time_gaps = [
        current - previous
        for previous, current in zip(sim_times, sim_times[1:])
        if current >= previous
    ]
    metrics['base_tracking_error_raw_max_m'] = max_or_none(
        raw_tracking_errors)
    metrics['base_tracking_error_max_m'] = max_or_none(
        evaluable_tracking_errors)
    metrics['base_tracking_evaluable_samples'] = len(
        evaluable_tracking_errors)
    metrics['replay_duplicate_sim_time_rows'] = duplicate_sim_time_rows
    metrics['replay_late_schedule_rows'] = late_schedule_rows
    metrics['replay_schedule_error_max_sec'] = max_or_none(schedule_errors)
    metrics['replay_sim_time_gap_max_sec'] = max_or_none(sim_time_gaps)
    command_peak = 0.0
    for row in replay_rows:
        command = parse_semicolon_vector(row.get('velocity_command'))
        command_peak = max(
            command_peak,
            max((abs(value) for value in command), default=0.0))
    metrics['joint_velocity_command_peak_rad_s'] = command_peak

    camera_quaternions = [
        quaternion_from_row(row, 'actual_camera') for row in replay_rows]
    camera_quaternions = [value for value in camera_quaternions if value is not None]
    if camera_quaternions:
        initial = camera_quaternions[0]
        metrics['camera_orientation_drift_max_rad'] = max(
            quaternion_angle(initial, current) for current in camera_quaternions)
    else:
        metrics['camera_orientation_drift_max_rad'] = None
    return metrics


def evaluate(metrics, args):
    checks = {
        'enough_steady_rows': metrics['steady_visual_rows'] >= 100,
        'replay_complete': metrics['replay_samples'] == args.expected_replay_samples,
        'set_success': metrics['set_failures'] == 0,
        'base_tracking': (
            at_most(
                metrics['base_tracking_error_max_m'],
                args.max_base_tracking_error_m)
            and at_most(metrics['replay_schedule_error_max_sec'], 0.010000001)
            and at_most(metrics['replay_sim_time_gap_max_sec'], 0.020000001)),
        'open_loop_zero_command': at_most(
            metrics['joint_velocity_command_peak_rad_s'],
            args.max_joint_velocity_command_rad_s),
        'camera_resolution': (
            metrics['camera_width_min_px'] == args.expected_camera_width_px
            and metrics['camera_width_max_px'] == args.expected_camera_width_px
            and metrics['camera_height_min_px'] == args.expected_camera_height_px
            and metrics['camera_height_max_px'] == args.expected_camera_height_px),
        'image_rate': at_least(
            metrics['image_rate_hz_p05'], args.min_source_rate_hz),
        'camera_info_rate': at_least(
            metrics['camera_info_rate_hz_p05'], args.min_source_rate_hz),
        'apriltag_message_rate': at_least(
            metrics['apriltag_msg_rate_hz_p05'], args.min_source_rate_hz),
        'target_detection_rate': at_least(
            metrics['target_detection_rate_hz_p05'], args.min_visual_rate_hz),
        'target_frame_coverage': at_least(
            metrics['target_frame_coverage'], args.min_target_frame_coverage),
        'family_detection_count': at_least(
            metrics['family_detection_count_min'], 1.0),
        'visual_pose_rate': at_least(
            metrics['visual_pose_rate_hz_p05'], args.min_visual_rate_hz),
        'visual_valid_fraction': at_least(
            metrics['visual_valid_fraction'], args.min_visual_valid_fraction),
        'visual_pose_age': at_most(
            metrics['visual_pose_age_max_sec'], args.max_visual_pose_age_sec),
        'decision_margin': at_least(
            metrics['target_decision_margin_p05'], args.min_decision_margin),
        'corner_margin': at_least(
            metrics['target_corner_margin_min_px'], args.min_corner_margin_px),
        'tag_edge_length': at_least(
            metrics['target_min_edge_min_px'], args.min_tag_edge_px),
        'tag_polygon_area': at_least(
            metrics['target_polygon_area_min_px2'], args.min_tag_area_px2),
        'tag_range': in_range(
            metrics['tag_range_min_m'],
            metrics['tag_range_max_m'],
            args.min_tag_range_m,
            args.max_tag_range_m),
        'optical_axis_angle': at_most(
            metrics['tag_optical_axis_angle_max_rad'],
            args.max_optical_axis_angle_rad),
        'incidence_angle': at_most(
            metrics['tag_incidence_angle_max_rad'],
            args.max_incidence_angle_rad),
        'camera_orientation_drift': at_most(
            metrics['camera_orientation_drift_max_rad'],
            args.max_camera_orientation_drift_rad),
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
        description='Evaluate phase 4 camera visibility and pose observability.')
    parser.add_argument('--visual-chain-csv', required=True)
    parser.add_argument('--replay-csv', required=True)
    parser.add_argument('--label', default='')
    parser.add_argument('--output-json', default='')
    parser.add_argument('--output-csv', default='')
    parser.add_argument('--steady-start-wall-sec', type=float, default=8.0)
    parser.add_argument('--expected-replay-samples', type=int, default=3001)
    parser.add_argument('--expected-camera-width-px', type=int, default=320)
    parser.add_argument('--expected-camera-height-px', type=int, default=240)
    parser.add_argument('--min-source-rate-hz', type=float, default=12.0)
    parser.add_argument('--min-visual-rate-hz', type=float, default=10.0)
    parser.add_argument('--min-target-frame-coverage', type=float, default=0.95)
    parser.add_argument('--min-visual-valid-fraction', type=float, default=0.95)
    parser.add_argument('--max-visual-pose-age-sec', type=float, default=0.20)
    parser.add_argument('--min-decision-margin', type=float, default=20.0)
    parser.add_argument('--min-corner-margin-px', type=float, default=8.0)
    parser.add_argument('--min-tag-edge-px', type=float, default=20.0)
    parser.add_argument('--min-tag-area-px2', type=float, default=400.0)
    parser.add_argument('--min-tag-range-m', type=float, default=0.90)
    parser.add_argument('--max-tag-range-m', type=float, default=1.40)
    parser.add_argument('--max-optical-axis-angle-rad', type=float, default=0.35)
    parser.add_argument('--max-incidence-angle-rad', type=float, default=0.35)
    parser.add_argument('--max-camera-orientation-drift-rad', type=float, default=0.01)
    parser.add_argument('--max-base-tracking-error-m', type=float, default=0.0021)
    parser.add_argument('--max-joint-velocity-command-rad-s', type=float, default=1e-9)
    parser.add_argument('--require-pass', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    metrics = compute_metrics(args)
    checks = evaluate(metrics, args)
    payload = write_outputs(metrics, checks, args)
    print(json.dumps(payload, indent=2, sort_keys=True))
    if args.require_pass and not payload['overall_passed']:
        raise SystemExit('phase 4 visibility/observability gate failed')


if __name__ == '__main__':
    main()
