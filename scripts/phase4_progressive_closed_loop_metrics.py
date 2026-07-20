#!/usr/bin/env python3

import argparse
import csv
import json
import math
import statistics
import sys


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


def parse_vector(value, expected_length=None):
    if not value:
        return []
    try:
        parts = value if isinstance(value, (list, tuple)) else str(value).split(';')
        result = [float(part) for part in parts]
    except (TypeError, ValueError):
        return []
    if expected_length is not None and len(result) != expected_length:
        return []
    return result if all(math.isfinite(item) for item in result) else []


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = fraction * (len(values) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return values[lower]
    weight = index - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def rms(values):
    return math.sqrt(statistics.mean(value * value for value in values)) \
        if values else None


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def at_most(value, threshold):
    return bool(value is not None and value <= threshold)


def at_least(value, threshold):
    return bool(value is not None and value >= threshold)


def vector_from_row(row, prefix):
    values = [parse_float(row.get(f'{prefix}_{axis}')) for axis in 'xyz']
    return None if any(value is None for value in values) else values


def quaternion_from_row(row, prefix):
    values = [parse_float(row.get(f'{prefix}_q{axis}')) for axis in 'xyzw']
    if any(value is None for value in values):
        return None
    length = norm(values)
    return [value / length for value in values] if length > 1e-12 else None


def quaternion_angle(left, right):
    cosine = abs(sum(a * b for a, b in zip(left, right)))
    return 2.0 * math.acos(max(-1.0, min(1.0, cosine)))


def max_stale_burst(ages, threshold, sample_rate_hz):
    longest = 0
    current = 0
    for age in ages:
        if age > threshold:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest / sample_rate_hz


def controller_config_matches(status, args):
    if status.get('dry_run') is not False or status.get('control_mode') != 'xyz':
        return False
    vector_expectations = {
        'task_gain': args.task_gain_xyz + args.task_gain_orientation,
        'task_gain_xyz': args.task_gain_xyz,
        'max_task_velocity_xyz': args.max_task_velocity_xyz,
    }
    for key, expected in vector_expectations.items():
        actual = status.get(key)
        if not isinstance(actual, list) or len(actual) != len(expected):
            return False
        if any(abs(float(a) - b) > 1e-9 for a, b in zip(actual, expected)):
            return False
    scalar_expectations = {
        'max_joint_velocity': args.max_joint_velocity,
        'max_joint_acceleration_rad_s2': args.max_joint_acceleration_rad_s2,
        'damping': args.damping,
        'position_deadband_m': args.position_deadband_m,
        'measurement_timeout_sec': args.measurement_timeout_sec,
        'orientation_deadband_rad': args.orientation_deadband_rad,
    }
    return (
        status.get('preserve_kinematic_orientation') is True
        and all(
            parse_float(status.get(key)) is not None
            and abs(float(status[key]) - expected) <= 1e-9
            for key, expected in scalar_expectations.items()))


def summarize(path, args, visual):
    with open(path, 'r', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))

    origin = None
    origin_orientation = None
    errors = []
    ee_orientation_drifts = []
    steady_errors = []
    base_errors = []
    orientation_errors = []
    schedule_errors = []
    sim_times = []
    command_peaks = []
    replay_command_accelerations = []
    controller_command_accelerations = []
    controller_control_gaps = []
    saturated = 0
    valid_commands = 0
    invalid_command_samples = 0
    previous_command = None
    previous_trajectory_time = None
    previous_controller_command = None
    previous_controller_stamp = None
    previous_controller_state = None
    controller_stamp_regressions = 0
    controller_control_stamp_samples = 0
    visual_valid = []
    ages = []
    statuses = []
    status_parse_failures = 0
    holding_command_peak = 0.0
    set_failures = 0

    trajectory_end = parse_float(rows[-1].get('trajectory_time_sec')) if rows else None
    for row in rows:
        if parse_bool(row.get('set_success')) is not True:
            set_failures += 1
        position = vector_from_row(row, 'actual_link6')
        orientation = quaternion_from_row(row, 'actual_link6')
        if position is not None and max(abs(value) for value in position) <= 5.0:
            if origin is None:
                origin = position
            errors.append(norm([
                value - initial for value, initial in zip(position, origin)]))
            trajectory_time = parse_float(row.get('trajectory_time_sec'))
            if (
                    trajectory_time is not None
                    and trajectory_end is not None
                    and trajectory_time >= trajectory_end - args.steady_window_sec):
                steady_errors.append(errors[-1])
        if orientation is not None:
            if origin_orientation is None:
                origin_orientation = orientation
            ee_orientation_drifts.append(quaternion_angle(
                origin_orientation, orientation))

        base_error = parse_float(row.get('pose_tracking_error_m'))
        orientation_error = parse_float(row.get('orientation_tracking_error_rad'))
        schedule_error = parse_float(row.get('schedule_error_sec'))
        sim_time = parse_float(row.get('sim_time_sec'))
        if base_error is not None:
            base_errors.append(base_error)
        if orientation_error is not None:
            orientation_errors.append(orientation_error)
        if schedule_error is not None:
            schedule_errors.append(abs(schedule_error))
        if sim_time is not None:
            sim_times.append(sim_time)

        command = parse_vector(row.get('velocity_command'), 6)
        trajectory_time = parse_float(row.get('trajectory_time_sec'))
        if command:
            peak = max(abs(value) for value in command)
            command_peaks.append(peak)
            valid_commands += 1
            if peak >= args.max_joint_velocity - args.saturation_epsilon:
                saturated += 1
            if previous_command is not None and previous_trajectory_time is not None:
                dt = trajectory_time - previous_trajectory_time
                if dt > 0.0:
                    replay_command_accelerations.append(max(
                        abs(current - previous) / dt
                        for current, previous in zip(command, previous_command)))
            previous_command = command
            previous_trajectory_time = trajectory_time
        elif visual:
            invalid_command_samples += 1

        valid = parse_bool(row.get('visual_valid'))
        age = parse_float(row.get('latest_visual_pose_age_sec'))
        if valid is not None:
            visual_valid.append(valid)
        if age is not None:
            ages.append(age)

        raw_status = row.get('visual_status', '')
        if visual and raw_status:
            try:
                status = json.loads(raw_status)
            except json.JSONDecodeError:
                status_parse_failures += 1
                continue
            statuses.append(status)
            control_stamp = parse_float(status.get('control_stamp_sec'))
            status_command = parse_vector(status.get('dq_command'), 6)
            controller_state = status.get('safety_state')
            if control_stamp is not None and status_command:
                if (
                        previous_controller_stamp is not None
                        and control_stamp < previous_controller_stamp):
                    controller_stamp_regressions += 1
                elif (
                        previous_controller_stamp is None
                        or control_stamp > previous_controller_stamp):
                    if previous_controller_stamp is not None:
                        dt = control_stamp - previous_controller_stamp
                        controller_control_gaps.append(dt)
                        if (
                                previous_controller_state == 'TRACKING'
                                and controller_state == 'TRACKING'):
                            controller_command_accelerations.append(max(
                                abs(current - previous) / dt
                                for current, previous in zip(
                                    status_command,
                                    previous_controller_command)))
                    previous_controller_command = status_command
                    previous_controller_stamp = control_stamp
                    previous_controller_state = controller_state
                    controller_control_stamp_samples += 1
            if status.get('safety_state') == 'HOLDING_INPUT_LOSS':
                if status_command:
                    holding_command_peak = max(
                        holding_command_peak,
                        max(abs(value) for value in status_command))

    sim_gaps = [
        right - left for left, right in zip(sim_times, sim_times[1:])
        if right >= left]
    config_matches = [controller_config_matches(status, args) for status in statuses]
    fault_count = sum(
        status.get('safety_state') == 'FAULT_LATCHED'
        or bool(status.get('safety_stop')) for status in statuses)
    lock_counts = [int(status.get('target_lock_count', 0)) for status in statuses]
    reset_counts = [int(status.get('target_reset_count', 0)) for status in statuses]
    holding_count = sum(
        status.get('safety_state') == 'HOLDING_INPUT_LOSS'
        for status in statuses)

    return {
        'path': path,
        'samples': len(rows),
        'set_failures': set_failures,
        'base_tracking_error_max_m': max(base_errors) if base_errors else None,
        'base_orientation_error_max_rad': (
            max(orientation_errors) if orientation_errors else None),
        'schedule_error_max_sec': max(schedule_errors) if schedule_errors else None,
        'sim_time_gap_max_sec': max(sim_gaps) if sim_gaps else None,
        'xyz_rms_m': rms(errors),
        'xyz_max_m': max(errors) if errors else None,
        'steady_xyz_mean_m': statistics.mean(steady_errors) if steady_errors else None,
        'steady_xyz_max_m': max(steady_errors) if steady_errors else None,
        'ee_orientation_drift_max_rad': (
            max(ee_orientation_drifts) if ee_orientation_drifts else None),
        'command_peak_rad_s': max(command_peaks) if command_peaks else 0.0,
        'command_acceleration_peak_rad_s2': (
            max(controller_command_accelerations)
            if visual and controller_command_accelerations
            else max(replay_command_accelerations)
            if replay_command_accelerations else 0.0),
        'command_saturation_fraction': (
            saturated / valid_commands if valid_commands else 0.0),
        'invalid_command_samples': invalid_command_samples,
        'visual_valid_fraction': (
            sum(visual_valid) / len(visual_valid) if visual_valid else None),
        'visual_pose_age_p99_sec': percentile(ages, 0.99),
        'visual_pose_age_max_sec': max(ages) if ages else None,
        'visual_pose_stale_fraction': (
            sum(age > args.stale_age_threshold_sec for age in ages) / len(ages)
            if ages else None),
        'visual_pose_stale_burst_max_sec': max_stale_burst(
            ages, args.stale_age_threshold_sec, args.replay_rate_hz)
        if ages else None,
        'controller_status_samples': len(statuses),
        'controller_control_stamp_samples': controller_control_stamp_samples,
        'controller_control_gap_max_sec': (
            max(controller_control_gaps) if controller_control_gaps else None),
        'controller_stamp_regressions': controller_stamp_regressions,
        'status_parse_failures': status_parse_failures,
        'controller_config_fraction': (
            sum(config_matches) / len(config_matches) if config_matches else None),
        'fault_status_count': fault_count,
        'holding_status_fraction': holding_count / len(statuses) if statuses else 0.0,
        'holding_command_peak_rad_s': holding_command_peak,
        'target_lock_count_max': max(lock_counts) if lock_counts else None,
        'target_reset_count_max': max(reset_counts) if reset_counts else None,
    }


def common_checks(summary, args, visual):
    checks = {
        'replay_complete': (
            summary['samples'] == args.expected_samples
            and summary['set_failures'] == 0),
        'base_tracking': (
            at_most(summary['base_tracking_error_max_m'],
                    args.max_base_tracking_error_m)
            and at_most(summary['base_orientation_error_max_rad'],
                        args.max_base_orientation_error_rad)
            and at_most(summary['schedule_error_max_sec'], 0.010000001)
            and at_most(summary['sim_time_gap_max_sec'], 0.020000001)),
        'finite_continuous_command': (
            summary['invalid_command_samples'] == 0
            and at_most(summary['command_peak_rad_s'],
                        args.max_joint_velocity + args.command_tolerance)
            and at_most(summary['command_acceleration_peak_rad_s2'],
                        args.max_joint_acceleration_rad_s2
                        + args.acceleration_tolerance)
            and at_most(summary['command_saturation_fraction'],
                        args.max_command_saturation_fraction)),
    }
    if not visual:
        return checks
    checks.update({
        'controller_config': (
            summary['status_parse_failures'] == 0
            and summary['controller_stamp_regressions'] == 0
            and summary['controller_control_stamp_samples'] >= 2
            and at_most(
                summary['controller_control_gap_max_sec'],
                args.max_controller_control_gap_sec)
            and summary['controller_config_fraction'] == 1.0),
        'visual_liveness': (
            at_least(summary['visual_valid_fraction'],
                     args.min_visual_valid_fraction)
            and at_most(summary['visual_pose_age_p99_sec'],
                        args.max_visual_pose_age_p99_sec)
            and at_most(summary['visual_pose_stale_fraction'],
                        args.max_visual_pose_stale_fraction)
            and summary['visual_pose_stale_burst_max_sec']
            < args.max_visual_pose_stale_burst_sec),
        'safety_target': (
            summary['fault_status_count'] == 0
            and summary['target_lock_count_max'] == 1
            and summary['target_reset_count_max'] == 0
            and at_most(summary['holding_command_peak_rad_s'], 1e-9)),
        'ee_orientation_preservation': at_most(
            summary['ee_orientation_drift_max_rad'],
            args.max_ee_orientation_drift_rad),
    })
    return checks


def evaluate(baseline, visual, args):
    checks = {
        f'baseline_{key}': value
        for key, value in common_checks(baseline, args, False).items()
    } if baseline else {}
    checks.update({
        f'visual_{key}': value
        for key, value in common_checks(visual, args, True).items()
    })
    if args.mode == 'static':
        checks['static_stability'] = (
            at_most(visual['xyz_rms_m'], args.max_static_rms_m)
            and at_most(visual['steady_xyz_max_m'], args.max_steady_error_m))
    elif args.mode == 'step':
        checks['step_recovery'] = (
            at_most(visual['steady_xyz_mean_m'], args.max_steady_error_m)
            and at_most(visual['steady_xyz_max_m'], args.max_steady_error_m))
    else:
        checks['dynamic_improvement'] = (
            baseline is not None
            and visual['xyz_rms_m'] < baseline['xyz_rms_m']
            and visual['steady_xyz_mean_m'] < baseline['steady_xyz_mean_m'])
    return checks


def parse_xyz(value):
    result = [float(part) for part in str(value).replace(',', ' ').split()]
    if len(result) != 3:
        raise argparse.ArgumentTypeError('expected three values')
    return result


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Evaluate Gate B progressive visual closed-loop runs.')
    parser.add_argument('--mode', choices=('static', 'step', 'dynamic'), required=True)
    parser.add_argument('--visual-csv', required=True)
    parser.add_argument('--baseline-csv', default='')
    parser.add_argument('--label', default='')
    parser.add_argument('--output-json', default='')
    parser.add_argument('--output-csv', default='')
    parser.add_argument('--expected-samples', type=int, required=True)
    parser.add_argument('--max-joint-velocity', type=float, default=0.2)
    parser.add_argument('--max-joint-acceleration-rad-s2', type=float, default=0.3)
    parser.add_argument('--max-task-velocity-xyz', type=parse_xyz,
                        default=parse_xyz('0.05 0.05 0.05'))
    parser.add_argument('--task-gain-xyz', type=parse_xyz,
                        default=parse_xyz('4 4 4'))
    parser.add_argument('--task-gain-orientation', type=parse_xyz,
                        default=parse_xyz('4 4 4'))
    parser.add_argument('--damping', type=float, default=0.05)
    parser.add_argument('--position-deadband-m', type=float, default=0.003)
    parser.add_argument('--orientation-deadband-rad', type=float, default=0.005)
    parser.add_argument('--max-ee-orientation-drift-rad', type=float, default=0.010)
    parser.add_argument('--measurement-timeout-sec', type=float, default=0.25)
    parser.add_argument('--max-base-tracking-error-m', type=float, default=0.0021)
    parser.add_argument('--max-base-orientation-error-rad', type=float, default=0.01)
    parser.add_argument('--max-command-saturation-fraction', type=float, default=0.20)
    parser.add_argument('--max-controller-control-gap-sec', type=float, default=0.030000001)
    parser.add_argument('--saturation-epsilon', type=float, default=1e-4)
    parser.add_argument('--command-tolerance', type=float, default=1e-6)
    parser.add_argument('--acceleration-tolerance', type=float, default=0.05)
    parser.add_argument('--min-visual-valid-fraction', type=float, default=0.95)
    parser.add_argument('--max-visual-pose-age-p99-sec', type=float, default=0.20)
    parser.add_argument('--stale-age-threshold-sec', type=float, default=0.25)
    parser.add_argument('--max-visual-pose-stale-fraction', type=float, default=0.01)
    parser.add_argument('--max-visual-pose-stale-burst-sec', type=float, default=0.50)
    parser.add_argument('--replay-rate-hz', type=float, default=100.0)
    parser.add_argument('--steady-window-sec', type=float, default=5.0)
    parser.add_argument('--max-static-rms-m', type=float, default=0.010)
    parser.add_argument('--max-steady-error-m', type=float, default=0.010)
    parser.add_argument('--require-pass', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.mode == 'dynamic' and not args.baseline_csv:
        raise SystemExit('--baseline-csv is required for dynamic mode')
    baseline = summarize(args.baseline_csv, args, False) \
        if args.baseline_csv else None
    visual = summarize(args.visual_csv, args, True)
    checks = evaluate(baseline, visual, args)
    payload = {
        'label': args.label,
        'mode': args.mode,
        'baseline': baseline,
        'visual': visual,
        'checks': checks,
        'overall_passed': all(checks.values()),
    }
    if args.output_json:
        with open(args.output_json, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
    if args.output_csv:
        row = {'label': args.label, 'mode': args.mode}
        if baseline:
            row.update({f'baseline_{key}': value for key, value in baseline.items()})
        row.update({f'visual_{key}': value for key, value in visual.items()})
        row.update({f'check_{key}': value for key, value in checks.items()})
        row['overall_passed'] = payload['overall_passed']
        with open(args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
    print(json.dumps(payload, indent=2, sort_keys=True))
    if args.require_pass and not payload['overall_passed']:
        raise SystemExit('phase 4 progressive closed-loop gate failed')


if __name__ == '__main__':
    main()
