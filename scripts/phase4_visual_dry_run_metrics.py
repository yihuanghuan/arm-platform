#!/usr/bin/env python3

import argparse
import bisect
import csv
import json
import math
import os
import sys

import numpy as np
import pinocchio as pin


def default_urdf_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(
            os.path.dirname(os.path.dirname(script_dir)),
            'share', 'manipulator', 'arm.urdf'),
        os.path.join(os.path.dirname(script_dir), 'config', 'arm.urdf'),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


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


def vector_from_row(row, prefix):
    result = [parse_float(row.get(f'{prefix}_{axis}')) for axis in 'xyz']
    return None if any(item is None for item in result) else result


def norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def subtract(left, right):
    return [a - b for a, b in zip(left, right)]


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
    return math.sqrt(sum(value * value for value in values) / len(values)) \
        if values else None


def unique_stage_samples(rows, stamp_field, receive_field):
    samples = {}
    regressions = 0
    previous_stamp = None
    for row in rows:
        stamp = parse_float(row.get(stamp_field))
        received = parse_float(row.get(receive_field))
        if stamp is None or received is None or stamp <= 0.0 or received <= 0.0:
            continue
        if previous_stamp is not None and stamp < previous_stamp - 1e-9:
            regressions += 1
        previous_stamp = stamp
        key = round(stamp, 9)
        samples[key] = min(received, samples.get(key, received))
    return samples, regressions


def matched_processing_delays(source_samples, destination_samples):
    return [
        destination_samples[stamp] - source_samples[stamp]
        for stamp in source_samples.keys() & destination_samples.keys()
        if destination_samples[stamp] >= source_samples[stamp]
    ]


def max_consecutive_fraction_duration(values, predicate, sample_rate_hz):
    longest = 0
    current = 0
    for value in values:
        if predicate(value):
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest / sample_rate_hz if sample_rate_hz > 0.0 else None


def slope(left, right):
    if len(left) < 2 or len(left) != len(right):
        return None
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    variance = sum((value - left_mean) ** 2 for value in left)
    if variance <= 0.0:
        return None
    covariance = sum(
        (a - left_mean) * (b - right_mean)
        for a, b in zip(left, right))
    return covariance / variance


def cosine(left, right):
    denominator = norm(left) * norm(right)
    if denominator <= 1e-12:
        return None
    return float(sum(a * b for a, b in zip(left, right)) / denominator)


def interpolate(times, vectors, stamp):
    index = bisect.bisect_left(times, stamp)
    if index <= 0:
        return vectors[0] if times and stamp >= times[0] else None
    if index >= len(times):
        return vectors[-1] if times and stamp <= times[-1] else None
    left_time = times[index - 1]
    right_time = times[index]
    if right_time <= left_time:
        return None
    ratio = (stamp - left_time) / (right_time - left_time)
    return [
        vectors[index - 1][axis] * (1.0 - ratio)
        + vectors[index][axis] * ratio
        for axis in range(len(vectors[index]))
    ]


def at_least(value, threshold):
    return bool(value is not None and value >= threshold)


def at_most(value, threshold):
    return bool(value is not None and value <= threshold)


class KinematicOracle:
    def __init__(self, urdf_path, ee_frame, joint_names, damping):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        if not self.model.existFrame(ee_frame):
            raise ValueError(f'End-effector frame not found: {ee_frame}')
        self.ee_frame_id = self.model.getFrameId(ee_frame)
        self.q_indices = []
        self.v_indices = []
        for name in joint_names:
            if not self.model.existJointName(name):
                raise ValueError(f'Joint not found: {name}')
            joint = self.model.joints[self.model.getJointId(name)]
            self.q_indices.append(joint.idx_q)
            self.v_indices.append(joint.idx_v)
        self.damping = damping

    def jacobian(self, joint_positions):
        q = pin.neutral(self.model)
        for index, value in zip(self.q_indices, joint_positions):
            q[index] = value
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        full = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED)
        jacobian = np.zeros((3, len(self.v_indices)), dtype=float)
        for output_column, velocity_index in enumerate(self.v_indices):
            jacobian[:, output_column] = full[:3, velocity_index]
        return jacobian


def controller_config_matches(status, args):
    expected_vectors = {
        'task_gain_xyz': args.task_gain_xyz,
        'max_task_velocity_xyz': args.max_task_velocity_xyz,
    }
    if status.get('dry_run') is not True or status.get('control_mode') != 'xyz':
        return False
    for key, expected in expected_vectors.items():
        actual = status.get(key)
        if not isinstance(actual, list) or len(actual) != len(expected):
            return False
        if any(abs(float(a) - b) > 1e-9 for a, b in zip(actual, expected)):
            return False
    expected_scalars = {
        'max_joint_velocity': args.max_joint_velocity,
        'max_joint_acceleration_rad_s2': args.max_joint_acceleration_rad_s2,
        'damping': args.damping,
        'position_deadband_m': args.position_deadband_m,
    }
    return all(
        parse_float(status.get(key)) is not None
        and abs(float(status[key]) - expected) <= 1e-9
        for key, expected in expected_scalars.items())


def compute_metrics(args):
    with open(args.replay_csv, 'r', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    oracle = KinematicOracle(
        args.urdf_path, args.ee_frame, args.joint_names, args.damping)

    gt_by_time = {}
    for row in rows:
        stamp = parse_float(row.get('sim_time_sec'))
        position = vector_from_row(row, 'actual_link6')
        if stamp is not None and position is not None:
            gt_by_time[stamp] = position
    gt_times = sorted(gt_by_time)
    gt_positions = [gt_by_time[stamp] for stamp in gt_times]
    gt_target = gt_positions[0] if gt_positions else None

    sample_rows = {}
    parsed_statuses = []
    status_parse_failures = 0
    for row in rows:
        raw_status = row.get('visual_status', '')
        if not raw_status:
            continue
        try:
            status = json.loads(raw_status)
        except json.JSONDecodeError:
            status_parse_failures += 1
            continue
        parsed_statuses.append(status)
        stamp = parse_float(status.get('latest_processed_visual_stamp_sec'))
        if stamp is not None and stamp > 0.0:
            sample_rows[round(stamp, 9)] = (row, status)

    error_differences = []
    direction_cosines = []
    gt_error_norms = []
    visual_error_norms = []
    axis_sign_results = []
    clik_direction_cosines = []
    clik_scale_ratios = []
    finite_samples = 0
    evaluated_samples = 0
    dq_target_peak = 0.0

    if gt_target is not None:
        for stamp in sorted(sample_rows):
            row, status = sample_rows[stamp]
            gt_position = interpolate(gt_times, gt_positions, stamp)
            visual_error = parse_vector(
                status.get('latest_control_update_error'), 6)
            dq_raw = parse_vector(status.get('dq_raw'), 6)
            dq_target = parse_vector(status.get('dq_target'), 6)
            dq_command = parse_vector(status.get('dq_command'), 6)
            joint_positions = parse_vector(row.get('joint_positions'), 6)
            if (
                    gt_position is None
                    or not visual_error
                    or not dq_raw
                    or not dq_target
                    or not dq_command
                    or not joint_positions):
                continue
            evaluated_samples += 1
            finite_samples += 1
            gt_error = subtract(gt_target, gt_position)
            visual_xyz = visual_error[:3]
            error_differences.append(norm(subtract(visual_xyz, gt_error)))
            gt_norm = norm(gt_error)
            visual_norm = norm(visual_xyz)
            gt_error_norms.append(gt_norm)
            visual_error_norms.append(visual_norm)
            if gt_norm >= args.direction_min_error_m and visual_norm > 1e-12:
                direction_cosines.append(cosine(gt_error, visual_xyz))
            for gt_axis, visual_axis in zip(gt_error, visual_xyz):
                if abs(gt_axis) >= args.axis_sign_min_error_m:
                    axis_sign_results.append(gt_axis * visual_axis > 0.0)

            dq_target_peak = max(
                dq_target_peak, max(abs(value) for value in dq_target))
            task_velocity = np.clip(
                np.asarray(args.task_gain_xyz) * np.asarray(visual_xyz),
                -np.asarray(args.max_task_velocity_xyz),
                np.asarray(args.max_task_velocity_xyz))
            if visual_norm <= args.position_deadband_m:
                task_velocity = np.zeros(3, dtype=float)
            task_norm = float(np.linalg.norm(task_velocity))
            if task_norm >= args.clik_min_task_velocity_m_s:
                jacobian = oracle.jacobian(joint_positions)
                achieved = jacobian @ np.asarray(dq_raw)
                achieved_norm = float(np.linalg.norm(achieved))
                achieved_cosine = cosine(task_velocity, achieved)
                if achieved_cosine is not None:
                    clik_direction_cosines.append(achieved_cosine)
                if achieved_norm > 1e-12:
                    clik_scale_ratios.append(achieved_norm / task_norm)

    valid_values = [parse_bool(row.get('visual_valid')) for row in rows]
    valid_values = [value for value in valid_values if value is not None]
    ages = [parse_float(row.get('latest_visual_pose_age_sec')) for row in rows]
    ages = [value for value in ages if value is not None]
    image_samples, image_stamp_regressions = unique_stage_samples(
        rows, 'latest_image_stamp_sec', 'latest_image_receive_sec')
    detection_samples, detection_stamp_regressions = unique_stage_samples(
        rows,
        'latest_target_detection_stamp_sec',
        'latest_target_detection_receive_sec')
    pose_samples, pose_stamp_regressions = unique_stage_samples(
        rows,
        'latest_visual_pose_stamp_sec',
        'latest_visual_pose_receive_sec')
    control_samples, control_stamp_regressions = unique_stage_samples(
        rows,
        'latest_controller_visual_stamp_sec',
        'latest_controller_status_receive_sec')
    image_latencies = [
        received - stamp for stamp, received in image_samples.items()
        if received >= stamp]
    detection_latencies = [
        received - stamp for stamp, received in detection_samples.items()
        if received >= stamp]
    pose_latencies = [
        received - stamp for stamp, received in pose_samples.items()
        if received >= stamp]
    control_latencies = [
        received - stamp for stamp, received in control_samples.items()
        if received >= stamp]
    image_to_detection_delays = matched_processing_delays(
        image_samples, detection_samples)
    detection_to_pose_delays = matched_processing_delays(
        detection_samples, pose_samples)
    pose_to_control_delays = matched_processing_delays(
        pose_samples, control_samples)
    command_peak = max((
        abs(value)
        for row in rows
        for value in parse_vector(row.get('velocity_command'))
    ), default=0.0)
    joint_max_step = max((
        parse_float(row.get('joint_max_step_rad')) or 0.0 for row in rows
    ), default=0.0)

    set_states = [parse_bool(row.get('set_success')) for row in rows]
    raw_tracking_errors = [
        parse_float(row.get('pose_tracking_error_m')) for row in rows]
    raw_tracking_errors = [value for value in raw_tracking_errors if value is not None]
    tracking_errors = []
    sim_times = []
    previous_sim_time = None
    schedule_errors = []
    for row in rows:
        sim_time = parse_float(row.get('sim_time_sec'))
        schedule_error = parse_float(row.get('schedule_error_sec'))
        if sim_time is not None:
            sim_times.append(sim_time)
        if schedule_error is not None:
            schedule_errors.append(abs(schedule_error))
        if (
                (previous_sim_time is None
                 or sim_time is None
                 or sim_time > previous_sim_time)
                and schedule_error is not None
                and abs(schedule_error) <= 1e-9):
            error = parse_float(row.get('pose_tracking_error_m'))
            if error is not None:
                tracking_errors.append(error)
        if sim_time is not None:
            previous_sim_time = sim_time
    sim_gaps = [
        right - left for left, right in zip(sim_times, sim_times[1:])
        if right >= left]

    config_states = [
        controller_config_matches(status, args) for status in parsed_statuses]
    fault_states = [
        status.get('safety_state') == 'FAULT_LATCHED'
        or bool(status.get('safety_stop'))
        for status in parsed_statuses]
    target_lock_counts = [
        int(status.get('target_lock_count', 0)) for status in parsed_statuses]
    target_reset_counts = [
        int(status.get('target_reset_count', 0)) for status in parsed_statuses]

    return {
        'label': args.label,
        'replay_samples': len(rows),
        'set_failures': sum(value is not True for value in set_states),
        'base_tracking_error_max_m': max(tracking_errors) if tracking_errors else None,
        'base_tracking_error_raw_max_m': (
            max(raw_tracking_errors) if raw_tracking_errors else None),
        'replay_schedule_error_max_sec': (
            max(schedule_errors) if schedule_errors else None),
        'replay_sim_time_gap_max_sec': max(sim_gaps) if sim_gaps else None,
        'controller_status_samples': len(parsed_statuses),
        'status_parse_failures': status_parse_failures,
        'controller_config_fraction': (
            sum(config_states) / len(config_states) if config_states else None),
        'fault_status_count': sum(fault_states),
        'target_lock_count_max': max(target_lock_counts) if target_lock_counts else None,
        'target_reset_count_max': max(target_reset_counts) if target_reset_counts else None,
        'actual_command_peak_rad_s': command_peak,
        'joint_max_step_rad': joint_max_step,
        'visual_valid_fraction': (
            sum(valid_values) / len(valid_values) if valid_values else None),
        'visual_pose_age_max_sec': max(ages) if ages else None,
        'visual_pose_age_p99_sec': percentile(ages, 0.99),
        'visual_pose_stale_fraction': (
            sum(value > args.stale_age_threshold_sec for value in ages) / len(ages)
            if ages else None),
        'visual_pose_stale_burst_max_sec': max_consecutive_fraction_duration(
            ages,
            lambda value: value > args.stale_age_threshold_sec,
            args.replay_rate_hz),
        'image_unique_samples': len(image_samples),
        'target_detection_unique_samples': len(detection_samples),
        'visual_pose_unique_timestamp_samples': len(pose_samples),
        'controller_consumed_unique_samples': len(control_samples),
        'timestamp_regression_count': (
            image_stamp_regressions
            + detection_stamp_regressions
            + pose_stamp_regressions
            + control_stamp_regressions),
        'image_source_to_replay_p99_sec': percentile(image_latencies, 0.99),
        'detection_source_to_replay_p99_sec': percentile(
            detection_latencies, 0.99),
        'visual_pose_source_to_replay_p99_sec': percentile(
            pose_latencies, 0.99),
        'control_source_to_status_p99_sec': percentile(
            control_latencies, 0.99),
        'image_to_detection_processing_p99_sec': percentile(
            image_to_detection_delays, 0.99),
        'detection_to_visual_pose_processing_p99_sec': percentile(
            detection_to_pose_delays, 0.99),
        'visual_pose_to_control_processing_p99_sec': percentile(
            pose_to_control_delays, 0.99),
        'unique_visual_samples': evaluated_samples,
        'finite_sample_fraction': (
            finite_samples / evaluated_samples if evaluated_samples else None),
        'visual_gt_error_difference_rms_m': rms(error_differences),
        'visual_gt_error_difference_p95_m': percentile(error_differences, 0.95),
        'visual_gt_direction_cosine_p05': percentile(direction_cosines, 0.05),
        'visual_gt_error_norm_slope': slope(gt_error_norms, visual_error_norms),
        'excited_axis_sign_agreement': (
            sum(axis_sign_results) / len(axis_sign_results)
            if axis_sign_results else None),
        'dq_target_peak_rad_s': dq_target_peak,
        'clik_task_direction_cosine_min': min(clik_direction_cosines)
        if clik_direction_cosines else None,
        'clik_task_scale_ratio_min': min(clik_scale_ratios)
        if clik_scale_ratios else None,
        'clik_task_scale_ratio_max': max(clik_scale_ratios)
        if clik_scale_ratios else None,
    }


def evaluate(metrics, args):
    return {
        'replay_complete': (
            metrics['replay_samples'] == args.expected_replay_samples
            and metrics['set_failures'] == 0),
        'base_tracking': (
            at_most(metrics['base_tracking_error_max_m'],
                    args.max_base_tracking_error_m)
            and at_most(metrics['replay_schedule_error_max_sec'], 0.010000001)
            and at_most(metrics['replay_sim_time_gap_max_sec'], 0.020000001)),
        'controller_config': (
            metrics['status_parse_failures'] == 0
            and metrics['controller_config_fraction'] == 1.0),
        'dry_run_isolation': (
            at_most(metrics['actual_command_peak_rad_s'],
                    args.max_actual_command_rad_s)
            and at_most(metrics['joint_max_step_rad'], args.max_joint_step_rad)),
        'visual_input': (
            at_least(metrics['visual_valid_fraction'],
                     args.min_visual_valid_fraction)
            and at_most(metrics['visual_pose_age_p99_sec'],
                        args.max_visual_pose_age_p99_sec)
            and at_most(metrics['visual_pose_stale_fraction'],
                        args.max_visual_pose_stale_fraction)
            and metrics['visual_pose_stale_burst_max_sec']
            < args.max_visual_pose_stale_burst_sec
            and metrics['unique_visual_samples'] >= args.min_visual_samples),
        'timestamp_chain': (
            metrics['image_unique_samples'] >= args.min_visual_samples
            and metrics['target_detection_unique_samples'] >= args.min_visual_samples
            and metrics['visual_pose_unique_timestamp_samples']
            >= args.min_visual_samples
            and metrics['controller_consumed_unique_samples']
            >= args.min_visual_samples
            and metrics['timestamp_regression_count'] == 0
            and all(
                metrics[key] is not None
                for key in (
                    'image_source_to_replay_p99_sec',
                    'detection_source_to_replay_p99_sec',
                    'visual_pose_source_to_replay_p99_sec',
                    'control_source_to_status_p99_sec',
                    'image_to_detection_processing_p99_sec',
                    'detection_to_visual_pose_processing_p99_sec',
                    'visual_pose_to_control_processing_p99_sec'))),
        'finite_outputs': metrics['finite_sample_fraction'] == 1.0,
        'visual_gt_difference': (
            at_most(metrics['visual_gt_error_difference_rms_m'],
                    args.max_visual_gt_difference_rms_m)
            and at_most(metrics['visual_gt_error_difference_p95_m'],
                        args.max_visual_gt_difference_p95_m)),
        'visual_gt_direction': at_least(
            metrics['visual_gt_direction_cosine_p05'],
            args.min_visual_gt_direction_cosine_p05),
        'visual_gt_scale': (
            at_least(metrics['visual_gt_error_norm_slope'],
                     args.min_visual_gt_error_norm_slope)
            and at_most(metrics['visual_gt_error_norm_slope'],
                        args.max_visual_gt_error_norm_slope)),
        'excited_axis_sign': at_least(
            metrics['excited_axis_sign_agreement'],
            args.min_excited_axis_sign_agreement),
        'nonzero_bounded_intent': (
            at_least(metrics['dq_target_peak_rad_s'], args.min_dq_target_peak_rad_s)
            and at_most(metrics['dq_target_peak_rad_s'],
                        args.max_joint_velocity)),
        'clik_task_direction': at_least(
            metrics['clik_task_direction_cosine_min'],
            args.min_clik_task_direction_cosine),
        'clik_task_scale': (
            at_least(metrics['clik_task_scale_ratio_min'],
                     args.min_clik_task_scale_ratio)
            and at_most(metrics['clik_task_scale_ratio_max'],
                        args.max_clik_task_scale_ratio)),
        'safety_target': (
            metrics['fault_status_count'] == 0
            and metrics['target_lock_count_max'] == 1
            and metrics['target_reset_count_max'] == 0),
    }


def parse_xyz(value):
    result = [float(part) for part in str(value).replace(',', ' ').split()]
    if len(result) != 3:
        raise argparse.ArgumentTypeError('expected three values')
    return result


def parse_names(value):
    result = [part for part in str(value).replace(',', ' ').split() if part]
    if len(result) != 6:
        raise argparse.ArgumentTypeError('expected six joint names')
    return result


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Evaluate phase 4 visual dry-run against Gazebo ground truth.')
    parser.add_argument('--replay-csv', required=True)
    parser.add_argument('--label', default='')
    parser.add_argument('--output-json', default='')
    parser.add_argument('--output-csv', default='')
    parser.add_argument('--urdf-path', default=default_urdf_path())
    parser.add_argument('--ee-frame', default='link6')
    parser.add_argument('--joint-names', type=parse_names,
                        default=parse_names('joint1 joint2 joint3 joint4 joint5 joint6'))
    parser.add_argument('--task-gain-xyz', type=parse_xyz,
                        default=parse_xyz('4 4 4'))
    parser.add_argument('--max-task-velocity-xyz', type=parse_xyz,
                        default=parse_xyz('0.05 0.05 0.05'))
    parser.add_argument('--max-joint-velocity', type=float, default=0.2)
    parser.add_argument('--max-joint-acceleration-rad-s2', type=float, default=0.3)
    parser.add_argument('--damping', type=float, default=0.05)
    parser.add_argument('--position-deadband-m', type=float, default=0.003)
    parser.add_argument('--expected-replay-samples', type=int, default=3001)
    parser.add_argument('--max-base-tracking-error-m', type=float, default=0.0021)
    parser.add_argument('--max-actual-command-rad-s', type=float, default=1e-9)
    parser.add_argument('--max-joint-step-rad', type=float, default=1e-6)
    parser.add_argument('--min-visual-valid-fraction', type=float, default=0.95)
    parser.add_argument('--max-visual-pose-age-p99-sec', type=float, default=0.20)
    parser.add_argument('--stale-age-threshold-sec', type=float, default=0.25)
    parser.add_argument('--max-visual-pose-stale-fraction', type=float, default=0.01)
    parser.add_argument('--max-visual-pose-stale-burst-sec', type=float, default=0.50)
    parser.add_argument('--replay-rate-hz', type=float, default=100.0)
    parser.add_argument('--min-visual-samples', type=int, default=400)
    parser.add_argument('--max-visual-gt-difference-rms-m', type=float, default=0.025)
    parser.add_argument('--max-visual-gt-difference-p95-m', type=float, default=0.040)
    parser.add_argument('--direction-min-error-m', type=float, default=0.01)
    parser.add_argument('--min-visual-gt-direction-cosine-p05',
                        type=float, default=0.65)
    parser.add_argument('--min-visual-gt-error-norm-slope', type=float, default=0.80)
    parser.add_argument('--max-visual-gt-error-norm-slope', type=float, default=1.20)
    parser.add_argument('--axis-sign-min-error-m', type=float, default=0.01)
    parser.add_argument('--min-excited-axis-sign-agreement', type=float, default=0.95)
    parser.add_argument('--min-dq-target-peak-rad-s', type=float, default=0.02)
    parser.add_argument('--clik-min-task-velocity-m-s', type=float, default=0.01)
    parser.add_argument('--min-clik-task-direction-cosine', type=float, default=0.995)
    parser.add_argument('--min-clik-task-scale-ratio', type=float, default=0.85)
    parser.add_argument('--max-clik-task-scale-ratio', type=float, default=1.05)
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    metrics = compute_metrics(args)
    checks = evaluate(metrics, args)
    payload = {
        'metrics': metrics,
        'checks': checks,
        'overall_passed': all(checks.values()),
    }
    if args.output_json:
        with open(args.output_json, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
    if args.output_csv:
        row = dict(metrics)
        row.update({f'check_{key}': value for key, value in checks.items()})
        row['overall_passed'] = payload['overall_passed']
        with open(args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
    print(json.dumps(payload, indent=2, sort_keys=True))
    if args.require_pass and not payload['overall_passed']:
        raise SystemExit('phase 4 visual dry-run gate failed')


if __name__ == '__main__':
    main()
