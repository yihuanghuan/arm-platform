#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from collections import Counter


def parse_float(value):
    if value in (None, ''):
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def parse_bool(value):
    if value in (None, ''):
        return None
    return str(value).strip().lower() in ('1', 'true', 'yes')


def load_rows(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, 'r', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def vector_from_row(row, prefix):
    values = [
        parse_float(row.get(f'{prefix}_x')),
        parse_float(row.get(f'{prefix}_y')),
        parse_float(row.get(f'{prefix}_z')),
    ]
    if any(value is None for value in values):
        return None
    return values


def parse_vector_text(value):
    if value in (None, ''):
        return []
    result = []
    for part in str(value).split(';'):
        parsed = parse_float(part)
        if parsed is None:
            return []
        result.append(parsed)
    return result


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def summarize_position_span(rows, prefix):
    positions = [vector_from_row(row, prefix) for row in rows]
    positions = [position for position in positions if position is not None]
    if not positions:
        return {
            'samples': 0,
            'axis_span_m': None,
            'span_norm_m': None,
            'start_to_end_m': None,
        }
    axis_span = [
        max(position[index] for position in positions)
        - min(position[index] for position in positions)
        for index in range(3)
    ]
    start_to_end = [
        positions[-1][index] - positions[0][index]
        for index in range(3)
    ]
    return {
        'samples': len(positions),
        'axis_span_m': axis_span,
        'span_norm_m': norm(axis_span),
        'start_to_end_m': norm(start_to_end),
    }


def summarize_pair_delta(rows, left_prefix, right_prefix):
    deltas = []
    for row in rows:
        left = vector_from_row(row, left_prefix)
        right = vector_from_row(row, right_prefix)
        if left is None or right is None:
            continue
        deltas.append(norm([left[index] - right[index] for index in range(3)]))
    return {
        'samples': len(deltas),
        'mean_m': sum(deltas) / len(deltas) if deltas else None,
        'max_m': max(deltas) if deltas else None,
    }


def summarize_transform_chain(rows):
    summary = {
        'rows': len(rows),
        'base_gt': summarize_position_span(rows, 'base_gt'),
        'link6_gt': summarize_position_span(rows, 'link6_gt'),
        'camera_tag_gt': summarize_position_span(rows, 'camera_tag_gt'),
        'camera_tag_measured': summarize_position_span(rows, 'camera_tag_measured'),
        'visual_ee': summarize_position_span(rows, 'visual_ee'),
        'camera_tag_measurement_error': summarize_pair_delta(
            rows, 'camera_tag_measured', 'camera_tag_gt'),
    }
    lookup_values = [parse_bool(row.get('tag_tf_lookup_ok')) for row in rows]
    lookup_values = [value for value in lookup_values if value is not None]
    summary['tag_tf_lookup_ok_rate'] = (
        sum(1 for value in lookup_values if value) / len(lookup_values)
        if lookup_values else None)
    config_errors = [
        parse_float(row.get('world_to_tag_config_translation_error_m'))
        for row in rows
    ]
    config_errors = [value for value in config_errors if value is not None]
    summary['world_to_tag_config_translation_error_max_m'] = (
        max(config_errors) if config_errors else None)
    return summary


def summarize_visual_chain(rows):
    valid_values = [parse_bool(row.get('visual_valid')) for row in rows]
    valid_values = [value for value in valid_values if value is not None]
    reasons = Counter(row.get('visual_debug_reason', '') for row in rows)
    selected_tags = Counter()
    full_vs_kinematic = []
    payload_count = 0
    for row in rows:
        payload_text = row.get('visual_debug_payload') or ''
        if not payload_text:
            continue
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            continue
        payload_count += 1
        selected = payload.get('selected_tag_id')
        if selected is not None:
            selected_tags[str(selected)] += 1
        full_pose = payload.get('world_to_ee_full')
        kinematic = payload.get('world_to_ee_kinematic')
        if (
                isinstance(full_pose, list)
                and isinstance(kinematic, list)
                and len(full_pose) >= 3
                and len(kinematic) >= 3):
            full_vs_kinematic.append(norm([
                float(full_pose[index]) - float(kinematic[index])
                for index in range(3)
            ]))
    return {
        'rows': len(rows),
        'visual_valid_samples': len(valid_values),
        'visual_loss_rate': (
            1.0 - sum(1 for value in valid_values if value) / len(valid_values)
            if valid_values else None),
        'visual_debug_reasons': dict(reasons),
        'debug_payload_rows': payload_count,
        'selected_tag_counts': dict(selected_tags),
        'full_vs_kinematic_delta_mean_m': (
            sum(full_vs_kinematic) / len(full_vs_kinematic)
            if full_vs_kinematic else None),
        'full_vs_kinematic_delta_max_m': (
            max(full_vs_kinematic) if full_vs_kinematic else None),
    }


def summarize_replay(rows):
    valid_values = [parse_bool(row.get('visual_valid')) for row in rows]
    valid_values = [value for value in valid_values if value is not None]
    visual_errors = []
    command_peaks = []
    for row in rows:
        visual_error = parse_vector_text(row.get('visual_error'))
        if len(visual_error) >= 3:
            visual_errors.append(norm(visual_error[:3]))
        command = parse_vector_text(row.get('velocity_command'))
        if command:
            command_peaks.append(max(abs(value) for value in command))
    return {
        'rows': len(rows),
        'visual_loss_rate': (
            1.0 - sum(1 for value in valid_values if value) / len(valid_values)
            if valid_values else None),
        'visual_error_max_m': max(visual_errors) if visual_errors else None,
        'joint_velocity_peak_rad_s': max(command_peaks) if command_peaks else None,
    }


def classify(summary):
    chain = summary.get('transform_chain', {})
    visual = summary.get('visual_chain', {})
    camera_gt_span = (chain.get('camera_tag_gt') or {}).get('span_norm_m')
    camera_measured_span = (chain.get('camera_tag_measured') or {}).get('span_norm_m')
    visual_span = (chain.get('visual_ee') or {}).get('span_norm_m')
    loss_rate = visual.get('visual_loss_rate')
    notes = []
    if loss_rate is not None and loss_rate > 0.1:
        notes.append('visual target loss is still significant')
    if (
            camera_gt_span is not None
            and camera_gt_span > 0.01
            and camera_measured_span is not None
            and camera_measured_span < 0.003):
        notes.append('camera->tag TF is not reflecting GT relative motion')
    if (
            camera_measured_span is not None
            and camera_measured_span > 0.003
            and visual_span is not None
            and visual_span < 0.003):
        notes.append('visual EE output is not following measured camera->tag motion')
    if not notes:
        notes.append('no obvious transform-chain blocker found in the provided CSVs')
    return notes


def format_value(value):
    if value is None:
        return 'n/a'
    if isinstance(value, float):
        return f'{value:.6g}'
    if isinstance(value, list):
        return '[' + ', '.join(format_value(item) for item in value) + ']'
    return str(value)


def write_markdown(path, summary):
    if not path:
        return
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    chain = summary['transform_chain']
    visual = summary['visual_chain']
    replay = summary['replay']
    lines = [
        '# Phase4.3 Transform Chain Analysis',
        '',
        '## Diagnosis',
    ]
    for note in summary['classification']:
        lines.append(f'- {note}')
    lines.extend([
        '',
        '## Transform CSV',
        f'- rows: {chain["rows"]}',
        f'- link6_gt span norm m: {format_value(chain["link6_gt"]["span_norm_m"])}',
        f'- camera_tag_gt span norm m: {format_value(chain["camera_tag_gt"]["span_norm_m"])}',
        f'- camera_tag_measured span norm m: {format_value(chain["camera_tag_measured"]["span_norm_m"])}',
        f'- visual_ee span norm m: {format_value(chain["visual_ee"]["span_norm_m"])}',
        f'- tag TF lookup ok rate: {format_value(chain["tag_tf_lookup_ok_rate"])}',
        f'- camera tag measurement error max m: {format_value(chain["camera_tag_measurement_error"]["max_m"])}',
        '',
        '## Visual Chain CSV',
        f'- rows: {visual["rows"]}',
        f'- visual loss rate: {format_value(visual["visual_loss_rate"])}',
        f'- selected tag counts: {visual["selected_tag_counts"]}',
        f'- full vs kinematic max delta m: {format_value(visual["full_vs_kinematic_delta_max_m"])}',
        f'- debug reasons: {visual["visual_debug_reasons"]}',
        '',
        '## Replay CSV',
        f'- rows: {replay["rows"]}',
        f'- visual loss rate: {format_value(replay["visual_loss_rate"])}',
        f'- visual error max m: {format_value(replay["visual_error_max_m"])}',
        f'- joint velocity peak rad/s: {format_value(replay["joint_velocity_peak_rad_s"])}',
        '',
    ])
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Analyze phase4.3 transform-chain, visual-chain, and replay CSVs.')
    parser.add_argument('--transform-csv',
                        default='/tmp/phase4_3_visual_pose_transform_chain.csv')
    parser.add_argument('--visual-chain-csv',
                        default='/tmp/phase4_visual_chain_diagnostics.csv')
    parser.add_argument('--replay-csv',
                        default='/tmp/base_disturbance_replay.csv')
    parser.add_argument('--output-md',
                        default='/tmp/phase4_3_transform_chain_analysis.md')
    parser.add_argument('--output-json', default='')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    summary = {
        'transform_chain': summarize_transform_chain(load_rows(args.transform_csv)),
        'visual_chain': summarize_visual_chain(load_rows(args.visual_chain_csv)),
        'replay': summarize_replay(load_rows(args.replay_csv)),
    }
    summary['classification'] = classify(summary)
    write_markdown(args.output_md, summary)
    if args.output_json:
        with open(args.output_json, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
