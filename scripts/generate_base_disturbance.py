#!/usr/bin/env python3

import argparse
import csv
import math
import os
import random
import sys

import yaml


AXES = ('x', 'y', 'z')
CSV_FIELDS = [
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


def deep_merge(base, override):
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_profile(config_path, profile_name):
    with open(config_path, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f'Invalid config file: {config_path}')
    defaults = config.get('defaults', {})
    profiles = config.get('profiles', {})
    if profile_name not in profiles:
        raise ValueError(
            f'Unknown profile "{profile_name}". Available: {", ".join(sorted(profiles))}')
    return deep_merge(defaults, profiles[profile_name])


def time_samples(duration_sec, sample_rate_hz):
    if duration_sec <= 0.0:
        raise ValueError('duration_sec must be positive')
    if sample_rate_hz <= 0.0:
        raise ValueError('sample_rate_hz must be positive')
    sample_count = int(round(duration_sec * sample_rate_hz)) + 1
    dt = 1.0 / sample_rate_hz
    return [index * dt for index in range(sample_count)]


def smooth_envelope(t, duration_sec, ramp_sec):
    if ramp_sec <= 0.0:
        return 1.0, 0.0
    ramp_sec = min(ramp_sec, duration_sec * 0.5)
    if t < ramp_sec:
        ratio = t / ramp_sec
        value = 0.5 - 0.5 * math.cos(math.pi * ratio)
        derivative = 0.5 * math.pi / ramp_sec * math.sin(math.pi * ratio)
        return value, derivative
    if t > duration_sec - ramp_sec:
        ratio = (duration_sec - t) / ramp_sec
        value = 0.5 - 0.5 * math.cos(math.pi * ratio)
        derivative = -0.5 * math.pi / ramp_sec * math.sin(math.pi * ratio)
        return value, derivative
    return 1.0, 0.0


def generate_static(times):
    positions = {axis: [0.0] * len(times) for axis in AXES}
    velocities = {axis: [0.0] * len(times) for axis in AXES}
    return positions, velocities


def generate_sine(times, profile):
    positions, velocities = generate_static(times)
    axis = str(profile.get('axis', 'x'))
    axes = AXES if axis == 'xyz' else (axis,)
    invalid = [item for item in axes if item not in AXES]
    if invalid:
        raise ValueError(f'sine axis must be one of {AXES} or xyz, got {axis}')
    amplitudes = profile['translation_amplitude']
    frequency_hz = float(profile.get('frequency_hz', profile.get('sine_frequency_hz', 0.2)))
    duration_sec = float(profile['duration_sec'])
    ramp_sec = float(profile.get('ramp_sec', min(2.0, duration_sec * 0.1)))
    omega = 2.0 * math.pi * frequency_hz
    for index, t in enumerate(times):
        envelope, envelope_dot = smooth_envelope(t, duration_sec, ramp_sec)
        raw = math.sin(omega * t)
        raw_dot = omega * math.cos(omega * t)
        for item in axes:
            amplitude = float(amplitudes[item])
            positions[item][index] = amplitude * envelope * raw
            velocities[item][index] = amplitude * (envelope_dot * raw + envelope * raw_dot)
    return positions, velocities


def make_random_components(profile, rng):
    band = profile['frequency_band']
    min_hz = float(band['min'])
    max_hz = float(band['max'])
    if min_hz <= 0.0 or max_hz < min_hz:
        raise ValueError('frequency_band must satisfy 0 < min <= max')
    component_count = int(profile.get('component_count', 6))
    if component_count <= 0:
        raise ValueError('component_count must be positive')
    components = {}
    for axis in AXES:
        axis_components = []
        for _ in range(component_count):
            axis_components.append({
                'frequency_hz': rng.uniform(min_hz, max_hz),
                'phase': rng.uniform(0.0, 2.0 * math.pi),
                'weight': rng.uniform(0.4, 1.0) * (-1.0 if rng.random() < 0.5 else 1.0),
            })
        components[axis] = axis_components
    return components


def raw_random_value(components, t):
    value = 0.0
    velocity = 0.0
    for component in components:
        frequency_hz = component['frequency_hz']
        omega = 2.0 * math.pi * frequency_hz
        phase = component['phase']
        weight = component['weight']
        value += weight * math.sin(omega * t + phase)
        velocity += weight * omega * math.cos(omega * t + phase)
    return value, velocity


def generate_random_translation_3d(times, profile):
    duration_sec = float(profile['duration_sec'])
    ramp_sec = float(profile.get('ramp_sec', min(2.0, duration_sec * 0.1)))
    rng = random.Random(int(profile.get('random_seed', 42)))
    components_by_axis = make_random_components(profile, rng)
    amplitudes = profile['translation_amplitude']
    positions = {axis: [] for axis in AXES}
    velocities = {axis: [] for axis in AXES}

    raw_positions = {axis: [] for axis in AXES}
    raw_velocities = {axis: [] for axis in AXES}
    for axis in AXES:
        components = components_by_axis[axis]
        raw_at_zero, _ = raw_random_value(components, 0.0)
        for t in times:
            value, velocity = raw_random_value(components, t)
            raw_positions[axis].append(value - raw_at_zero)
            raw_velocities[axis].append(velocity)

    scales = {}
    for axis in AXES:
        max_abs = max(abs(value) for value in raw_positions[axis])
        scales[axis] = float(amplitudes[axis]) / max_abs if max_abs > 1e-12 else 0.0

    for index, t in enumerate(times):
        envelope, envelope_dot = smooth_envelope(t, duration_sec, ramp_sec)
        for axis in AXES:
            raw = raw_positions[axis][index]
            raw_dot = raw_velocities[axis][index]
            scale = scales[axis]
            positions[axis].append(scale * envelope * raw)
            velocities[axis].append(scale * (envelope_dot * raw + envelope * raw_dot))

    max_translation_norm = profile.get('max_translation_norm')
    if max_translation_norm is not None:
        max_translation_norm = float(max_translation_norm)
        if max_translation_norm <= 0.0:
            raise ValueError('max_translation_norm must be positive')
        max_norm = max(
            math.sqrt(sum(positions[axis][index] ** 2 for axis in AXES))
            for index in range(len(times)))
        if max_norm > max_translation_norm:
            scale = max_translation_norm / max_norm
            for axis in AXES:
                positions[axis] = [value * scale for value in positions[axis]]
                velocities[axis] = [value * scale for value in velocities[axis]]
    return positions, velocities


def generate_rows(profile):
    times = time_samples(float(profile['duration_sec']), float(profile['sample_rate_hz']))
    profile_type = str(profile.get('type', 'static'))
    if profile_type == 'static':
        positions, velocities = generate_static(times)
    elif profile_type == 'sine':
        positions, velocities = generate_sine(times, profile)
    elif profile_type == 'random_translation_3d':
        positions, velocities = generate_random_translation_3d(times, profile)
    else:
        raise ValueError(f'Unsupported profile type: {profile_type}')

    rows = []
    for index, t in enumerate(times):
        rows.append({
            'time_sec': t,
            'x': positions['x'][index],
            'y': positions['y'][index],
            'z': positions['z'][index],
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0,
            'linear_x': velocities['x'][index],
            'linear_y': velocities['y'][index],
            'linear_z': velocities['z'][index],
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
        })
    return rows


def write_csv(path, rows):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: f'{float(row[key]):.9f}'
                for key in CSV_FIELDS
            })


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Generate repeatable base disturbance trajectories for Gazebo replay.')
    parser.add_argument('--config', required=True, help='base_disturbance_profiles.yaml path')
    parser.add_argument('--profile', required=True, help='Profile name to generate')
    parser.add_argument('--output-csv', required=True, help='Output trajectory CSV path')
    parser.add_argument('--duration-sec', type=float, help='Override profile duration')
    parser.add_argument('--sample-rate-hz', type=float, help='Override profile sample rate')
    parser.add_argument('--random-seed', type=int, help='Override profile random seed')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    profile = load_profile(args.config, args.profile)
    if args.duration_sec is not None:
        profile['duration_sec'] = args.duration_sec
    if args.sample_rate_hz is not None:
        profile['sample_rate_hz'] = args.sample_rate_hz
    if args.random_seed is not None:
        profile['random_seed'] = args.random_seed

    rows = generate_rows(profile)
    write_csv(args.output_csv, rows)
    print(
        f'wrote {len(rows)} samples for profile {args.profile} '
        f'to {args.output_csv}')


if __name__ == '__main__':
    main()
