#!/usr/bin/env python3

import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def make_transform(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def rotation_error(a_matrix, b_matrix):
    relative = a_matrix[:3, :3].T @ b_matrix[:3, :3]
    return float(Rotation.from_matrix(relative).magnitude())


def translation_error(a_matrix, b_matrix):
    return float(np.linalg.norm(a_matrix[:3, 3] - b_matrix[:3, 3]))


def recover_world_to_ee(world_to_tag, camera_to_tag, ee_to_camera):
    return world_to_tag @ np.linalg.inv(camera_to_tag) @ np.linalg.inv(ee_to_camera)


def run_case(name, world_to_tag, world_to_ee, ee_to_camera):
    world_to_camera = world_to_ee @ ee_to_camera
    camera_to_tag = np.linalg.inv(world_to_camera) @ world_to_tag
    recovered = recover_world_to_ee(world_to_tag, camera_to_tag, ee_to_camera)
    trans_err = translation_error(world_to_ee, recovered)
    rot_err = rotation_error(world_to_ee, recovered)
    passed = trans_err < 1e-9 and rot_err < 1e-9
    print(
        f'{name}: pass={str(passed).lower()} '
        f'translation_error={trans_err:.3e} rotation_error={rot_err:.3e}')
    return passed


def main():
    world_to_tag = make_transform(
        [1.6, 0.0, 0.35],
        [0.0, -math.pi / 2.0, 0.0])
    optical_rotation = make_transform(
        [0.071, 0.033, 0.053],
        [-math.pi / 2.0, 0.0, -math.pi / 2.0])
    cases = [
        (
            'pure_translation',
            world_to_tag,
            make_transform([0.35, 0.0, 0.37], [0.0, 0.0, 0.0]),
            make_transform([0.07, 0.03, 0.05], [0.0, 0.0, 0.0]),
        ),
        (
            'optical_frame_rotation',
            world_to_tag,
            make_transform([0.35, 0.0, 0.37], [0.0, 0.0, 0.0]),
            optical_rotation,
        ),
        (
            'ee_rotation',
            world_to_tag,
            make_transform([0.35, 0.0, 0.37], [0.2, -0.1, 0.3]),
            optical_rotation,
        ),
        (
            'base_translation',
            world_to_tag,
            make_transform([0.45, -0.03, 0.39], [0.0, 0.0, 0.0]),
            optical_rotation,
        ),
        (
            'base_translation_rotation',
            world_to_tag,
            make_transform([0.45, -0.03, 0.39], [0.1, -0.2, 0.15]),
            optical_rotation,
        ),
    ]
    if not all(run_case(*case) for case in cases):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
