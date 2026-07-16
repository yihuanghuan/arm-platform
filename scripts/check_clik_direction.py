#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time

from gazebo_msgs.msg import LinkStates
import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def default_urdf_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(
            os.path.dirname(os.path.dirname(script_dir)),
            'share',
            'manipulator',
            'arm.urdf'),
        os.path.join(os.path.dirname(script_dir), 'config', 'arm.urdf'),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


DEFAULT_URDF = default_urdf_path()


def finite_vector(values):
    return np.all(np.isfinite(values))


def pose_xyz(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)


def serialize(values):
    return ';'.join(f'{float(value):.9g}' for value in values)


class ClikDirectionCheck(Node):
    def __init__(self, args):
        super().__init__(
            'check_clik_direction',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.model = pin.buildModelFromUrdf(args.urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId(args.ee_frame)
        self.q = pin.neutral(self.model)
        self.q_indices = []
        self.v_indices = []
        for name in self.joint_names:
            joint_id = self.model.getJointId(name)
            joint_model = self.model.joints[joint_id]
            self.q_indices.append(joint_model.idx_q)
            self.v_indices.append(joint_model.idx_v)

        self.joint_ready = False
        self.gt_position = None
        self.visual_position = None
        self.command_pub = self.create_publisher(
            Float64MultiArray, args.command_topic, 10)
        self.create_subscription(JointState, args.joint_states_topic,
                                 self.joint_state_callback, 20)
        self.create_subscription(LinkStates, args.link_states_topic,
                                 self.link_states_callback, 20)
        self.create_subscription(PoseStamped, args.visual_pose_topic,
                                 self.visual_pose_callback, 20)

    def joint_state_callback(self, msg):
        for name, q_index in zip(self.joint_names, self.q_indices):
            try:
                index = msg.name.index(name)
            except ValueError:
                return
            if index >= len(msg.position):
                return
            value = float(msg.position[index])
            if not math.isfinite(value):
                return
            self.q[q_index] = value
        self.joint_ready = True

    def link_states_callback(self, msg):
        try:
            index = msg.name.index(self.args.ee_link_name)
        except ValueError:
            return
        if index < len(msg.pose):
            position = pose_xyz(msg.pose[index])
            if finite_vector(position):
                self.gt_position = position

    def visual_pose_callback(self, msg):
        position = pose_xyz(msg.pose)
        if finite_vector(position):
            self.visual_position = position

    def current_source_position(self):
        if self.args.source == 'ground_truth':
            return self.gt_position
        return self.visual_position

    def wait_ready(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.joint_ready and self.current_source_position() is not None:
                return True
        return False

    def compute_dq(self, desired_delta):
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        jacobian_full = pin.computeFrameJacobian(
            self.model, self.data, self.q, self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED)
        jacobian = np.zeros((3, 6), dtype=float)
        for out_col, v_index in enumerate(self.v_indices):
            jacobian[:, out_col] = jacobian_full[:3, v_index]
        task_velocity = desired_delta / max(self.args.pulse_duration_sec, 1e-6)
        lhs = jacobian @ jacobian.T + (
            self.args.damping * self.args.damping * np.eye(3, dtype=float))
        dq = jacobian.T @ np.linalg.solve(lhs, task_velocity)
        dq = np.clip(dq, -self.args.max_joint_velocity, self.args.max_joint_velocity)
        predicted_velocity = jacobian @ dq
        return dq, predicted_velocity

    def publish_command(self, values):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        self.command_pub.publish(msg)

    def run(self):
        if not self.wait_ready(self.args.ready_timeout_sec):
            raise RuntimeError('Timed out waiting for joint states and pose source')
        output_dir = os.path.dirname(os.path.abspath(self.args.output_csv))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        axes = [
            ('+x', np.array([self.args.axis_step_m, 0.0, 0.0], dtype=float), 0),
            ('-x', np.array([-self.args.axis_step_m, 0.0, 0.0], dtype=float), 0),
            ('+y', np.array([0.0, self.args.axis_step_m, 0.0], dtype=float), 1),
            ('-y', np.array([0.0, -self.args.axis_step_m, 0.0], dtype=float), 1),
            ('+z', np.array([0.0, 0.0, self.args.axis_step_m], dtype=float), 2),
            ('-z', np.array([0.0, 0.0, -self.args.axis_step_m], dtype=float), 2),
        ]
        fieldnames = [
            'axis', 'desired_delta', 'dq', 'predicted_dx', 'predicted_dy',
            'predicted_dz', 'gt_dx', 'gt_dy', 'gt_dz', 'visual_dx',
            'visual_dy', 'visual_dz', 'sign_pass', 'axis_alignment_pass']
        with open(self.args.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for axis_name, desired_delta, axis_index in axes:
                if not self.wait_ready(self.args.ready_timeout_sec):
                    raise RuntimeError('Pose source became unavailable')
                before_gt = None if self.gt_position is None else self.gt_position.copy()
                before_visual = (
                    None if self.visual_position is None else self.visual_position.copy())
                dq, predicted_velocity = self.compute_dq(desired_delta)
                self.publish_command(dq)
                end_time = time.monotonic() + self.args.pulse_duration_sec
                while time.monotonic() < end_time and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.01)
                    self.publish_command(dq)
                self.publish_command(np.zeros(6, dtype=float))
                settle_until = time.monotonic() + self.args.settle_sec
                while time.monotonic() < settle_until and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.02)
                after_gt = None if self.gt_position is None else self.gt_position.copy()
                after_visual = (
                    None if self.visual_position is None else self.visual_position.copy())
                gt_delta = (
                    np.zeros(3, dtype=float)
                    if before_gt is None or after_gt is None else after_gt - before_gt)
                visual_delta = (
                    np.zeros(3, dtype=float)
                    if before_visual is None or after_visual is None
                    else after_visual - before_visual)
                measured = gt_delta if self.args.source == 'ground_truth' else visual_delta
                wanted_sign = 1.0 if desired_delta[axis_index] > 0.0 else -1.0
                sign_pass = wanted_sign * measured[axis_index] > 0.0
                other = np.delete(np.abs(measured), axis_index)
                axis_alignment_pass = abs(measured[axis_index]) >= 2.0 * max(other.max(), 1e-9)
                writer.writerow({
                    'axis': axis_name,
                    'desired_delta': serialize(desired_delta),
                    'dq': serialize(dq),
                    'predicted_dx': f'{predicted_velocity[0]:.9g}',
                    'predicted_dy': f'{predicted_velocity[1]:.9g}',
                    'predicted_dz': f'{predicted_velocity[2]:.9g}',
                    'gt_dx': f'{gt_delta[0]:.9g}',
                    'gt_dy': f'{gt_delta[1]:.9g}',
                    'gt_dz': f'{gt_delta[2]:.9g}',
                    'visual_dx': f'{visual_delta[0]:.9g}',
                    'visual_dy': f'{visual_delta[1]:.9g}',
                    'visual_dz': f'{visual_delta[2]:.9g}',
                    'sign_pass': str(bool(sign_pass)).lower(),
                    'axis_alignment_pass': str(bool(axis_alignment_pass)).lower(),
                })
                handle.flush()
        self.publish_command(np.zeros(6, dtype=float))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Small-pulse CLIK direction check for phase4.2')
    parser.add_argument('--source', choices=('ground_truth', 'visual'),
                        default='ground_truth')
    parser.add_argument('--output-csv', default='/tmp/phase4_2_clik_direction.csv')
    parser.add_argument('--urdf-path', default=DEFAULT_URDF)
    parser.add_argument('--ee-frame', default='link6')
    parser.add_argument('--ee-link-name', default='windylab_arm::link6')
    parser.add_argument('--joint-states-topic', default='/joint_states')
    parser.add_argument('--link-states-topic', default='/link_states')
    parser.add_argument('--visual-pose-topic', default='/visual_ee_pose')
    parser.add_argument('--command-topic', default='/arm_velocity_controller/commands')
    parser.add_argument('--axis-step-m', type=float, default=0.005)
    parser.add_argument('--pulse-duration-sec', type=float, default=0.08)
    parser.add_argument('--settle-sec', type=float, default=0.35)
    parser.add_argument('--max-joint-velocity', type=float, default=0.03)
    parser.add_argument('--damping', type=float, default=0.05)
    parser.add_argument('--ready-timeout-sec', type=float, default=8.0)
    parser.add_argument('--use-sim-time', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = ClikDirectionCheck(args)
    try:
        node.run()
    finally:
        try:
            node.publish_command(np.zeros(6, dtype=float))
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
