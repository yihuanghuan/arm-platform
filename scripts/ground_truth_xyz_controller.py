#!/usr/bin/env python3

import json
import math
import os
import sys

from gazebo_msgs.msg import LinkStates
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


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


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def finite_vector(values):
    return np.all(np.isfinite(values))


class GroundTruthXyzController(Node):
    def __init__(self):
        super().__init__('ground_truth_xyz_controller')
        self.urdf_path = self.declare_parameter('urdf_path', DEFAULT_URDF).value
        self.ee_frame = self.declare_parameter('ee_frame', 'link6').value
        self.ee_link_name = self.declare_parameter(
            'ee_link_name', 'windylab_arm::link6').value
        self.joint_names = list(self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']).value)
        self.link_states_topic = self.declare_parameter(
            'link_states_topic', '/link_states').value
        self.joint_states_topic = self.declare_parameter(
            'joint_states_topic', '/joint_states').value
        self.command_topic = self.declare_parameter(
            'command_topic', '/arm_velocity_controller/commands').value
        self.control_rate = float(self.declare_parameter('control_rate', 100.0).value)
        self.measurement_timeout_sec = float(self.declare_parameter(
            'measurement_timeout_sec', 0.30).value)
        self.joint_state_timeout_sec = float(self.declare_parameter(
            'joint_state_timeout_sec', 0.35).value)
        self.damping = float(self.declare_parameter('damping', 0.05).value)
        self.max_joint_velocity = float(self.declare_parameter(
            'max_joint_velocity', 0.2).value)
        self.max_joint_acceleration = float(self.declare_parameter(
            'max_joint_acceleration_rad_s2', 0.3).value)
        self.max_visual_error_norm_m = float(self.declare_parameter(
            'max_visual_error_norm_m', 0.20).value)
        self.stop_on_large_error = bool(self.declare_parameter(
            'stop_on_large_error', True).value)
        self.position_deadband_m = float(self.declare_parameter(
            'position_deadband_m', 0.003).value)
        self.dry_run = bool(self.declare_parameter('dry_run', True).value)
        self.task_gain = self.vector_parameter(
            'task_gain', [4.0, 4.0, 4.0], 3)
        self.max_task_velocity_xyz = self.vector_parameter(
            'max_task_velocity_xyz', '0.05 0.05 0.05', 3)

        if len(self.joint_names) != 6:
            raise ValueError('joint_names must contain exactly 6 names')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')
        if self.max_joint_velocity < 0.0:
            raise ValueError('max_joint_velocity must be non-negative')
        if self.max_joint_acceleration < 0.0:
            raise ValueError('max_joint_acceleration_rad_s2 must be non-negative')

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        if not self.model.existFrame(self.ee_frame):
            raise ValueError(f'End-effector frame not found: {self.ee_frame}')
        self.ee_frame_id = self.model.getFrameId(self.ee_frame)

        self.q_indices = []
        self.v_indices = []
        for name in self.joint_names:
            if not self.model.existJointName(name):
                raise ValueError(f'Joint not found in URDF: {name}')
            joint_id = self.model.getJointId(name)
            joint_model = self.model.joints[joint_id]
            self.q_indices.append(joint_model.idx_q)
            self.v_indices.append(joint_model.idx_v)
        self.lower_limits = np.array(
            [self.model.lowerPositionLimit[index] for index in self.q_indices],
            dtype=float)
        self.upper_limits = np.array(
            [self.model.upperPositionLimit[index] for index in self.q_indices],
            dtype=float)

        self.q = pin.neutral(self.model)
        self.joint_state_ready = False
        self.last_joint_state_ns = 0
        self.latest_position = None
        self.latest_position_ns = 0
        self.latest_pose_seq = 0
        self.latest_processed_pose_seq = 0
        self.target_position = None
        self.target_lock_count = 0
        self.dq_target = np.zeros(6, dtype=float)
        self.dq_command = np.zeros(6, dtype=float)
        self.last_error = np.zeros(6, dtype=float)
        self.last_dq_raw = np.zeros(6, dtype=float)
        self.last_control_ns = 0
        self.safety_stop = False
        self.safety_stop_reason = ''

        self.create_subscription(JointState, self.joint_states_topic,
                                 self.joint_state_callback, 20)
        self.create_subscription(LinkStates, self.link_states_topic,
                                 self.link_states_callback, 20)
        self.error_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/error', 10)
        self.dq_raw_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/dq_raw', 10)
        self.dq_limited_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/dq_limited', 10)
        self.dq_target_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/dq_target', 10)
        self.target_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/target_pose', 10)
        self.status_pub = self.create_publisher(
            String, '/visual_stabilization/status', 10)
        self.command_pub = None
        if not self.dry_run:
            self.command_pub = self.create_publisher(
                Float64MultiArray, self.command_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.get_logger().info(
            f'Ground-truth XYZ controller ready: link={self.ee_link_name}, '
            f'dry_run={self.dry_run}')

    def vector_parameter(self, name, default, expected_length):
        raw_value = self.declare_parameter(name, default).value
        if isinstance(raw_value, str):
            values = [float(part) for part in raw_value.replace(',', ' ').split()]
        else:
            values = [float(value) for value in raw_value]
        if len(values) != expected_length:
            raise ValueError(f'{name} must contain {expected_length} values')
        return np.array(values, dtype=float)

    def joint_state_callback(self, msg):
        for name, q_index in zip(self.joint_names, self.q_indices):
            try:
                msg_index = msg.name.index(name)
            except ValueError:
                return
            if msg_index >= len(msg.position):
                return
            value = float(msg.position[msg_index])
            if not math.isfinite(value):
                return
            self.q[q_index] = value
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.last_joint_state_ns = (
            stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds)
        self.joint_state_ready = True

    def link_states_callback(self, msg):
        try:
            index = msg.name.index(self.ee_link_name)
        except ValueError:
            return
        if index >= len(msg.pose):
            return
        pose = msg.pose[index]
        position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ], dtype=float)
        if not finite_vector(position):
            return
        self.latest_position = position
        self.latest_position_ns = self.get_clock().now().nanoseconds
        self.latest_pose_seq += 1

    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = self.compute_dt(now_ns)
        reason = 'ok'
        if self.safety_stop:
            self.dq_target = np.zeros(6, dtype=float)
            reason = self.safety_stop_reason
        else:
            ready, reason = self.inputs_ready(now_ns)
            if ready and self.latest_pose_seq != self.latest_processed_pose_seq:
                self.latest_processed_pose_seq = self.latest_pose_seq
                reason = self.update_target_command()
            elif not ready:
                self.dq_target = np.zeros(6, dtype=float)
        self.dq_command = self.ramp(self.dq_command, self.dq_target, dt)
        if self.command_would_push_joint_limit(self.dq_command):
            self.dq_command = np.zeros(6, dtype=float)
            self.dq_target = np.zeros(6, dtype=float)
            reason = 'joint_limit_guard'
        self.publish_outputs(reason)
        self.publish_command()

    def compute_dt(self, now_ns):
        if self.last_control_ns <= 0 or now_ns <= self.last_control_ns:
            dt = 1.0 / self.control_rate
        else:
            dt = (now_ns - self.last_control_ns) * 1e-9
        self.last_control_ns = now_ns
        return max(0.0, min(dt, 0.1))

    def inputs_ready(self, now_ns):
        if not self.joint_state_ready:
            return False, 'waiting_for_joint_states'
        if now_ns - self.last_joint_state_ns > int(self.joint_state_timeout_sec * 1e9):
            return False, 'joint_state_timeout'
        if self.latest_position is None:
            return False, 'waiting_for_link_states'
        if now_ns - self.latest_position_ns > int(self.measurement_timeout_sec * 1e9):
            return False, 'link_state_timeout'
        return True, 'ok'

    def update_target_command(self):
        if self.target_position is None:
            self.target_position = np.array(self.latest_position, dtype=float)
            self.target_lock_count += 1
            self.get_logger().info(
                'Locked GT EE target at '
                f'[{self.target_position[0]:.4f}, '
                f'{self.target_position[1]:.4f}, {self.target_position[2]:.4f}]')
        error_xyz = self.target_position - self.latest_position
        self.last_error = np.zeros(6, dtype=float)
        self.last_error[:3] = error_xyz
        error_norm = float(np.linalg.norm(error_xyz))
        if self.max_visual_error_norm_m > 0.0 and error_norm > self.max_visual_error_norm_m:
            reason = f'gt_error_norm_{error_norm:.4f}'
            if self.stop_on_large_error:
                self.safety_stop = True
                self.safety_stop_reason = reason
            self.last_dq_raw = np.zeros(6, dtype=float)
            self.dq_target = np.zeros(6, dtype=float)
            return reason

        task_velocity = self.task_gain * error_xyz
        if error_norm <= self.position_deadband_m:
            task_velocity = np.zeros(3, dtype=float)
            self.last_error[:3] = 0.0
        task_velocity = np.clip(
            task_velocity, -self.max_task_velocity_xyz, self.max_task_velocity_xyz)
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        jacobian_full = pin.computeFrameJacobian(
            self.model, self.data, self.q, self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED)
        jacobian = np.zeros((3, 6), dtype=float)
        for out_col, v_index in enumerate(self.v_indices):
            jacobian[:, out_col] = jacobian_full[:3, v_index]
        lhs = jacobian @ jacobian.T + (
            self.damping * self.damping * np.eye(3, dtype=float))
        dq_raw = jacobian.T @ np.linalg.solve(lhs, task_velocity)
        dq_limited = np.clip(dq_raw, -self.max_joint_velocity, self.max_joint_velocity)
        if not finite_vector(dq_raw) or not finite_vector(dq_limited):
            self.last_dq_raw = np.zeros(6, dtype=float)
            self.dq_target = np.zeros(6, dtype=float)
            return 'non_finite_gt_command'
        self.last_dq_raw = dq_raw
        self.dq_target = dq_limited
        return 'ok'

    def ramp(self, current, target, dt):
        if self.max_joint_acceleration <= 0.0:
            return np.array(target, dtype=float)
        max_step = self.max_joint_acceleration * dt
        return current + np.clip(target - current, -max_step, max_step)

    def command_would_push_joint_limit(self, dq):
        margin = 0.05
        for position, velocity, lower, upper in zip(
                self.q[self.q_indices], dq, self.lower_limits, self.upper_limits):
            if math.isfinite(lower) and position <= lower + margin and velocity < 0.0:
                return True
            if math.isfinite(upper) and position >= upper - margin and velocity > 0.0:
                return True
        return False

    def publish_outputs(self, reason):
        self.error_pub.publish(self.make_array(self.last_error))
        self.dq_raw_pub.publish(self.make_array(self.last_dq_raw))
        self.dq_limited_pub.publish(self.make_array(self.dq_command))
        self.dq_target_pub.publish(self.make_array(self.dq_target))
        target = [0.0] * 7
        if self.target_position is not None:
            target[:3] = [float(value) for value in self.target_position]
            target[6] = 1.0
        self.target_pub.publish(self.make_array(target))
        payload = {
            'status': reason,
            'source': 'gazebo_link_states',
            'target_lock_count': self.target_lock_count,
            'target_position': (
                None if self.target_position is None
                else [float(value) for value in self.target_position]),
            'safety_stop': self.safety_stop,
            'safety_stop_reason': self.safety_stop_reason,
            'dq_target': [float(value) for value in self.dq_target],
            'dq_command': [float(value) for value in self.dq_command],
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_command(self):
        if self.command_pub is not None:
            self.command_pub.publish(self.make_array(self.dq_command))

    @staticmethod
    def make_array(values):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        return msg


def main(argv=None):
    rclpy.init(args=argv)
    node = GroundTruthXyzController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception as exc:
        if 'context is not valid' not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
