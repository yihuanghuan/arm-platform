#!/usr/bin/env python3

import math
import os
import sys

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config',
    'arm.urdf')


def pose_to_se3(pose):
    rotation = Rotation.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]).as_matrix()
    translation = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ], dtype=float)
    return pin.SE3(rotation, translation)


def se3_to_xyz_quat(transform):
    quat = Rotation.from_matrix(transform.rotation).as_quat()
    return [
        float(transform.translation[0]),
        float(transform.translation[1]),
        float(transform.translation[2]),
        float(quat[0]),
        float(quat[1]),
        float(quat[2]),
        float(quat[3]),
    ]


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def finite_vector(values):
    return np.all(np.isfinite(values))


class VisualEeStabilizationController(Node):
    def __init__(self):
        super().__init__('visual_ee_stabilization_controller')

        self.urdf_path = self.declare_parameter('urdf_path', DEFAULT_URDF).value
        self.ee_frame = self.declare_parameter('ee_frame', 'link6').value
        self.joint_names = list(self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']).value)
        self.control_rate = float(self.declare_parameter('control_rate', 100.0).value)
        self.visual_pose_topic = self.declare_parameter(
            'visual_pose_topic', '/visual_ee_pose').value
        self.visual_valid_topic = self.declare_parameter(
            'visual_valid_topic', '/visual_ee_pose_valid').value
        self.joint_states_topic = self.declare_parameter(
            'joint_states_topic', '/joint_states').value
        self.measurement_timeout_sec = float(self.declare_parameter(
            'measurement_timeout_sec', 0.35).value)
        self.joint_state_timeout_sec = float(self.declare_parameter(
            'joint_state_timeout_sec', 0.35).value)
        self.damping = float(self.declare_parameter('damping', 0.05).value)
        self.max_joint_velocity = float(self.declare_parameter(
            'max_joint_velocity', 1.0).value)
        self.dry_run = bool(self.declare_parameter('dry_run', True).value)
        self.control_mode = self.declare_parameter('control_mode', 'xyz').value
        self.command_topic = self.declare_parameter(
            'command_topic', '/arm_velocity_controller/commands').value
        self.command_start_delay_sec = float(self.declare_parameter(
            'command_start_delay_sec', 0.0).value)
        self.position_deadband_m = float(self.declare_parameter(
            'position_deadband_m', 0.003).value)
        self.joint_limit_margin_rad = float(self.declare_parameter(
            'joint_limit_margin_rad', 0.05).value)
        self.task_gain = self._vector_parameter(
            'task_gain',
            [4.0, 4.0, 4.0, 2.0, 2.0, 2.0],
            6)
        self.max_task_velocity_xyz = self._vector_parameter(
            'max_task_velocity_xyz',
            '0.08 0.08 0.08',
            3)
        self.max_task_velocity = self._vector_parameter(
            'max_task_velocity',
            [0.5, 0.5, 0.5, 1.0, 1.0, 1.0],
            6)
        self.use_visual_valid_topic = bool(self.declare_parameter(
            'use_visual_valid_topic', True).value)
        self.required_consecutive_valid_poses = int(self.declare_parameter(
            'required_consecutive_valid_poses', 1).value)
        self.relock_after_visual_loss_sec = float(self.declare_parameter(
            'relock_after_visual_loss_sec', 0.0).value)
        self.max_visual_error_norm_m = float(self.declare_parameter(
            'max_visual_error_norm_m', 0.0).value)

        if len(self.joint_names) != 6:
            raise ValueError('joint_names must contain exactly 6 names')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')
        if self.damping < 0.0:
            raise ValueError('damping must be non-negative')
        if self.max_joint_velocity < 0.0:
            raise ValueError('max_joint_velocity must be non-negative')
        if self.control_mode not in ('xyz', 'se3_debug'):
            raise ValueError('control_mode must be xyz or se3_debug')
        if self.position_deadband_m < 0.0:
            raise ValueError('position_deadband_m must be non-negative')
        if self.joint_limit_margin_rad < 0.0:
            raise ValueError('joint_limit_margin_rad must be non-negative')
        if self.command_start_delay_sec < 0.0:
            raise ValueError('command_start_delay_sec must be non-negative')
        if self.required_consecutive_valid_poses <= 0:
            raise ValueError('required_consecutive_valid_poses must be positive')
        if self.relock_after_visual_loss_sec < 0.0:
            raise ValueError('relock_after_visual_loss_sec must be non-negative')
        if self.max_visual_error_norm_m < 0.0:
            raise ValueError('max_visual_error_norm_m must be non-negative')

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
            if joint_model.nq != 1 or joint_model.nv != 1:
                raise ValueError(f'Only 1-DoF joints are supported: {name}')
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
        self.latest_visual_pose = None
        self.latest_visual_stamp_ns = 0
        self.counted_visual_stamp_ns = 0
        self.consecutive_valid_poses = 0
        self.visual_invalid_since_ns = 0
        self.visual_valid = False
        self.target_pose = None
        self.target_lock_ns = 0
        self.consecutive_valid_poses = 0
        self.counted_visual_stamp_ns = self.latest_visual_stamp_ns
        self.last_status = 'waiting_for_inputs'

        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10)
        self.create_subscription(
            PoseStamped,
            self.visual_pose_topic,
            self.visual_pose_callback,
            10)
        if self.use_visual_valid_topic:
            self.create_subscription(
                Bool,
                self.visual_valid_topic,
                self.visual_valid_callback,
                10)

        self.error_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/error', 10)
        self.dq_raw_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/dq_raw', 10)
        self.dq_limited_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/dq_limited', 10)
        self.target_pose_pub = self.create_publisher(
            Float64MultiArray, '/visual_stabilization/target_pose', 10)
        self.command_pub = None
        if not self.dry_run:
            self.command_pub = self.create_publisher(
                Float64MultiArray, self.command_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            'Visual CLIK controller ready: '
            f'urdf={self.urdf_path}, ee_frame={self.ee_frame}, '
            f'rate={self.control_rate:.1f} Hz, mode={self.control_mode}, '
            f'dry_run={self.dry_run}')

    def _vector_parameter(self, name, default, expected_length):
        raw_value = self.declare_parameter(name, default).value
        if isinstance(raw_value, str):
            value = [float(part) for part in raw_value.replace(',', ' ').split()]
        else:
            value = [float(item) for item in raw_value]
        if len(value) != expected_length:
            raise ValueError(f'{name} must contain {expected_length} values')
        return np.array(value, dtype=float)

    def joint_state_callback(self, msg):
        missing = []
        for name, q_index in zip(self.joint_names, self.q_indices):
            try:
                msg_index = msg.name.index(name)
            except ValueError:
                missing.append(name)
                continue
            if msg_index >= len(msg.position):
                missing.append(name)
                continue
            value = float(msg.position[msg_index])
            if not math.isfinite(value):
                self.last_status = 'non_finite_joint_state'
                return
            self.q[q_index] = value

        if missing:
            self.last_status = 'joint_state_missing_' + ','.join(missing)
            return

        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.last_joint_state_ns = (
            stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds)
        self.joint_state_ready = True

    def visual_pose_callback(self, msg):
        try:
            pose = pose_to_se3(msg.pose)
        except Exception as exc:
            self.last_status = f'visual_pose_parse_failed: {exc}'
            return
        pose_vector = np.array(se3_to_xyz_quat(pose), dtype=float)
        if not finite_vector(pose_vector):
            self.last_status = 'non_finite_visual_pose'
            return

        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.latest_visual_stamp_ns = (
            stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds)
        self.latest_visual_pose = pose
        if not self.use_visual_valid_topic:
            self.visual_valid = True

    def visual_valid_callback(self, msg):
        self.visual_valid = bool(msg.data)
        if self.visual_valid:
            self.visual_invalid_since_ns = 0
        else:
            self.consecutive_valid_poses = 0
            self.counted_visual_stamp_ns = 0
            if self.visual_invalid_since_ns <= 0:
                self.visual_invalid_since_ns = self.get_clock().now().nanoseconds

    def record_valid_visual_pose(self, stamp_ns):
        if stamp_ns <= 0 or stamp_ns == self.counted_visual_stamp_ns:
            return
        self.counted_visual_stamp_ns = stamp_ns
        self.consecutive_valid_poses += 1
        self.visual_invalid_since_ns = 0

    def reset_target(self, reason):
        if self.target_pose is not None:
            self.get_logger().info(
                f'Resetting visual EE target: {reason}',
                throttle_duration_sec=1.0)
        self.target_pose = None
        self.target_lock_ns = 0

    def control_loop(self):
        error = np.zeros(6, dtype=float)
        dq_raw = np.zeros(6, dtype=float)
        dq_limited = np.zeros(6, dtype=float)

        ready, reason = self.inputs_ready()
        if ready:
            self.record_valid_visual_pose(self.latest_visual_stamp_ns)
            if self.target_pose is None:
                if self.consecutive_valid_poses < self.required_consecutive_valid_poses:
                    reason = (
                        'warming_visual_pose_'
                        f'{self.consecutive_valid_poses}/'
                        f'{self.required_consecutive_valid_poses}')
                    self.last_status = reason
                    self.publish_outputs(error, dq_raw, dq_limited)
                    self.publish_command(dq_limited)
                    return
                self.target_pose = self.latest_visual_pose.copy()
                self.target_lock_ns = self.get_clock().now().nanoseconds
                self.get_logger().info(
                    'Locked visual EE target at '
                    f'[{self.target_pose.translation[0]:.4f}, '
                    f'{self.target_pose.translation[1]:.4f}, '
                    f'{self.target_pose.translation[2]:.4f}]')

            try:
                error, dq_raw, dq_limited = self.compute_command()
                reason = 'ok'
            except Exception as exc:
                reason = f'compute_failed: {exc}'
                error = np.zeros(6, dtype=float)
                dq_raw = np.zeros(6, dtype=float)
                dq_limited = np.zeros(6, dtype=float)
        else:
            self.handle_not_ready(reason)

        self.last_status = reason
        self.publish_outputs(error, dq_raw, dq_limited)
        self.publish_command(dq_limited)

    def inputs_ready(self):
        now_ns = self.get_clock().now().nanoseconds
        if not self.joint_state_ready:
            return False, 'waiting_for_joint_states'
        if now_ns - self.last_joint_state_ns > int(self.joint_state_timeout_sec * 1e9):
            return False, 'joint_state_timeout'
        if self.latest_visual_pose is None:
            return False, 'waiting_for_visual_pose'
        if self.use_visual_valid_topic and not self.visual_valid:
            return False, 'visual_pose_invalid'
        if now_ns - self.latest_visual_stamp_ns > int(self.measurement_timeout_sec * 1e9):
            return False, 'visual_pose_timeout'
        return True, 'ok'

    def handle_not_ready(self, reason):
        if not reason.startswith('visual_pose') and reason != 'waiting_for_visual_pose':
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.visual_invalid_since_ns <= 0:
            self.visual_invalid_since_ns = now_ns
        if self.relock_after_visual_loss_sec <= 0.0:
            return
        elapsed_sec = (now_ns - self.visual_invalid_since_ns) * 1e-9
        if elapsed_sec >= self.relock_after_visual_loss_sec:
            self.reset_target(reason)

    def compute_command(self):
        if self.control_mode == 'xyz':
            return self.compute_xyz_command()
        return self.compute_se3_debug_command()

    def compute_xyz_command(self):
        position_error = (
            self.target_pose.translation - self.latest_visual_pose.translation)
        error = np.zeros(6, dtype=float)
        error[:3] = position_error
        if not finite_vector(error):
            raise ValueError('non-finite XYZ error')

        error_norm = np.linalg.norm(position_error)
        if self.max_visual_error_norm_m > 0.0 and error_norm > self.max_visual_error_norm_m:
            self.reset_target(f'visual_error_norm_{error_norm:.4f}')
            return error, np.zeros(6, dtype=float), np.zeros(6, dtype=float)

        task_velocity = self.task_gain[:3] * position_error
        if error_norm <= self.position_deadband_m:
            task_velocity = np.zeros(3, dtype=float)
            error[:3] = np.zeros(3, dtype=float)
        task_velocity = np.clip(
            task_velocity,
            -self.max_task_velocity_xyz,
            self.max_task_velocity_xyz)

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        jacobian_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.q,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED)
        jacobian = np.zeros((3, 6), dtype=float)
        for out_col, v_index in enumerate(self.v_indices):
            jacobian[:, out_col] = jacobian_full[:3, v_index]

        lhs = jacobian @ jacobian.T + (
            self.damping * self.damping * np.eye(3, dtype=float))
        dq_raw = jacobian.T @ np.linalg.solve(lhs, task_velocity)
        dq_limited = self.limit_joint_velocity(dq_raw)

        if not finite_vector(dq_raw) or not finite_vector(dq_limited):
            raise ValueError('non-finite joint velocity')
        return error, dq_raw, dq_limited

    def compute_se3_debug_command(self):
        delta = self.latest_visual_pose.inverse() * self.target_pose
        error = np.asarray(pin.log6(delta).vector, dtype=float).reshape(6)
        if not finite_vector(error):
            raise ValueError('non-finite SE(3) error')

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        jacobian_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.q,
            self.ee_frame_id,
            pin.LOCAL)
        jacobian = np.zeros((6, 6), dtype=float)
        for out_col, v_index in enumerate(self.v_indices):
            jacobian[:, out_col] = jacobian_full[:, v_index]

        task_velocity = self.task_gain * error
        task_velocity = np.clip(
            task_velocity,
            -self.max_task_velocity,
            self.max_task_velocity)
        lhs = jacobian @ jacobian.T + (
            self.damping * self.damping * np.eye(6, dtype=float))
        dq_raw = jacobian.T @ np.linalg.solve(lhs, task_velocity)
        dq_limited = self.limit_joint_velocity(dq_raw)

        if not finite_vector(dq_raw) or not finite_vector(dq_limited):
            raise ValueError('non-finite joint velocity')
        return error, dq_raw, dq_limited

    def limit_joint_velocity(self, dq_raw):
        dq_limited = np.clip(
            dq_raw,
            -self.max_joint_velocity,
            self.max_joint_velocity)
        if self.command_would_push_joint_limit(dq_limited):
            self.last_status = 'joint_limit_guard'
            return np.zeros(6, dtype=float)
        return dq_limited

    def command_would_push_joint_limit(self, dq):
        if self.joint_limit_margin_rad <= 0.0:
            return False
        for index, (position, velocity, lower, upper) in enumerate(zip(
                self.q[self.q_indices],
                dq,
                self.lower_limits,
                self.upper_limits)):
            if math.isfinite(lower) and position <= lower + self.joint_limit_margin_rad:
                if velocity < 0.0:
                    self.get_logger().warn(
                        f'{self.joint_names[index]} near lower limit; zeroing command',
                        throttle_duration_sec=2.0)
                    return True
            if math.isfinite(upper) and position >= upper - self.joint_limit_margin_rad:
                if velocity > 0.0:
                    self.get_logger().warn(
                        f'{self.joint_names[index]} near upper limit; zeroing command',
                        throttle_duration_sec=2.0)
                    return True
        return False

    def publish_outputs(self, error, dq_raw, dq_limited):
        self.error_pub.publish(self.make_array(error))
        self.dq_raw_pub.publish(self.make_array(dq_raw))
        self.dq_limited_pub.publish(self.make_array(dq_limited))
        if self.target_pose is None:
            self.target_pose_pub.publish(self.make_array([0.0] * 7))
        else:
            self.target_pose_pub.publish(self.make_array(se3_to_xyz_quat(self.target_pose)))

    def publish_command(self, dq_limited):
        if self.command_pub is None:
            return
        if self.target_lock_ns > 0:
            elapsed_sec = (
                self.get_clock().now().nanoseconds - self.target_lock_ns) * 1e-9
            if elapsed_sec < self.command_start_delay_sec:
                return
        self.command_pub.publish(self.make_array(dq_limited))

    @staticmethod
    def make_array(values):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        return msg


def main(argv=None):
    rclpy.init(args=argv)
    node = VisualEeStabilizationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
