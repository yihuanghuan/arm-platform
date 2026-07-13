#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class StudentJointVelocityBridge(Node):
    def __init__(self):
        super().__init__('student_joint_velocity_bridge')

        self.declare_parameter('input_topic', '/student/joint_command')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('controller_command_topic', '/arm_velocity_controller/commands')
        self.declare_parameter('joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('lower_limits', [-3.14, -3.14, -3.14, -3.14, -3.14, -3.15])
        self.declare_parameter('upper_limits', [3.14, 3.14, 3.14, 3.14, 3.14, 3.15])
        self.declare_parameter('kp', 4.0)
        self.declare_parameter('feedforward_scale', 1.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('command_timeout_sec', 0.25)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.lower_limits = list(self.get_parameter('lower_limits').value)
        self.upper_limits = list(self.get_parameter('upper_limits').value)
        self.kp = float(self.get_parameter('kp').value)
        self.feedforward_scale = float(self.get_parameter('feedforward_scale').value)
        self.max_velocity = abs(float(self.get_parameter('max_velocity').value))
        self.position_tolerance = abs(float(self.get_parameter('position_tolerance').value))
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout_sec = abs(float(self.get_parameter('command_timeout_sec').value))

        if not (
            len(self.joint_names)
            == len(self.lower_limits)
            == len(self.upper_limits)
        ):
            raise RuntimeError('joint_names/lower_limits/upper_limits 参数长度必须一致')
        if publish_rate_hz <= 0.0:
            raise RuntimeError('publish_rate_hz 必须大于 0')

        self.current_positions = None
        self.target_positions = None
        self.target_velocities = [0.0] * len(self.joint_names)
        self.last_command_time = None

        input_topic = self.get_parameter('input_topic').value
        joint_state_topic = self.get_parameter('joint_state_topic').value
        output_topic = self.get_parameter('controller_command_topic').value

        self.publisher = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.command_subscription = self.create_subscription(
            JointState, input_topic, self.command_callback, 10)
        self.joint_state_subscription = self.create_subscription(
            JointState, joint_state_topic, self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_velocity_command)

        self.get_logger().info(
            f'Bridging {input_topic} JointState targets to {output_topic} velocity commands')

    def joint_state_callback(self, msg):
        if not msg.name or len(msg.position) < len(msg.name):
            return

        position_by_name = dict(zip(msg.name, msg.position))
        if any(name not in position_by_name for name in self.joint_names):
            return

        self.current_positions = [position_by_name[name] for name in self.joint_names]

    def command_callback(self, msg):
        positions = self._extract_positions(msg)
        if positions is None:
            return

        velocities = self._extract_velocities(msg)
        self.target_positions = [
            min(max(position, lower), upper)
            for position, lower, upper in zip(
                positions, self.lower_limits, self.upper_limits)
        ]
        self.target_velocities = velocities
        self.last_command_time = self.get_clock().now()

    def publish_velocity_command(self):
        if self.current_positions is None:
            return

        if self.target_positions is None:
            velocities = [0.0] * len(self.joint_names)
        else:
            velocities = []
            command_is_fresh = (
                self.last_command_time is not None
                and (
                    self.get_clock().now() - self.last_command_time
                ).nanoseconds * 1e-9 <= self.command_timeout_sec
            )
            for current, target, target_velocity in zip(
                self.current_positions, self.target_positions, self.target_velocities
            ):
                feedforward_velocity = target_velocity if command_is_fresh else 0.0
                error = target - current
                command = self.kp * error + self.feedforward_scale * feedforward_velocity
                if abs(error) <= self.position_tolerance and abs(feedforward_velocity) < 1e-6:
                    command = 0.0
                velocities.append(min(max(command, -self.max_velocity), self.max_velocity))

        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)

    def _extract_positions(self, msg):
        positions = self._ordered_values(msg.name, msg.position, 'position')
        if positions is None:
            return None
        if any(not math.isfinite(position) for position in positions):
            self.get_logger().warn(
                '/student/joint_command.position 包含非有限数值，已忽略',
                throttle_duration_sec=2.0)
            return None
        return positions

    def _extract_velocities(self, msg):
        velocities = self._ordered_values(msg.name, msg.velocity, 'velocity')
        if velocities is None:
            return [0.0] * len(self.joint_names)
        if any(not math.isfinite(velocity) for velocity in velocities):
            self.get_logger().warn(
                '/student/joint_command.velocity 包含非有限数值，已忽略前馈速度',
                throttle_duration_sec=2.0)
            return [0.0] * len(self.joint_names)
        return velocities

    def _ordered_values(self, names, values, field_name):
        if not values:
            return None

        if names:
            value_by_name = dict(zip(names, values))
            missing = [name for name in self.joint_names if name not in value_by_name]
            if missing:
                if field_name == 'position':
                    self.get_logger().warn(
                        f'/student/joint_command 缺少关节: {missing}',
                        throttle_duration_sec=2.0)
                return None
            return [value_by_name[name] for name in self.joint_names]

        if len(values) < len(self.joint_names):
            if field_name == 'position':
                self.get_logger().warn(
                    '/student/joint_command.position 长度不足，无法驱动 6 个关节',
                    throttle_duration_sec=2.0)
            return None

        return list(values[:len(self.joint_names)])


def main(args=None):
    rclpy.init(args=args)
    node = StudentJointVelocityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
