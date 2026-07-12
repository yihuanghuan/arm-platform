#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class StudentJointCommandBridge(Node):
    def __init__(self):
        super().__init__('student_joint_command_bridge')

        self.declare_parameter('input_topic', '/student/joint_command')
        self.declare_parameter('controller_command_topic', '/arm_position_controller/commands')
        self.declare_parameter('joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('lower_limits', [-3.14, -3.14, -3.14, -3.14, -3.14, -3.15])
        self.declare_parameter('upper_limits', [3.14, 3.14, 3.14, 3.14, 3.14, 3.15])
        self.declare_parameter('publish_initial_command', True)
        self.declare_parameter('initial_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.lower_limits = list(self.get_parameter('lower_limits').value)
        self.upper_limits = list(self.get_parameter('upper_limits').value)
        self.initial_positions = list(self.get_parameter('initial_positions').value)

        if not (
            len(self.joint_names)
            == len(self.lower_limits)
            == len(self.upper_limits)
            == len(self.initial_positions)
        ):
            raise RuntimeError(
                'joint_names/lower_limits/upper_limits/initial_positions 参数长度必须一致')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('controller_command_topic').value
        self.publish_initial_command = self.get_parameter('publish_initial_command').value
        self.received_student_command = False

        self.publisher = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.subscription = self.create_subscription(
            JointState, input_topic, self.command_callback, 10)
        self.initial_timer = None
        if self.publish_initial_command:
            self.initial_timer = self.create_timer(0.2, self.publish_initial_hold_command)

        self.get_logger().info(
            f'Bridging {input_topic} JointState.position to {output_topic} Float64MultiArray')

    def command_callback(self, msg):
        positions = self._extract_positions(msg)
        if positions is None:
            return

        self.received_student_command = True
        command = Float64MultiArray()
        command.data = [
            min(max(position, lower), upper)
            for position, lower, upper in zip(
                positions, self.lower_limits, self.upper_limits)
        ]
        self.publisher.publish(command)

    def publish_initial_hold_command(self):
        if self.received_student_command:
            self.initial_timer.cancel()
            return

        command = Float64MultiArray()
        command.data = [
            min(max(position, lower), upper)
            for position, lower, upper in zip(
                self.initial_positions, self.lower_limits, self.upper_limits)
        ]
        self.publisher.publish(command)

    def _extract_positions(self, msg):
        if msg.name:
            position_by_name = dict(zip(msg.name, msg.position))
            missing = [name for name in self.joint_names if name not in position_by_name]
            if missing:
                self.get_logger().warn(
                    f'/student/joint_command 缺少关节: {missing}',
                    throttle_duration_sec=2.0)
                return None
            positions = [position_by_name[name] for name in self.joint_names]
        else:
            if len(msg.position) < len(self.joint_names):
                self.get_logger().warn(
                    '/student/joint_command.position 长度不足，无法驱动 6 个关节',
                    throttle_duration_sec=2.0)
                return None
            positions = list(msg.position[:len(self.joint_names)])

        if any(not math.isfinite(position) for position in positions):
            self.get_logger().warn(
                '/student/joint_command.position 包含非有限数值，已忽略',
                throttle_duration_sec=2.0)
            return None

        return positions


def main(args=None):
    rclpy.init(args=args)
    node = StudentJointCommandBridge()
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
