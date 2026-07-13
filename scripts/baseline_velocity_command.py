#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray


class BaselineVelocityCommand(Node):
    def __init__(self, args):
        super().__init__(
            'baseline_velocity_command',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
            ])
        self.args = args
        self.publisher = self.create_publisher(Float64MultiArray, args.output_topic, 10)
        self.command = Float64MultiArray()
        self.command.data = [0.0] * args.joint_count
        self.timer = self.create_timer(1.0 / args.rate_hz, self.publish_command)
        self.get_logger().info(
            f'Publishing {args.joint_count} zero velocity commands to '
            f'{args.output_topic} at {args.rate_hz:.1f} Hz')

    def publish_command(self):
        self.publisher.publish(self.command)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Publish zero joint velocities for moving-base baseline experiments.')
    parser.add_argument('--output-topic', default='/arm_velocity_controller/commands')
    parser.add_argument('--joint-count', type=int, default=6)
    parser.add_argument('--rate-hz', type=float, default=100.0)
    parser.add_argument('--use-sim-time', action='store_true')
    args, _ = parser.parse_known_args(argv)
    if args.joint_count <= 0:
        raise SystemExit('--joint-count must be positive')
    if args.rate_hz <= 0.0:
        raise SystemExit('--rate-hz must be positive')
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = BaselineVelocityCommand(args)
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
    main()
