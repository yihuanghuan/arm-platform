#!/usr/bin/env python3
"""6DOF 机械臂关节空间摆动演示脚本。

向 /student/joint_command 发布正弦摆动指令，让 6 轴机械臂前几个关节小幅度来回摆动。
用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 move_arm_demo_6dof.py
按 Ctrl+C 停止。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_COUNT = 6
PUBLISH_RATE_HZ = 50.0
AMPLITUDE = 0.3
PERIOD_SEC = 4.0


class MoveArmDemo6Dof(Node):
    def __init__(self):
        super().__init__('move_arm_demo_6dof')
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info('开始发布 6DOF 摆动指令，Ctrl+C 停止')

    def tick(self):
        self.t += self.dt
        w = 2.0 * math.pi / PERIOD_SEC
        pos = [0.0] * JOINT_COUNT
        vel = [0.0] * JOINT_COUNT

        for i in range(3):
            phase = i * math.pi / 3.0
            pos[i] = AMPLITUDE * math.sin(w * self.t + phase)
            vel[i] = AMPLITUDE * w * math.cos(w * self.t + phase)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = pos
        msg.velocity = vel
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MoveArmDemo6Dof()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
