#!/usr/bin/env python3
"""6DOF 机械臂末端画圆演示脚本。

让机械臂末端 link6 在笛卡尔空间画一个圆，每帧用 IK 解出关节角，
发布到 /student/joint_command。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pinocchio_ik_6dof import PinocchioIK6Dof

JOINT_COUNT = 6
PUBLISH_RATE_HZ = 50.0
CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
CIRCLE_RADIUS = 0.08
PERIOD_SEC = 8.0


class MoveArmIkDemo6Dof(Node):
    def __init__(self):
        super().__init__('move_arm_ik_demo_6dof')
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.ik = PinocchioIK6Dof()
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.q_last = None

        p0 = self._target(0.0)
        q0, ok = self.ik.solve(p0)
        if not ok:
            raise RuntimeError(f'起始点 {p0.tolist()} IK 不可达，请调整圆心/半径')
        self.q_last = q0

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'6DOF 末端画圆: 圆心 {CIRCLE_CENTER.tolist()}, 半径 {CIRCLE_RADIUS} m, Ctrl+C 停止')

    def _target(self, t: float) -> np.ndarray:
        w = 2.0 * math.pi / PERIOD_SEC
        return CIRCLE_CENTER + CIRCLE_RADIUS * np.array(
            [0.0, math.cos(w * t), math.sin(w * t)])

    def tick(self):
        self.t += self.dt
        q, ok = self.ik.solve(self._target(self.t), q_init=self.q_last)
        if not ok:
            self.get_logger().warn('IK 未收敛，跳过本帧', throttle_duration_sec=2.0)
            return
        self.q_last = q

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = q.tolist()
        msg.velocity = [0.0] * JOINT_COUNT
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MoveArmIkDemo6Dof()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
