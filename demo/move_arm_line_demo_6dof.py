#!/usr/bin/env python3
"""6DOF 机械臂末端直线往返演示脚本。

让机械臂末端 link6 在两点之间沿直线来回运动，每帧用 IK 解出关节角，
发布到 /student/joint_command。插值使用余弦曲线，端点处速度为零。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pinocchio_ik_6dof import PinocchioIK6Dof

JOINT_COUNT = 6
PUBLISH_RATE_HZ = 50.0
POINT_A = np.array([0.35, -0.15, 0.25])
POINT_B = np.array([0.35, 0.15, 0.25])
PERIOD_SEC = 6.0


class MoveArmLineDemo6Dof(Node):
    def __init__(self):
        super().__init__('move_arm_line_demo_6dof')
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.ik = PinocchioIK6Dof()
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.q_last = None

        for name, p in (('A', POINT_A), ('B', POINT_B)):
            q, ok = self.ik.solve(p, q_init=self.q_last)
            if not ok:
                raise RuntimeError(f'端点 {name} {p.tolist()} IK 不可达，请调整坐标')
            self.q_last = q
        self.q_last, _ = self.ik.solve(POINT_A)

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'6DOF 末端直线往返: A={POINT_A.tolist()} <-> B={POINT_B.tolist()}, '
            f'周期 {PERIOD_SEC}s, Ctrl+C 停止')

    def _target(self, t: float) -> np.ndarray:
        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / PERIOD_SEC))
        return POINT_A + s * (POINT_B - POINT_A)

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
    node = MoveArmLineDemo6Dof()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
