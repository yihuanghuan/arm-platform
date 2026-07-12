#!/usr/bin/env python3
"""基于 Pinocchio 的逆运动学 (IK) 模块。

针对 A-L1-GAMMA 7 自由度机械臂 (src/arm-platform/config/arm.urdf)。
采用阻尼最小二乘 (Damped Least Squares / Levenberg-Marquardt) 迭代求解。

说明:
- 该臂 joint6 与 joint7 轴线共线 (均为 x 轴), 末端姿态存在病态方向,
  因此默认只做 3D 位置 IK (7 关节冗余, 求解稳定);
  如需姿态, 可传入 target_rotation, 姿态误差以较低权重加入任务。
- 求解结果自动钳制到 URDF 关节限位内。

用法示例:
    from pinocchio_ik import PinocchioIK
    ik = PinocchioIK()
    q = ik.solve(np.array([0.4, 0.0, 0.2]))        # 仅位置
    pos, rot = ik.forward(q)                        # 正解验证
"""

import os

import numpy as np
import pinocchio as pin

DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'arm.urdf')

EE_FRAME = 'link7'  # 末端 frame 名


class PinocchioIK:
    def __init__(self, urdf_path: str = DEFAULT_URDF, ee_frame: str = EE_FRAME):
        # 1. 从 URDF 构建 Pinocchio 运动学模型。
        # model 保存机器人拓扑、关节、frame、关节限位等静态信息；
        # data 保存每次 FK / Jacobian 计算产生的中间结果，例如各 frame 位姿。
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # IK 的输出目标是让末端 frame 到达期望位姿。
        # 当前项目的末端 frame 默认是 link7，对应 7 自由度机械臂的最后一节。
        self.frame_id = self.model.getFrameId(ee_frame)
        self.nq = self.model.nq

        # URDF 关节限位；每次更新 q 后都会 clip 到这个范围内，避免解跑出物理限制。
        self.q_lower = self.model.lowerPositionLimit
        self.q_upper = self.model.upperPositionLimit

    def forward(self, q: np.ndarray):
        """正解: 返回末端 (position(3,), rotation(3,3))。"""
        # FK: q -> 当前末端位姿。
        # framesForwardKinematics 会把每个 frame 的全局位姿写入 self.data.oMf。
        pin.framesForwardKinematics(self.model, self.data, q)
        oMf = self.data.oMf[self.frame_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def solve(self,
              target_position: np.ndarray,
              target_rotation: np.ndarray = None,
              q_init: np.ndarray = None,
              max_iter: int = 200,
              eps: float = 1e-4,
              dt: float = 0.5,
              damp: float = 1e-6,
              rot_weight: float = 0.1):
        """求解 IK。

        Args:
            target_position: 目标位置 (3,), base_link 坐标系。
            target_rotation: 可选目标姿态 (3,3) 旋转矩阵; None 则只解位置。
            q_init: 初值 (nq,); None 则用零位。重复调用时传上次解可热启动。
            max_iter / eps: 迭代上限与位置误差收敛阈值 (m)。
            dt: 迭代步长。
            damp: 阻尼系数 (避免奇异)。
            rot_weight: 姿态任务权重 (相对位置任务)。

        Returns:
            (q, ok): q 为关节角 (nq,) 已钳制限位; ok 表示是否收敛。
        """
        # q 是本次 IK 迭代的当前关节配置。
        # 如果外部传入 q_init，就相当于从上一帧解附近热启动，轨迹会更连续；
        # 否则从 Pinocchio 的 neutral 配置开始。
        q = pin.neutral(self.model) if q_init is None else q_init.copy()

        for _ in range(max_iter):
            # -------------------------
            # A. FK: 用当前 q 算末端实际位姿 f(q)
            # -------------------------
            pin.framesForwardKinematics(self.model, self.data, q)
            oMf = self.data.oMf[self.frame_id]

            # -------------------------
            # B. 误差: target - current
            # -------------------------
            # 位置误差 e_p = p_target - p_current。
            # 这里 target_position 和 oMf.translation 都在 Pinocchio 的全局/world 坐标下。
            err_pos = target_position - oMf.translation

            if target_rotation is not None:
                # 姿态误差 e_R = log(R_target * R_current^T)。
                # log3 会把 SO(3) 旋转矩阵误差转成 3 维旋转向量：
                # 方向是旋转轴，模长近似还需要转多少弧度。
                err_rot = pin.log3(target_rotation @ oMf.rotation.T)

                # 6D 任务误差 e = [e_p; w_R * e_R]。
                # rot_weight 降低姿态任务权重，避免共线轴带来的病态方向主导求解。
                err = np.concatenate([err_pos, rot_weight * err_rot])
            else:
                # 默认只做 3D 位置 IK。
                # 对 7 自由度臂来说，这是冗余任务：关节数多于任务维度，通常更稳定。
                err = err_pos

            # 收敛判定只看位置误差。
            # 即使传入姿态目标，当前接口也把位置到达作为 ok 的主判据；
            # 姿态只作为加权项参与每一步的 dq 计算。
            if np.linalg.norm(err_pos) < eps:
                return np.clip(q, self.q_lower, self.q_upper), True

            # -------------------------
            # C. Jacobian: 线性化当前 q 附近的末端运动
            # -------------------------
            # J6 是 6 x nq 的末端 frame 雅可比：
            #   前 3 行: 关节速度 dq 对末端线速度 / 位置变化的影响；
            #   后 3 行: 关节速度 dq 对末端角速度 / 姿态变化的影响。
            # LOCAL_WORLD_ALIGNED 表示速度方向用 world 坐标轴表达，
            # 因此前 3 行可以直接对应 world 系下的 err_pos。
            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id,
                                          pin.LOCAL_WORLD_ALIGNED)
            if target_rotation is not None:
                # 姿态 Jacobian 和姿态误差使用同一个权重，保持等式 J*dq ≈ err 的尺度一致。
                J = np.vstack([J6[:3, :], rot_weight * J6[3:, :]])
            else:
                # 只做位置 IK 时，任务 Jacobian 是 3 x nq。
                J = J6[:3, :]

            # -------------------------
            # D. 阻尼最小二乘 IK: 求关节增量 dq
            # -------------------------
            # 目标是在当前线性化下求 J * dq ≈ err。
            # 这里使用右伪逆形式：
            #   dq = J^T (J J^T + λI)^-1 err
            # damp 是阻尼项，避免 Jacobian 奇异或接近奇异时 dq 过大。
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + damp * np.eye(JJt.shape[0]), err)

            # -------------------------
            # E. 更新 q_target 候选值
            # -------------------------
            # dt 是迭代步长。Pinocchio 的 integrate 按机器人关节流形更新 q；
            # 对普通 revolute joint 来说，可以理解为 q_new = q_old + dq * dt。
            q = pin.integrate(self.model, q, dq * dt)

            # 迭代中保持在限位内, 避免解跑飞
            q = np.clip(q, self.q_lower, self.q_upper)

        return q, np.linalg.norm(err_pos) < eps


if __name__ == '__main__':
    # 自测: 取一个可达位姿做 FK, 再用 IK 反求, 验证误差
    ik = PinocchioIK()
    print(f'模型: nq={ik.nq}, 末端 frame={EE_FRAME}')
    print(f'限位 lower: {np.round(ik.q_lower, 3)}')
    print(f'限位 upper: {np.round(ik.q_upper, 3)}')

    rng = np.random.default_rng(0)
    n_ok = 0
    for i in range(10):
        q_true = rng.uniform(ik.q_lower * 0.5, ik.q_upper * 0.5)
        p_target, _ = ik.forward(q_true)
        q_sol, ok = ik.solve(p_target)
        p_sol, _ = ik.forward(q_sol)
        e = np.linalg.norm(p_sol - p_target)
        n_ok += ok
        print(f'[{i}] target={np.round(p_target, 4)} err={e:.2e} ok={ok}')
    print(f'收敛 {n_ok}/10')
