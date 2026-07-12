#!/usr/bin/env python3
"""6DOF A-L1-GAMMA 机械臂 Pinocchio IK 模块。

默认加载 src/arm-platform/config/arm.urdf，末端 frame 为 link6。
采用阻尼最小二乘迭代求解，默认只做 3D 位置 IK。
"""

import os

import numpy as np
import pinocchio as pin

DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'arm.urdf')

EE_FRAME = 'link6'


class PinocchioIK6Dof:
    def __init__(self, urdf_path: str = DEFAULT_URDF, ee_frame: str = EE_FRAME):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(ee_frame)
        self.nq = self.model.nq
        self.q_lower = self.model.lowerPositionLimit
        self.q_upper = self.model.upperPositionLimit

    def forward(self, q: np.ndarray):
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
        q = pin.neutral(self.model) if q_init is None else q_init.copy()

        for _ in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            oMf = self.data.oMf[self.frame_id]
            err_pos = target_position - oMf.translation

            if target_rotation is not None:
                err_rot = pin.log3(target_rotation @ oMf.rotation.T)
                err = np.concatenate([err_pos, rot_weight * err_rot])
            else:
                err = err_pos

            if np.linalg.norm(err_pos) < eps:
                return np.clip(q, self.q_lower, self.q_upper), True

            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id,
                                          pin.LOCAL_WORLD_ALIGNED)
            if target_rotation is not None:
                J = np.vstack([J6[:3, :], rot_weight * J6[3:, :]])
            else:
                J = J6[:3, :]

            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + damp * np.eye(JJt.shape[0]), err)
            q = pin.integrate(self.model, q, dq * dt)
            q = np.clip(q, self.q_lower, self.q_upper)

        return q, np.linalg.norm(err_pos) < eps


if __name__ == '__main__':
    ik = PinocchioIK6Dof()
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
