#include <manipulator/controller/smooth_position_controller.h>

namespace manipulator::controller {
  JointCommand SmoothPositionController::Compute(
    const JointStates& joint_states,
    const JointSetPoint& joint_setpoint,
    double dt) {
      JointCommand cmd;
      size_t num_joints = joint_states.position.size();
      if(pos_set_.empty()) {
        pos_set_ = joint_states.position;
      }
      cmd.position.resize(num_joints);
      cmd.velocity.resize(num_joints);
      cmd.current.resize(num_joints);
      std::vector<double> kp = {10, 10, 10, 1, 1, 1, 1};
      cmd.p.resize(num_joints);
      for (size_t i = 0; i < num_joints; ++i) {
        cmd.p[i] = kp[i];
      }
      cmd.d = std::vector<double>(num_joints, 0.1);

      for (size_t i = 0; i < num_joints; ++i) {
        double pos_err = joint_setpoint.q[i] - joint_states.position[i];
        double delta = pos_err * dt;
        // 最小步长限制
        if (std::abs(delta) < 0.01) {
            delta = (delta >= 0) ? 0.01 : -0.01;
        }
        pos_set_[i] += delta;
        // clamp（防止超调）
        pos_set_[i] = std::clamp(
          pos_set_[i],
          std::min(joint_setpoint.q[i], joint_states.position[i]),
          std::max(joint_setpoint.q[i], joint_states.position[i])
        );
        cmd.position[i] = pos_set_[i];
      }
    return cmd;
  }
} // namespace manipulator::controller