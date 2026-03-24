#include <manipulator/motor/dm_motor.h>
#include <rclcpp/rclcpp.hpp>

auto logger = rclcpp::get_logger("Controller");
namespace manipulator::motor {
DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
                 float kp, float kd, bool is_mirror)
 : protocol_(protocol), id_(id), default_kp_(kp), default_kd_(kd), vel_set_(0), is_mirror_(is_mirror) {

}

void DMMotor::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  // avoid sudden jitter caused by incorrent initial set value
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_); // DM motors return torque instead of current
  temperature_ = protocol_->GetTemperature(id_);
  if(is_mirror_) {
    torque_ = -torque_;
    position_ = -position_;
    velocity_ = -velocity_;
  }
  if (not is_received_) pos_set_ = position_;
  is_received_ = true;
}

void DMMotor::SetState(const sensor_msgs::msg::JointState& state) {
  dummy_interface::msg::MotorControl cmd;

  cmd.position.push_back(is_mirror_ ? -state.position[0] : state.position[0]);
  cmd.velocity.push_back(is_mirror_ ? -state.velocity[0] : state.velocity[0]);
  cmd.p.push_back(default_kp_);
  cmd.d.push_back(default_kd_);
  UpdateCommand(cmd);
}

void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (not is_received_) return;
  // Use constant Kp and Kd values; only a one-time publication is needed.
  protocol_->SetKp(id_, cmd.p[id_]);
  protocol_->SetKd(id_, cmd.d[id_]);
  protocol_->SetCurrent(id_, is_mirror_ ? -cmd.current[id_] : cmd.current[id_]);
  if (cmd.position.empty()) return;

  double pos_err = cmd.position[id_] - position_;
  if (abs(pos_err) < 0.01) {
    pos_set_ = position_;
    return;
  }

  double delta = pos_err * DT_;
  if (std::abs(delta) < 0.01) {
    delta = (delta >= 0) ? 0.01 : -0.01;
  }
  pos_set_ += delta;
  pos_set_ = std::clamp(pos_set_, std::min(cmd.position[id_], position_), std::max(cmd.position[id_], position_));
  
  if (id_ == 6) pos_set_ = cmd.position[id_]; // joint7(gripper) requires rapid movement
  RCLCPP_INFO(logger, "pos_set_: %f, cmd.position[id_]: %f, position_: %f", pos_set_, cmd.position[id_], position_);
  protocol_->SetPosition(id_, is_mirror_ ? -pos_set_ : pos_set_);
  protocol_->SetVelocity(id_, is_mirror_ ? -cmd.velocity[id_] : cmd.velocity[id_]);  // protocol_->SetPosition(id_, is_mirror_ ? -cmd.position[id_] : cmd.position[id_]);
  // protocol_->SetVelocity(id_, is_mirror_ ? -cmd.velocity[id_] : cmd.velocity[id_]);
}
}