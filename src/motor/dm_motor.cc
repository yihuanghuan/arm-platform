#include <manipulator/motor/dm_motor.h>
#include <rclcpp/rclcpp.hpp>

auto logger = rclcpp::get_logger("Controller");
namespace manipulator::motor {
DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
                 float kp, float kd)
 : protocol_(protocol), id_(id), default_kp_(kp), default_kd_(kd), vel_set_(0) {

}

void DMMotor::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_);
  temperature_ = protocol_->GetTemperature(id_);
  if (not is_received_) pos_set_ = position_;
  is_received_ = true;
}

void DMMotor::SetState(const sensor_msgs::msg::JointState& state) {
  dummy_interface::msg::MotorControl cmd;

  cmd.position.push_back(state.position[0]);
  cmd.velocity.push_back(state.velocity[0]);
  cmd.p.push_back(default_kp_);
  cmd.d.push_back(default_kd_);
  UpdateCommand(cmd);
}

void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (not is_received_) return;
  protocol_->SetKp(id_, cmd.p[id_]);
  protocol_->SetKd(id_, cmd.d[id_]);
  protocol_->SetCurrent(id_, cmd.current[id_]);
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
  
  if (id_ == 6) pos_set_ = cmd.position[id_];
  RCLCPP_INFO(logger, "pos_set_: %f, cmd.position[id_]: %f, position_: %f", pos_set_, cmd.position[id_], position_);
  protocol_->SetPosition(id_, pos_set_);
  protocol_->SetVelocity(id_, cmd.velocity[id_]);
}
}