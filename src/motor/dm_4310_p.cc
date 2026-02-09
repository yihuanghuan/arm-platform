#include <manipulator/motor/dm_4310_p.h>

namespace manipulator::motor {
DM4310P::DM4310P(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, double kp, double kd)
 : protocol_(protocol), id_(id), kp_(kp), kd_(kd) {

}

void DM4310P::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_); // DM motors return torque instead of current
  temperature_ = protocol_->GetTemperature(id_);
}

void DM4310P::UpdateCommand(const sensor_msgs::msg::JointState& state) {

  double pos_err = state.position[0] - position_;

  if(id_ < 6) pos_set_ =  position_ + std::min(std::max(-0.05, pos_err), 0.05);
  else pos_set_ = state.position[0];
  protocol_->SetPosition(id_, pos_set_);
  protocol_->SetVelocity(id_, state.velocity[0]);
  // protocol_->SetCurrent(id_, state.effort[0]);
  protocol_->SetKp(id_, kp_);
  protocol_->SetKd(id_, kd_);

}
}