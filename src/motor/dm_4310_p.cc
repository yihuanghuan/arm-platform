#include <manipulator/motor/dm_4310_p.h>

namespace manipulator::motor {
DM4310P::DM4310P(protocol::ProtocolV1::SharedPtr protocol)
 : protocol_(protocol) {

}

void DM4310P::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_); // DM motors return torque instead of current
  temperature_ = protocol_->GetTemperature(id_);
}

void DM4310P::UpdateCommand(const sensor_msgs::msg::JointState& state) {
  protocol_->SetPosition(id_, state.position[0]);
  protocol_->SetVelocity(id_, state.velocity[0]);
  protocol_->SetCurrent(id_, state.effort[0]);
}
}