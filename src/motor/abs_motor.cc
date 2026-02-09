#include <manipulator/motor/abs_motor.h>

namespace manipulator::motor {
AbsMotor::AbsMotor(bus::IBus::SharedPtr bus, protocol::IProtocol::SharedPtr protocol) {

}

void AbsMotor::UpdateState(const std::vector<uint8_t>& data) {
  protocol_->Pop(data);
}

void AbsMotor::UpdateCommand(const sensor_msgs::msg::JointState& state) {
  std::vector<uint8_t> bytes_to_send;
  protocol_->MakeFrame(state, bytes_to_send);
}
}