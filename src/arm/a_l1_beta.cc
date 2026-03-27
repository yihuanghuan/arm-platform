#include <manipulator/arm/a_l1_beta.h>
#include <manipulator/protocol/protocol_factory.h>
#include <manipulator/protocol/protocol_v1.h>
#include <manipulator/bus/serial_bus.h>
#include <manipulator/motor/dm_motor.h>

namespace manipulator::arm {

AL1Beta::AL1Beta() {

}

void AL1Beta::Init(const std::string& port, uint32_t baud) {
  protocol_ = std::make_shared<protocol::ProtocolV1>();

  auto joint1 = std::make_shared<motor::DMMotor>(protocol_, 0, 30, 0.2);
  AddMotor("joint1", joint1);
  auto joint2 = std::make_shared<motor::DMMotor>(protocol_, 1, 50, 0.2);
  AddMotor("joint2", joint2);
  auto joint3 = std::make_shared<motor::DMMotor>(protocol_, 2, 50, 0.2);
  AddMotor("joint3", joint3);
  auto joint4 = std::make_shared<motor::DMMotor>(protocol_, 3, 10, 0.2);
  AddMotor("joint4", joint4);
  auto joint5 = std::make_shared<motor::DMMotor>(protocol_, 4, 2, 0.5);
  AddMotor("joint5", joint5);
  auto joint6 = std::make_shared<motor::DMMotor>(protocol_, 5, 5, 0.2);
  AddMotor("joint6", joint6);
  auto joint7 = std::make_shared<motor::DMMotor>(protocol_, 6, 1, 0.2);
  AddMotor("joint7", joint7);

  protocol_->Attach(joint1);
  protocol_->Attach(joint2);
  protocol_->Attach(joint3);
  protocol_->Attach(joint4);
  protocol_->Attach(joint5);
  protocol_->Attach(joint6);
  protocol_->Attach(joint7);

  auto protocol_factory = std::make_unique<protocol::ProtocolFactory>();
  protocol_factory->Add(std::vector<uint8_t>{0xFE, 0x55}, protocol_);

  SetBus(std::make_unique<bus::SerialBus>(port, baud, std::move(protocol_factory)));
}

void AL1Beta::GetState(JointState& state) {
  GetJointStates();
  for(uint8_t i = 0; i < 7; i++) {
    state.position[i] = protocol_->GetPosition(i);
    state.velocity[i] = protocol_->GetVelocity(i);
    state.current[i] = protocol_->GetCurrent(i);
    state.voltage[i] = protocol_->GetVoltage(i);
    state.temperature[i] = protocol_->GetTemperature(i);
  }
  // state.position[1] = -state.position[1];
  // state.velocity[1] = -state.velocity[1];
  // state.current[1] = -state.current[1];
}
}