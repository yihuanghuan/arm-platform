#include <manipulator/arm/a_l1_beta.h>
#include <manipulator/protocol/protocol_factory.h>
#include <manipulator/protocol/protocol_v1.h>
#include <manipulator/bus/serial_bus.h>

namespace manipulator::arm {

AL1Beta::AL1Beta() {

}

void AL1Beta::Init(const std::string& port, uint32_t baud) {
  protocol_ = std::make_shared<protocol::ProtocolV1>();

  auto joint1 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint1", joint1);
  auto joint2 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint2", joint2);
  auto joint3 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint3", joint3);
  auto joint4 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint4", joint4);
  auto joint5 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint5", joint5);
  auto joint6 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint6", joint6);
  auto joint7 = std::make_shared<motor::DM4310P>(protocol_);
  AddMotor("joint7", joint7);

  protocol_->Attach(joint1);
  protocol_->Attach(joint2);
  protocol_->Attach(joint3);
  protocol_->Attach(joint4);
  protocol_->Attach(joint5);
  protocol_->Attach(joint6);
  protocol_->Attach(joint7);

  auto protocol_factory = std::make_unique<ProtocolFactory>();
  protocol_factory->Add(std::vector<uint8_t>{0xFE, 0x55}, protocol_);

  auto serial_bus = std::make_unique<bus::SerialBus>(port, baud, protocol_factory);
  SetBus(serial_bus);
}

void AL1Beta::GetState(JointState& state) {
  for(uint8_t i = 0; i < 7; i++) {
    state.position = protocol_->GetPosition(i);
    state.velocity = protocol_->GetVelocity(i);
    state.current = protocol_->GetCurrent(i);
    state.voltage = protocol_->GetVoltage(i);
    state.temperature = protocol_->GetTemperature(i);
  }
}
}