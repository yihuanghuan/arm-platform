#pragma once
#include <memory>
#include <sensor_msgs/msg/joint_state.hpp>
#include <manipulator/motor/i_motor.h>
#include <manipulator/protocol/i_protocol.h>

namespace manipulator::motor {

class AbsMotor : public IMotor {
 public:
  AbsMotor(protocol::IProtocol::SharedPtr protocol);
  virtual ~AbsMotor() = default;

  // IMotor interface implementation
  void UpdateState() override;
  void UpdateCommand(const sensor_msgs::msg::JointState& state) override;
 
 private:
  protocol::IProtocol::SharedPtr protocol_;
};

} // namespace manipulator::motor