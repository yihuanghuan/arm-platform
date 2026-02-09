#pragma once
#include <memory>
#include <manipulator/motor/i_motor.h>
#include <manipulator/protocol/protocol_v1.h>

namespace manipulator::motor {

class DM4310P final : public IMotor {
 public:
  DM4310P(protocol::ProtocolV1::SharedPtr protocol);
  virtual ~DM4310P() = default;

  void UpdateCommand(const sensor_msgs::msg::JointState& state) override;
  void UpdateState() override;
 
 private:
  protocol::ProtocolV1::SharedPtr protocol_;
  uint8_t id_;
  double position_;
  double velocity_;
  double torque_;
  double temperature_;

  double kp_;
  double kd_;
};

} // namespace manipulator::motor