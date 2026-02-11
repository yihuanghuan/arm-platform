#pragma once
#include <memory>
#include <manipulator/motor/i_motor.h>
#include <manipulator/protocol/protocol_v1.h>

namespace manipulator::motor {

class DMMotor final : public IMotor {
 public:
  DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, double kp, double kd);
  virtual ~DMMotor() = default;

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
  double pos_set_;
  double vel_set_;
  bool is_controlled_;
  const double MAX_ACCELERATION_ = 100;
  const double DT_ = 0.005;
  const double MAX_VELOCITY_ = 20;
};

} // namespace manipulator::motor