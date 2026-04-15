#pragma once
#include <memory>
#include <manipulator/robotics/motor/i_motor.h>
#include <manipulator/robotics/protocol/protocol_v1.h>

namespace manipulator::motor {

enum class CoordinateSystem {
  RightHand,
  LeftHand
};

class DMMotor final : public IMotor {
 public:
  DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
          float kp, float kd, CoordinateSystem coord_system = CoordinateSystem::RightHand);
  virtual ~DMMotor() = default;

  void UpdateCommand(const dummy_interface::msg::MotorControl& cmd) override;
  void SetState(const sensor_msgs::msg::JointState& state) override;
  void UpdateState() override;

  double GetPosition() const override;
  double GetVelocity() const override;
  double GetCurrent() const override;
  double GetTemperature() const override;
  double GetVoltage() const override;
 
 private:
  protocol::ProtocolV1::SharedPtr protocol_;
  uint8_t id_;
  double position_;
  double velocity_;
  double torque_;
  double temperature_;
  double voltage_;
  float default_kp_;
  float default_kd_;

  double pos_set_;
  double vel_set_;
  bool is_received_;
  const double DT_ = 0.005;
  CoordinateSystem coord_system_;
};

} // namespace manipulator::motor