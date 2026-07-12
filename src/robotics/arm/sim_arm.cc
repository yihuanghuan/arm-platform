#include <manipulator/robotics/arm/sim_arm.h>

#include <algorithm>
#include <cmath>

namespace manipulator::arm {

namespace {

// 空总线：仿真模式下无需任何硬件通信
class SimBus final : public bus::IBus {
 public:
  void Send() override {}
  void Read() override {}
};

// 理想伺服电机模型：位置指令经限位钳制后直接生效
class SimMotor final : public motor::IMotor {
 public:
  explicit SimMotor(uint8_t id) : id_(id) {}

  void UpdateState() override {}

  void UpdateCommand(const dummy_interface::msg::MotorControl& cmd) override {
    if (id_ >= cmd.position.size()) {
      return;
    }
    double desired_pos = cmd.position[id_];
    if (std::isnan(desired_pos)) {
      return;
    }
    desired_pos = std::clamp(desired_pos, lower_limit_, upper_limit_);
    if (id_ < cmd.velocity.size()) {
      velocity_ = cmd.velocity[id_];
    }
    position_ = desired_pos;
  }

  double GetPosition() const override { return position_; }
  double GetVelocity() const override { return velocity_; }
  double GetCurrent() const override { return 0.0; }
  double GetTemperature() const override { return 25.0; }
  double GetVoltage() const override { return 48.0; }
  double GetRatedTorque() const override { return rated_torque_; }
  void SetRateTorque(double rate_torque) override { rated_torque_ = rate_torque; }

  void SetJointLimit(double lower_limit, double upper_limit) override {
    lower_limit_ = lower_limit;
    upper_limit_ = upper_limit;
  }

  void SetDefaultGains(double, double) override {}

 private:
  uint8_t id_;
  double position_ = 0.0;
  double velocity_ = 0.0;
  double rated_torque_ = 0.0;
  double lower_limit_ = -M_PI;
  double upper_limit_ = M_PI;
};

}  // namespace

SimArm::SimArm() = default;

void SimArm::Init(const std::string& /*port*/, uint32_t /*baud*/) {
  // 与 6 轴 A-L1-GAMMA 配置及限位一致（见 config/arm_config.yaml）
  struct JointSpec {
    const char* name;
    double lower;
    double upper;
  };
  static const JointSpec kJoints[] = {
      {"joint1", -3.14, 3.14},
      {"joint2", -3.14, 3.14},
      {"joint3", -3.14, 3.14},
      {"joint4", -3.14, 3.14},
      {"joint5", -3.14, 3.14},
      {"joint6", -3.15, 3.15},
  };

  uint8_t id = 0;
  for (const auto& spec : kJoints) {
    auto motor = std::make_shared<SimMotor>(id++);
    motor->SetJointLimit(spec.lower, spec.upper);
    AddMotor(spec.name, motor);
  }

  SetBus(std::make_unique<SimBus>());
}

}  // namespace manipulator::arm
