#pragma once
#include <memory>
#include <sensor_msgs/msg/joint_state.hpp>

namespace manipulator::motor {

class IMotor {
 public:
  using SharedPtr = std::shared_ptr<IMotor>;

  virtual ~IMotor() = default;

  virtual void UpdateState() = 0;
  virtual void UpdateCommand(const sensor_msgs::msg::JointState& state) = 0;
};

} // namespace manipulator::motor