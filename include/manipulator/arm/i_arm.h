#pragma once
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>

namespace manipulator::arm {

struct JointLimit {
  double min_position;
  double max_position;
  double max_velocity;
  double max_effort;
};

class IArm {
 public:
  virtual ~IArm() = default;

  // Access joint states
  virtual bool SetJointStates(const sensor_msgs::msg::JointState& states) = 0;
  virtual void GetJointStates();
};

} // namespace manipulator::arm