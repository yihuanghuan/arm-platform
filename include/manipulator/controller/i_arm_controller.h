#pragma once
#include <memory>
#include <dummy_interface/msg/motor_control.hpp>
#include <manipulator/common_types.h>

namespace manipulator::controller {
    
using JointStates = manipulator::arm::JointState;
using JointCommand = dummy_interface::msg::MotorControl;
using JointSetPoint = manipulator::planning::JointSetPoint;
class IArmController {
 public:
  using UniPtr = std::unique_ptr<IArmController>;
  IArmController() = default;
  virtual ~IArmController() = default;
  
  virtual JointCommand Compute(
      const JointStates& joint_states,
      const JointSetPoint& joint_setpoint,
      double dt
      ) = 0;

};
} // namespace manipulator::controller
