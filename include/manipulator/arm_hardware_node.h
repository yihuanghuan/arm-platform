#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <manipulator/arm/abs_arm.h>
#include <dummy_interface/msg/motor_state.hpp>

namespace manipulator {

class ArmHardwareNode : public rclcpp::Node {
 public:
  ArmHardwareNode();
  ~ArmHardwareNode();

  void UpdateCurrentState();

 private:
  // Main control loop, runs periodically to send commands and publish state
  void ControlLoop();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_ctrl_;
  arm::AbsArm::UniPtr arm_;

  arm::AbsArm::JointState arm_current_state_;
};

} // namespace manipulator
