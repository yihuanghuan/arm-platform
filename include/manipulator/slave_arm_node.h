#pragma once

#include <memory>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <manipulator/arm/a_l1_beta.h>
#include <manipulator/gravity_compensation.h>

namespace manipulator {

class SlaveArmNode : public rclcpp::Node {
 public:
  SlaveArmNode();
  ~SlaveArmNode();

 private:
  void ControlLoop();
  void ComputeAndPublishCompensation();
  void DebugInfoCallback();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_position_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_compensation_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_calculate_compentation_;
  arm::AL1Beta& arm_;
  arm::AL1Beta::JointState arm_state_;
  GravityCompensation gravity_compensation_;
  
};

} // namespace manipulator
