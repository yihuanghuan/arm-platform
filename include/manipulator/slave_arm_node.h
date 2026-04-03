#pragma once

#include <memory>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <manipulator/arm/abs_arm.h>
#include <manipulator/gravity_compensation.h>

namespace manipulator {

class SlaveArmNode : public rclcpp::Node {
 public:
  SlaveArmNode();
  ~SlaveArmNode();

 private:
  void ControlLoop();
  void DebugInfoCallback();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_calculate_compensation_;
  // rclcpp::Publisher<dummy_interface::msg::MotorControl>::SharedPtr pub_joint_controller_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_currents_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_master_state_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_uav_pose_;
  
  arm::AbsArm::UniPtr arm_;
  arm::AbsArm::JointState arm_state_;
  GravityCompensation gravity_compensation_;
  
  bool got_feedback_;
  dummy_interface::msg::MotorControl cmd_;
  std::array<double, 7> ground_joint_positions_;
  std::array<double, 7> ground_joint_velocities_;
  
};

} // namespace manipulator
