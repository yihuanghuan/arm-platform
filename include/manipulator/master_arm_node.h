#pragma once

#include <memory>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <manipulator/arm/a_l1_beta.h>
#include <manipulator/gravity_compensation.h>

namespace manipulator {

class MasterArmNode : public rclcpp::Node {
 public:
  MasterArmNode();
  ~MasterArmNode();

 private:
  void ControlLoop();
  void ComputeAndPublishCompensation();
  void DebugInfoCallback();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  // rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_position_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_uav_calculate_compensation;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_compensation_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_uav_pose_;
  rclcpp::Subscription<dummy_interface::msg::MotorControl>::SharedPtr sub_uav_joint_currents;
  arm::AL1Beta& arm_;
  arm::AL1Beta::JointState arm_state_;
  std::array<double, 7> uav_joint_currents = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> uav_compensation_torques = {0, 0, 0, 0, 0, 0, 0};
  GravityCompensation gravity_compensation_;
  dummy_interface::msg::MotorControl cmd_;
};

} // namespace manipulator
