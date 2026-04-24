#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <manipulator/robotics/arm/abs_arm.h>
#include <manipulator/controller/i_arm_controller.h>
#include <manipulator/planning/abs_motion_planner.h>

namespace manipulator {

static constexpr size_t kJointCount = 7;
static constexpr double kDefaultTimeout = 5.0;
static constexpr double kControlPeriodMs = 10.0;

static const std::array<std::string, kJointCount> kJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

class SlaveArmNode : public rclcpp::Node {
 public:
  SlaveArmNode();
  ~SlaveArmNode();

 private:
  template<typename T>
  T GetParam(const std::string& name, T default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    return this->get_parameter(name).get_value<T>();
  }

  void ControlLoop();
  void DebugInfoCallback();
  void PublishJointState();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_master_state_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_uav_pose_;
  
  arm::AbsArm::UniPtr arm_;
  arm::JointState arm_state_;
  
  controller::IArmController::UniPtr controller_;
  planning::AbsMotionPlanner::UniPtr planner_;

  bool got_feedback_ = false;
  dummy_interface::msg::MotorControl cmd_;
  std::array<double, kJointCount> master_joint_positions_ = {0};
  std::array<double, kJointCount> master_joint_velocities_ = {0};
  
};

} // namespace manipulator
