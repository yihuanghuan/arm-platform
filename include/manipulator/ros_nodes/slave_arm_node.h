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
#include <manipulator/robotics/arm_data_subscriber.h>
#include <manipulator/robotics/arm_platform.h>

namespace manipulator {

static constexpr double kControlPeriodMs = 10.0;

class SlaveArmNode : public rclcpp::Node, public IArmDataSubscriber {
 public:
  SlaveArmNode();
  ~SlaveArmNode();
  void Init();

  void UpdateJointState(sensor_msgs::msg::JointState& msg) override;
  void UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) override;

 private:
  template<typename T>
  T GetParam(const std::string& name, T default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    return this->get_parameter(name).get_value<T>();
  }

  // void DebugInfoCallback();
  void SetArmPlatform();
  void MasterStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_master_state_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_uav_pose_;
  
  ArmPlatform::UniPtr arm_platform_;

  bool got_feedback_ = false;
  bool publish_joint_feedback_ = false;
  bool publish_joint_state_ = false;
  dummy_interface::msg::MotorControl cmd_;
};

} // namespace manipulator
