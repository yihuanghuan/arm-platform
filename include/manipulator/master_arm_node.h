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
#include <manipulator/gravity_compensation_pinocchio.h>

namespace manipulator {

/**
 * @brief Master arm control node for ground station
 * 
 * This node controls the master arm with gravity compensation
 * and publishes joint states. It subscribes to slave arm state
 * and UAV pose for force feedback control.
 */
class MasterArmNode : public rclcpp::Node {
 public:
  MasterArmNode();
  ~MasterArmNode();

 private:
  /**
   * @brief Main control loop running at high frequency
   * 
   * This function is called periodically by the control timer
   * to update arm state and compute compensation.
   */
  void ControlLoop();

  /**
   * @brief Compute and publish gravity compensation torques
   * 
   * Computes gravity compensation based on current joint positions
   * and applies collision detection if enabled.
   */
  void ComputeAndPublishCompensation();

  /**
   * @brief Debug information callback
   * 
   * Periodically prints debug information if debug mode is enabled.
   */
  void DebugInfoCallback();

  // ROS 2 timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_compensation_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_uav_pose_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_slave_state_;

  // Hardware interface
  arm::AL1Beta& arm_;
  arm::AL1Beta::JointState arm_state_;

  // Force feedback data from slave arm
  std::array<double, 7> uav_joint_currents = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> uav_compensation_torques = {0, 0, 0, 0, 0, 0, 0};

  // Gravity compensation algorithm
  GravityCompensationPinocchio gravity_compensation_;

  // Motor control command
  dummy_interface::msg::MotorControl cmd_;
};

} // namespace manipulator
