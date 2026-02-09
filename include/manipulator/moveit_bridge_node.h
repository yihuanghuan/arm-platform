#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>

namespace arm_platform {

class MoveItBridgeNode : public rclcpp::Node {
public:
    MoveItBridgeNode();
    ~MoveItBridgeNode();

private:
    // Callback for trajectory commands from MoveIt2
    void JoinStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS2 publishers
    rclcpp::Publisher<dummy_interface::msg::MotorControl>::SharedPtr pub_joint_ctrl_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;


    // Mapping between trajectory message joint names and internal indices
    std::vector<int> joint_indices_;
    std::vector<std::string> ordered_joint_names_;
};

} // namespace arm_platform::node
