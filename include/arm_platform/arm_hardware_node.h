#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <dummy_interface/msg/motor_state.hpp>

#include <arm_platform/controller/i_arm_controller.h>

namespace arm_platform {

class ArmHardwareNode : public rclcpp::Node {
public:
    ArmHardwareNode();
    ~ArmHardwareNode();

    void UpdateCurrentState();


private:
    // Main control loop, runs periodically to send commands and publish state
    void InitCommand();
    void ControlLoop();

    // Callback for manual joint control commands
    void JointControlCallback(const dummy_interface::msg::MotorControl::SharedPtr msg);

    rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_state_;
    rclcpp::Subscription<dummy_interface::msg::MotorControl>::SharedPtr sub_joint_ctrl_;
    std::shared_ptr<arm_platform::controller::IArmController> controller_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Control thread and synchronization
    std::thread control_thread_;
    std::mutex mutex_;
    bool run_thread_{true};

    // Desired joint positions
    // std::vector<double> arm_target_pos_{0,0,0,0,0,0,0};
    JointCommand arm_target_cmd_;

    // Mapping between trajectory message joint names and internal indices
    std::vector<int> joint_indices_;
    JointStateArray arm_current_state_;

    uint8_t num_joints_;
};

} // namespace arm_platform
