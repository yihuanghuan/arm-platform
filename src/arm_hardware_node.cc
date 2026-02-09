#include <manipulator/arm_hardware_node.h>

using namespace std::chrono_literals;

namespace manipulator {

ArmHardwareNode::ArmHardwareNode() 
 : Node("robot_arm_node"), arm_(arm::AL1Beta::Instance()) {
  // ------------------- Hardware stack -------------------
  std::string port;
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->get_parameter("port_name", port);

  arm_.Init(port, 921600);

  // ------------------- ROS interfaces -------------------
  pub_joint_state_ = this->create_publisher<dummy_interface::msg::MotorState>(
      "arm/joint_feedback", 10);

  sub_joint_ctrl_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "arm/joint_control", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      arm_.SetJointStates(*msg);
    });

  // ------------------- Control loop timer -------------------
  control_timer_ = this->create_wall_timer(
      5ms,   // 200Hz
      std::bind(&ArmHardwareNode::ControlLoop, this));

  RCLCPP_INFO(this->get_logger(), "ArmHardwareNode initialized (200Hz control loop)");
}

ArmHardwareNode::~ArmHardwareNode() = default;

// --------------------------------------------------------
// 200Hz real-time control loop
// --------------------------------------------------------
void ArmHardwareNode::ControlLoop() {
  arm_.GetState(arm_current_state_);

  dummy_interface::msg::MotorState msg;
  msg.header.stamp = this->now();

  for (size_t i = 0; i < arm_current_state_.position.size(); ++i) {
      msg.position.push_back(arm_current_state_.position[i]);
      msg.velocity.push_back(arm_current_state_.velocity[i]);
      msg.current.push_back(arm_current_state_.current[i]);
      msg.voltage.push_back(arm_current_state_.voltage[i]);
      msg.temperature.push_back(arm_current_state_.temperature[i]);
  }

  pub_joint_state_->publish(msg);
}
} // namespace manipulator

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<manipulator::ArmHardwareNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
