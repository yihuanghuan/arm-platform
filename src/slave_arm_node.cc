#include <manipulator/slave_arm_node.h>
#include <chrono>

namespace manipulator {

SlaveArmNode::SlaveArmNode()
    : Node("slave_arm_node"),
      arm_(arm::AL1Beta::Instance()) {
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<bool>("debug_info", false);
  this->declare_parameter<double>("debug_rate", 1.0);

  std::string port;
  this->get_parameter("port_name", port);

  arm_.Init(port, 921600);

  sub_compensation_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/master/arm/joint_compensation", 10,
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) { 
        if (msg->data.size() >= 7) {
          std::array<double, 7> tau_comp;
          for (int i = 0; i < 7; ++i) {
            tau_comp[i] = msg->data[i];
          }
          gravity_compensation_.SetUavPose(geometry_msgs::msg::Point());
          auto computed_tau = gravity_compensation_.Compute(tau_comp);
        }
      });

  pub_joint_state_ = this->create_publisher<dummy_interface::msg::MotorState>("arm/joint_feedback", 10);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { return ControlLoop(); });

  bool debug_info = this->get_parameter("debug_info").as_bool();
  if (debug_info) {
    double debug_rate = this->get_parameter("debug_rate").as_double();
    auto debug_period = std::chrono::duration<double>(1.0 / debug_rate);
    debug_timer_ = this->create_wall_timer(
        debug_period,
        [this]() { return DebugInfoCallback(); });
  }

  RCLCPP_INFO(this->get_logger(), "SlaveArmNode initialized (passive mode, 100Hz state read)");
}

SlaveArmNode::~SlaveArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
}

void SlaveArmNode::ControlLoop() {
  arm_.GetState(arm_state_);

  dummy_interface::msg::MotorState joint_state;
  joint_state.header.stamp = this->now();
  for (int i = 0; i < 7; ++i) {
    joint_state.position.push_back(arm_state_.position[i]);
    joint_state.current.push_back(arm_state_.current[i]);
  }

  pub_joint_state_->publish(joint_state);
}

void SlaveArmNode::DebugInfoCallback() {
  bool debug_info = this->get_parameter("debug_info").as_bool();
  if (!debug_info) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.position[0], arm_state_.position[1], arm_state_.position[2],
              arm_state_.position[3], arm_state_.position[4], arm_state_.position[5], arm_state_.position[6]);
  RCLCPP_INFO(this->get_logger(), "Joint currents (A): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.current[0], arm_state_.current[1], arm_state_.current[2],
              arm_state_.current[3], arm_state_.current[4], arm_state_.current[5], arm_state_.current[6]);
}

} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::SlaveArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
