#include <manipulator/ros_nodes/slave_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/planning/reset_motion_planner.h>
#include <manipulator/planning/trajectory/scurve_generator.h>
#include <chrono>
#include <thread>

namespace manipulator {

SlaveArmNode::SlaveArmNode()
    : Node("slave_arm_node"),
      got_feedback_(false) {
  
  bool debug_info = GetParam<bool>("debug_info", false);
  double debug_rate = GetParam<double>("debug_rate", 1.0);
  bool publish_joint_state = GetParam<bool>("publish_joint_state", true);
  bool publish_joint_feedback = GetParam<bool>("publish_joint_feedback", false);
  
  std::vector<double> p_gain = GetParam<std::vector<double>>("p_gain", {30, 30, 30, 5, 5, 5, 1});
  std::vector<double> d_gain = GetParam<std::vector<double>>("d_gain", {1, 1, 1, 0.1, 0.1, 0.1, 0.1});

  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  

  sub_master_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/master/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        if (msg->position.size() >= 7 && msg->velocity.size() >= 7) {
          for (size_t i = 0; i < 7; ++i) {
            master_joint_positions_[i] = msg->position[i];
            master_joint_velocities_[i] = msg->velocity[i];
          }
          got_feedback_ = true;
        }
      });

  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)),
      [this]() { return arm_platform_->ExecuteControlCycle(kControlPeriodMs); });

  if (debug_info) {
    auto debug_period = std::chrono::duration<double>(1.0 / debug_rate);
    debug_timer_ = this->create_wall_timer(
        debug_period,
        [this]() { return arm_platform_->PrintDebugInfo(); });
  }

  RCLCPP_INFO(this->get_logger(), "SlaveArmNode initialized (controlled mode, 100Hz control loop)");
}

void SlaveArmNode::SetArmPlatform() {  
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1_gamma");
  auto arm= arm::ArmFactory::Instance().Create(arm_type);
  arm->Init(port, 921600);
  arm_platform_->SetArm(std::move(arm));
  RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  auto smooth_position_controller = std::make_unique<controller::SmoothPositionController>();
  auto controller = std::move(smooth_position_controller);
  arm_platform_->SetController(std::move(controller));
}


SlaveArmNode::~SlaveArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
}

void SlaveArmNode::UpdateJointState(const sensor_msgs::msg::JointState& msg) {
  pub_joint_state_->publish(msg);
}

void SlaveArmNode::UpdateMotorFeedback(const dummy_interface::msg::MotorState& msg) {
  pub_joint_feedback_->publish(msg);
}

void SlaveArmNode::Init() {
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}
} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::SlaveArmNode>();
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
