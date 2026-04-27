#include <manipulator/ros_nodes/slave_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/planning/reset_motion_planner.h>
#include <manipulator/planning/trajectory/scurve_generator.h>
#include <manipulator/common_types.h>
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


  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);

  sub_master_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/master/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        MasterStateCallback(msg);
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

void SlaveArmNode::MasterStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
  planning::JointSetpoint joint_setpoint;
  size_t joint_num = msg->position.size();
  joint_setpoint.q.resize(joint_num);
  joint_setpoint.dq.resize(joint_num);
  for (size_t i = 0; i < joint_num; ++i) {
    joint_setpoint.q[i] = msg->position[i];
    joint_setpoint.dq[i] = msg->velocity[i];
  }
  arm_platform_->SetJointSetpoint(joint_setpoint);

  got_feedback_ = true;
}

void SlaveArmNode::SetArmPlatform() {  
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1_gamma");
  auto arm= arm::ArmFactory::Instance().Create(arm_type);
  arm->Init(port, 921600);
  arm_platform_->SetArm(std::move(arm));
  RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  auto smooth_position_controller = std::make_unique<controller::SmoothPositionController>();
  std::vector<double> p_gain = GetParam<std::vector<double>>("p_gain", {30, 30, 30, 5, 5, 5, 1});
  std::vector<double> d_gain = GetParam<std::vector<double>>("d_gain", {1, 1, 1, 0.1, 0.1, 0.1, 0.1});
  smooth_position_controller->SetKpKd(p_gain, d_gain);
  arm_platform_->SetController(std::move(smooth_position_controller));
}

SlaveArmNode::~SlaveArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
}

void SlaveArmNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
  pub_joint_state_->publish(msg);
}

void SlaveArmNode::UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
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
