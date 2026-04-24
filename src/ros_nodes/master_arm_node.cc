#include <manipulator/ros_nodes/master_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/gravity_controller.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/planning/reset_motion_planner.h>
#include <manipulator/planning/trajectory/scurve_generator.h>

namespace manipulator {

MasterArmNode::MasterArmNode()
    : Node("master_arm_node") {
  bool debug_info = GetParam<bool>("debug_info", false);
  double debug_rate = GetParam<double>("debug_rate", 1.0);
  publish_joint_state_ = GetParam<bool>("publish_joint_state", true);
  publish_joint_feedback_ = GetParam<bool>("publish_joint_feedback", false);
  auto_reset_ = GetParam<bool>("auto_reset", true);
  
  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10); 
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  
  sub_slave_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/slave/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { 
      for (size_t i = 0; i < 7 && i < msg->effort.size(); ++i) {
        uav_compensation_torques[i] = msg->effort[i];
        uav_joint_currents[i] = msg->velocity[i];
      }
  });
  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();
  // set arm platform first then start control loop
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)),
      [this]() { return arm_platform_->ExecuteControlCycle(kControlPeriodMs); });

  if (debug_info) {
    auto debug_period = std::chrono::duration<double>(1.0 / debug_rate);
    debug_timer_ = this->create_wall_timer(
        debug_period,
        [this]() { return arm_platform_->PrintDebugInfo(); });
  }
  RCLCPP_INFO(this->get_logger(), "MasterArmNode initialized (100Hz control loop)");
}

void MasterArmNode::SetArmPlatform() {
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1_beta");
  std::string urdf_path = GetParam<std::string>("urdf_path", "");
  // Initialize gravity controller
  auto gravity_controller = std::make_unique<manipulator::controller::GravityController>();
  if (!gravity_controller->LoadModel(urdf_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from: %s", urdf_path.c_str());
  } 
  // Set gravity controller 
  double max_torque = GetParam<double>("MAX_TORQUE", 3.0);
  double gravity = GetParam<double>("GRAVITY", 9.81);
  double force_threshold = GetParam<double>("FORCE_FEEDBACK_THRESHOLD", 0.5);
  double force_gain = GetParam<double>("FORCE_FEEDBACK_GAIN", 0.5);
  gravity_controller->SetParams(max_torque, gravity, force_threshold, force_gain);
  arm_platform_->SetController(std::move(gravity_controller));
  // Set arm
  auto arm = arm::ArmFactory::Instance().Create(arm_type);
  arm->Init(port, 921600);
  RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  arm_platform_->SetArm(std::move(arm));
}

MasterArmNode::~MasterArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
  if (auto_reset_) {
    Reset();
  }
}

void MasterArmNode::Reset() {
  RCLCPP_INFO(this->get_logger(), "Resetting arm to home position...");
  auto controller = std::make_unique<controller::SmoothPositionController>();
  auto trajectory_generator = std::make_unique<planning::SCurveGenerator>();
  auto planner = std::make_unique<planning::ResetMotionPlanner>();
  planner->SetTrajectoryGenerator(std::move(trajectory_generator));

  arm_platform_->SetController(std::move(controller));
  arm_platform_->SetPlanner(std::move(planner));
  // when planner is done, exit the loop
  while (!arm_platform_->ExecuteControlCycle(kControlPeriodMs)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)));
  }
}

void MasterArmNode::UpdateJointState(const sensor_msgs::msg::JointState& msg) {
  pub_joint_state_->publish(msg);
}

void MasterArmNode::UpdateMotorFeedback(const dummy_interface::msg::MotorState& msg) {
  if (publish_joint_feedback_) {
    pub_joint_feedback_->publish(msg);
  }
}

void MasterArmNode::Init() {
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}
} // namespace manipulator


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::MasterArmNode>();
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
