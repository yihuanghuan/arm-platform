#include <manipulator/ros_nodes/master_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/gravity_controller.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/planning/reset_motion_planner.h>
#include <manipulator/planning/trajectory/scurve_generator.h>

namespace manipulator {

MasterArmNode::MasterArmNode()
    : Node("master_arm_node") {
  
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1_beta");
  std::string urdf_path = GetParam<std::string>("urdf_path", "");
  double max_torque = GetParam<double>("MAX_TORQUE", 3.0);
  double gravity = GetParam<double>("GRAVITY", 9.81);
  double force_threshold = GetParam<double>("FORCE_FEEDBACK_THRESHOLD", 0.5);
  double force_gain = GetParam<double>("FORCE_FEEDBACK_GAIN", 0.5);
  bool debug_info = GetParam<bool>("debug_info", false);
  double debug_rate = GetParam<double>("debug_rate", 1.0);
  bool publish_joint_state = GetParam<bool>("publish_joint_state", true);
  bool publish_joint_feedback = GetParam<bool>("publish_joint_feedback", false);
  bool auto_reset = GetParam<bool>("auto_reset", true);
  auto_reset_ = auto_reset;
  
  // Initialize gravity controller
  auto gravity_controller = std::make_unique<manipulator::controller::GravityController>();
  if (!gravity_controller->LoadModel(urdf_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from: %s", urdf_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "URDF model loaded successfully from: %s", urdf_path.c_str());
  }
  gravity_controller->SetParams(max_torque, gravity, force_threshold, force_gain);

  // Initialize arm
  arm_ = arm::ArmFactory::Instance().Create(arm_type);
  arm_->Init(port, 921600);
  RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());

  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  if (publish_joint_feedback) {
    pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  }

  sub_slave_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/slave/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { 
        for (size_t i = 0; i < kJointCount && i < msg->effort.size(); ++i) {
          uav_compensation_torques[i] = msg->effort[i];
          uav_joint_currents[i] = msg->velocity[i];
        }
      });

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)),
      [this]() { return ControlLoop(); });

  if (debug_info) {
    auto debug_period = std::chrono::duration<double>(1.0 / debug_rate);
    debug_timer_ = this->create_wall_timer(
        debug_period,
        [this]() { return DebugInfoCallback(); });
  }

  controller_ = std::move(gravity_controller);

  RCLCPP_INFO(this->get_logger(), "MasterArmNode initialized (100Hz control loop)");
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
  controller_ = std::make_unique<controller::SmoothPositionController>();
  auto trajectory_generator = std::make_unique<planning::SCurveGenerator>();
  planner_ = std::make_unique<planning::ResetMotionPlanner>();
  planner_->SetTrajectoryGenerator(std::move(trajectory_generator));
  planner_->Plan(arm_state_);
  while (!planner_->IsDone()) {
    ControlLoop();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)));
  }
}

void MasterArmNode::ControlLoop() {
  arm_state_ = arm_->GetJointStates();

  controller::JointSetPoint joint_setpoint;

  if(planner_) {
    joint_setpoint = planner_->GetTrajectoryPoint();
  }
  if (controller_) {
    cmd_ = controller_->Compute(arm_state_, joint_setpoint, kControlPeriodMs);
    cmd_.header.stamp = this->now();
    arm_->SetMotorCommand(cmd_);
  } else {
    return;
  }
  PublishJointState();
}

void MasterArmNode::DebugInfoCallback() {
  if (!GetParam<bool>("debug_info", false)) return;
  
  RCLCPP_INFO(this->get_logger(), 
              "Joint position (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.position[0], arm_state_.position[1], arm_state_.position[2],
              arm_state_.position[3], arm_state_.position[4], arm_state_.position[5], arm_state_.position[6]);
  RCLCPP_INFO(this->get_logger(), 
              "Joint currents (A): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.current[0], arm_state_.current[1], arm_state_.current[2],
              arm_state_.current[3], arm_state_.current[4], arm_state_.current[5], arm_state_.current[6]);
}

void MasterArmNode::PublishJointState() {
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.name.assign(kJointNames.begin(), kJointNames.end());
  joint_state_msg.position = arm_state_.position;
  joint_state_msg.velocity = arm_state_.velocity;
  joint_state_msg.effort = arm_state_.current;
  pub_joint_state_->publish(joint_state_msg);
  
  if (pub_joint_feedback_) {
    dummy_interface::msg::MotorState joint_feedback_msg;
    joint_feedback_msg.header.stamp = this->now();
    joint_feedback_msg.position = arm_state_.position;
    joint_feedback_msg.velocity = arm_state_.velocity;
    joint_feedback_msg.current = arm_state_.current;
    joint_feedback_msg.temperature = arm_state_.temperature;
    pub_joint_feedback_->publish(joint_feedback_msg);
  }
}

} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::MasterArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
