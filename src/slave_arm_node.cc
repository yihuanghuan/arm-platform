#include <manipulator/slave_arm_node.h>
#include <chrono>

namespace manipulator {

SlaveArmNode::SlaveArmNode()
    : Node("slave_arm_node"),
      arm_(arm::AL1Beta::Instance()),
      got_feedback_(false) {
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<bool>("debug_info", false);
  this->declare_parameter<double>("debug_rate", 1.0);
  this->declare_parameter<double>("G_GAIN_0", 0.5);
  this->declare_parameter<double>("G_GAIN_1", 0.5);
  this->declare_parameter<double>("G_GAIN_2", 1.0);
  this->declare_parameter<double>("MAX_TORQUE", 3.0);
  this->declare_parameter<double>("GRAVITY", 9.81);
  this->declare_parameter<double>("FORCE_FEEDBACK_THRESHOLD", 0.5);
  this->declare_parameter<double>("FORCE_FEEDBACK_GAIN", 0.5);
  this->declare_parameter<bool>("publish_joint_states", true);

  std::string port;
  this->get_parameter("port_name", port);

  double G_GAIN_0 = this->get_parameter("G_GAIN_0").as_double();
  double G_GAIN_1 = this->get_parameter("G_GAIN_1").as_double();
  double G_GAIN_2 = this->get_parameter("G_GAIN_2").as_double();
  double MAX_TORQUE = this->get_parameter("MAX_TORQUE").as_double();
  double GRAVITY = this->get_parameter("GRAVITY").as_double();
  double FORCE_FEEDBACK_THRESHOLD = this->get_parameter("FORCE_FEEDBACK_THRESHOLD").as_double();
  double FORCE_FEEDBACK_GAIN = this->get_parameter("FORCE_FEEDBACK_GAIN").as_double();
  gravity_compensation_.SetParams(G_GAIN_0, G_GAIN_1, G_GAIN_2, MAX_TORQUE, GRAVITY, FORCE_FEEDBACK_THRESHOLD, FORCE_FEEDBACK_GAIN);

  arm_.Init(port, 921600);

  sub_master_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/master/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        if (msg->position.size() >= 7 and msg->velocity.size() >= 7) {
          for (int i = 0; i < 7; ++i) {
            ground_joint_positions_[i] = msg->position[i];
            ground_joint_velocities_[i] = msg->velocity[i];
          }
          got_feedback_ = true;
        }
      });

  sub_uav_pose_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/uav/pose", 10,
      [this](const geometry_msgs::msg::Point::ConstSharedPtr& msg) {
        gravity_compensation_.SetUavPose(*msg);
      });

  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  cmd_.current.resize(7);
  cmd_.position.resize(7);
  cmd_.velocity.resize(7);
  cmd_.p = {10, 10, 10, 5, 1, 1, 1};
  cmd_.d = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

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

  RCLCPP_INFO(this->get_logger(), "SlaveArmNode initialized (controlled mode, 100Hz control loop)");
}

SlaveArmNode::~SlaveArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
}

void SlaveArmNode::ControlLoop() {
  arm_state_ = arm_.GetJointStates();
  
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

  
  if (!got_feedback_) return;

  std::vector<double> cmd_pos(7, 0.0);
  for (int i = 0; i < 7; ++i) {
    cmd_.position[i] = ground_joint_positions_[i];
    cmd_.velocity[i] = ground_joint_velocities_[i];
  }
  cmd_.header.stamp = this->now();
  arm_.SetMotorCommand(cmd_);

  std::array<double, 7> joint_positions;
  for (int i = 0; i < 7; ++i) {
    joint_positions[i] = arm_state_.position[i];
  }

  auto tau_comp = gravity_compensation_.Compute(joint_positions);
  for (int i = 0; i < 7; ++i) {
    joint_state_msg.effort.push_back(tau_comp[i]);
    joint_state_msg.velocity.push_back(arm_state_.current[i]);
    joint_state_msg.position.push_back(arm_state_.position[i]);
  }
  pub_joint_state_->publish(joint_state_msg);

}

void SlaveArmNode::DebugInfoCallback() {
  bool debug_info = this->get_parameter("debug_info").as_bool();
  if (!debug_info) {
    return;
  }

  // RCLCPP_INFO(this->get_logger(), "Joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
  //             arm_state_.position[0], arm_state_.position[1], arm_state_.position[2],
  //             arm_state_.position[3], arm_state_.position[4], arm_state_.position[5], arm_state_.position[6]);
  // RCLCPP_INFO(this->get_logger(), "Joint currents (A): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
  //             arm_state_.current[0], arm_state_.current[1], arm_state_.current[2],
  //             arm_state_.current[3], arm_state_.current[4], arm_state_.current[5], arm_state_.current[6]);
  // RCLCPP_INFO(this->get_logger(), "Ground joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
  //             ground_joint_positions_[0], ground_joint_positions_[1], ground_joint_positions_[2],
  //             ground_joint_positions_[3], ground_joint_positions_[4], ground_joint_positions_[5], ground_joint_positions_[6]);
}

} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::SlaveArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
