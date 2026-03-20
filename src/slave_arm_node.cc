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

  sub_position_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/master/arm/joint_positions", 10,
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {
        if (msg->data.size() >= 7) {
          for (int i = 0; i < 7; ++i) {
            ground_joint_positions_[i] = msg->data[i];
          }
          got_feedback_ = true;
        }
      });

  sub_uav_pose_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/uav/pose", 10,
      [this](const geometry_msgs::msg::Point::ConstSharedPtr& msg) {
        gravity_compensation_.SetUavPose(*msg);
      });

  pub_joint_state_ = this->create_publisher<dummy_interface::msg::MotorState>("/uav/arm/joint_feedback", 10);
  pub_calculate_compensation_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/uav/arm/joint_compensation", 10);
  pub_joint_controller_ = this->create_publisher<dummy_interface::msg::MotorControl>("/uav/arm/joint_controller", 10);
  pub_joint_currents_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/uav/arm/joint_currents", 10);

  cmd_.current.resize(7);
  cmd_.position.resize(7);
  cmd_.p = {20, 10, 10, 5, 1, 1, 1};
  cmd_.velocity = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
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
  arm_.GetState(arm_state_);
  
  if (!got_feedback_) return;

  std::vector<double> cmd_pos(7, 0.0);
  for (int i = 0; i < 7; ++i) {
    cmd_pos[i] = ground_joint_positions_[i];
  }
  cmd_.position = cmd_pos;
  cmd_.header.stamp = this->now();
  pub_joint_controller_->publish(cmd_);

  ComputeAndPublishCompensation();
  arm_.SetMotorCommand(cmd_);

  std_msgs::msg::Float64MultiArray joint_currents_msg;
  for (int i = 0; i < 7; ++i) {
    joint_currents_msg.data.push_back(arm_state_.current[i]);
  }
  pub_joint_currents_->publish(joint_currents_msg);
}

void SlaveArmNode::ComputeAndPublishCompensation() {
  std::array<double, 7> joint_positions;
  for (int i = 0; i < 7; ++i) {
    joint_positions[i] = arm_state_.position[i];
  }

  auto tau_comp = gravity_compensation_.Compute(joint_positions);

  std_msgs::msg::Float64MultiArray tau_comp_msg;
  for (int i = 0; i < 7; ++i) {
    tau_comp_msg.data.push_back(tau_comp[i]);
  }
  pub_calculate_compensation_->publish(tau_comp_msg);
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
  RCLCPP_INFO(this->get_logger(), "Ground joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              ground_joint_positions_[0], ground_joint_positions_[1], ground_joint_positions_[2],
              ground_joint_positions_[3], ground_joint_positions_[4], ground_joint_positions_[5], ground_joint_positions_[6]);
}

} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::SlaveArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
