//地面端
#include <manipulator/master_arm_node.h>
#include <chrono>
#include <algorithm>

namespace manipulator {

MasterArmNode::MasterArmNode()
    : Node("master_arm_node"),
      arm_(arm::AL1Beta::Instance()) {
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<double>("G_GAIN_0", 0.5);
  this->declare_parameter<double>("G_GAIN_1", 0.5);
  this->declare_parameter<double>("G_GAIN_2", 1.0);
  this->declare_parameter<double>("MAX_TORQUE", 3.0);
  this->declare_parameter<double>("GRAVITY", 9.81);
  this->declare_parameter<double>("uav_roll", 0.0);
  this->declare_parameter<double>("uav_pitch", 0.0);
  this->declare_parameter<double>("uav_yaw", 0.0);
  this->declare_parameter<bool>("debug_info", false);
  this->declare_parameter<double>("debug_rate", 1.0);
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

  double uav_roll = this->get_parameter("uav_roll").as_double();
  double uav_pitch = this->get_parameter("uav_pitch").as_double();
  double uav_yaw = this->get_parameter("uav_yaw").as_double();
  geometry_msgs::msg::Point uav_pose;
  uav_pose.x = uav_yaw;
  uav_pose.y = uav_roll;
  uav_pose.z = uav_pitch;
  gravity_compensation_.SetUavPose(uav_pose);

  cmd_.current.resize(7);
  cmd_.position.resize(7);
  cmd_.p.resize(7);
  cmd_.velocity.resize(7);
  cmd_.d.resize(7);

  arm_.Init(port, 921600);

  //地面端只需要发布关节的实际位置
  pub_joint_position_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/master/arm/joint_positions", 10);
  // pub_joint_state_ = this->create_publisher<dummy_interface::msg::MotorState>("arm/joint_feedback", 10);
  // pub_joint_compensation_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/uav/arm/joint_compensation", 10);

  //地面端需要订阅天空端检测到的力矩和计算出的力矩
  sub_uav_joint_currents = this->create_subscription<dummy_interface::msg::MotorControl>(
      "/uav/arm/joint_currents", 10,
      [this](const  dummy_interface::msg::MotorControl::ConstSharedPtr& msg) { 
        for(int i = 0; i < 7; ++i) {
          uav_joint_currents[i] = msg->current[i];
        }
        // 处理接收到的关节电流数据
      });
  

  sub_uav_calculate_compensation = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/uav/arm/joint_compensation", 10,
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) { 
        // 处理接收到的计算补偿数据
        for(int i = 0; i < 7; i++)
          uav_compensation_torques[i] = msg->data[i];
      });

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

  RCLCPP_INFO(this->get_logger(), "MasterArmNode initialized (100Hz control loop)");
}

MasterArmNode::~MasterArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
}

void MasterArmNode::ControlLoop() {
  arm_.GetState(arm_state_);
  ComputeAndPublishCompensation();//零重力
}

void MasterArmNode::ComputeAndPublishCompensation() {
  std::array<double, 7> joint_positions;
  for (int i = 0; i < 7; ++i) {
    joint_positions[i] = arm_state_.position[i];
  }

  auto tau_comp = gravity_compensation_.Compute(joint_positions);
  // tau_comp = gravity_compensation_.collision_detection(tau_comp,uav_joint_currents,uav_compensation_torques); //力反馈 可以加个参数控制是否开启反馈
  std_msgs::msg::Float64MultiArray tau_comp_msg;
  cmd_.header.stamp = this->now();
  for (int i = 0; i < 7; ++i) {
    cmd_.current[i] = tau_comp[i];
    tau_comp_msg.data.push_back(tau_comp[i]);
  }

  arm_.SetMotorCommand(cmd_);

  // dummy_interface::msg::MotorState joint_state;
  // joint_state.header.stamp = this->now();
  // for (int i = 0; i < 7; ++i) {
  //   joint_state.position.push_back(arm_state_.position[i]);
  //   joint_state.current.push_back(arm_state_.current[i]);
  // }


//   pub_joint_state_->publish(joint_state);
//   pub_joint_compensation_->publish(tau_comp_msg);
}

void MasterArmNode::DebugInfoCallback() {
  bool debug_info = this->get_parameter("debug_info").as_bool();
  if(not debug_info) return;
  RCLCPP_INFO(this->get_logger(), "Joint currents (A): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.current[0], arm_state_.current[1], arm_state_.current[2],
              arm_state_.current[3], arm_state_.current[4], arm_state_.current[5], arm_state_.current[6]);
}

} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::MasterArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
