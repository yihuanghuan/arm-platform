//地面端
#include <manipulator/slave_arm_node.h>
#include <chrono>
#include <algorithm>
#include <manipulator/arm/arm_factory.h>

namespace manipulator {

SlaveArmNode::SlaveArmNode()
    : Node("slave_arm_node") {
  
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<std::string>("arm_type", "a_l1_beta");
  this->declare_parameter<std::string>("urdf_path", "/home/iusl/huaben_ws/src/ti5-description/urdf/ARM_1KG_STD.urdf");
  this->declare_parameter<double>("MAX_TORQUE", 3.0);
  this->declare_parameter<double>("GRAVITY", 9.81);
  this->declare_parameter<double>("uav_roll", 0.0);
  this->declare_parameter<double>("uav_pitch", 0.0);
  this->declare_parameter<double>("uav_yaw", 0.0);
  this->declare_parameter<double>("arm_roll", 0.0);
  this->declare_parameter<double>("arm_pitch", 0.0);
  this->declare_parameter<double>("arm_yaw", 0.0);
  this->declare_parameter<bool>("debug_info", false);
  this->declare_parameter<double>("debug_rate", 1.0);
  this->declare_parameter<double>("FORCE_FEEDBACK_THRESHOLD", 0.5);
  this->declare_parameter<double>("FORCE_FEEDBACK_GAIN", 0.5);
  this->declare_parameter<bool>("publish_joint_state", true);
  this->declare_parameter<bool>("publish_joint_feedback", false);

  std::string port;
  this->get_parameter("port_name", port);

  std::string urdf_path;
  this->get_parameter("urdf_path", urdf_path);
  if (!gravity_compensation_.LoadModel(urdf_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from: %s", urdf_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "URDF model loaded successfully from: %s", urdf_path.c_str());
  }

  double MAX_TORQUE = this->get_parameter("MAX_TORQUE").as_double();
  double GRAVITY = this->get_parameter("GRAVITY").as_double();
  double FORCE_FEEDBACK_THRESHOLD = this->get_parameter("FORCE_FEEDBACK_THRESHOLD").as_double();
  double FORCE_FEEDBACK_GAIN = this->get_parameter("FORCE_FEEDBACK_GAIN").as_double();
  gravity_compensation_.SetParams(MAX_TORQUE, GRAVITY, FORCE_FEEDBACK_THRESHOLD, FORCE_FEEDBACK_GAIN);

  double uav_roll = this->get_parameter("uav_roll").as_double();
  double uav_pitch = this->get_parameter("uav_pitch").as_double();
  double uav_yaw = this->get_parameter("uav_yaw").as_double();
  double arm_roll = this->get_parameter("arm_roll").as_double();
  double arm_pitch = this->get_parameter("arm_pitch").as_double();
  double arm_yaw = this->get_parameter("arm_yaw").as_double();
  geometry_msgs::msg::Point uav_pose;
  uav_pose.x = uav_yaw;
  uav_pose.y = uav_roll;
  uav_pose.z = uav_pitch;
  gravity_compensation_.SetUavPose(uav_pose);
  gravity_compensation_.SetRotationAngle(arm_roll, arm_pitch, arm_yaw);

  std::string arm_type;
  this->get_parameter("arm_type", arm_type);
  arm_ = arm::ArmFactory::Instance().Create(arm_type);
  arm_->Init(port, 921600);

  cmd_.current.resize(7);
  cmd_.p.resize(7);
  cmd_.velocity.resize(7);
  cmd_.d.resize(7);

  RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());

  bool publish_joint_state = this->get_parameter("publish_joint_state").as_bool();
  bool publish_joint_feedback = this->get_parameter("publish_joint_feedback").as_bool();


  //地面端只需要发布关节的实际位置
  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  // Create joint feedback publisher if enabled
  if (publish_joint_feedback) {
    pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  }

  sub_slave_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/slave/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { 
        // 处理接收到的计算补偿数据
        for(int i = 0; i < 7; i++) {
          uav_compensation_torques[i] = msg->effort[i];
          uav_joint_currents[i] = msg->velocity[i];
        }
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

  RCLCPP_INFO(this->get_logger(), "SlaveArmNode initialized (100Hz control loop)");
}

SlaveArmNode::~SlaveArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
}

void SlaveArmNode::ControlLoop() {
  arm_state_ = arm_->GetJointStates();
  ComputeAndPublishCompensation();
  
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.position = arm_state_.position;
  joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  joint_state_msg.effort = arm_state_.current;
  joint_state_msg.velocity = arm_state_.velocity;
  pub_joint_state_->publish(joint_state_msg);
  
  // Publish joint feedback message if enabled
  if (pub_joint_feedback_) {
    dummy_interface::msg::MotorState joint_feedback_msg;
    joint_feedback_msg.header.stamp = this->now();
    joint_feedback_msg.position = arm_state_.position;
    joint_feedback_msg.velocity = arm_state_.velocity;
    joint_feedback_msg.current = arm_state_.current;
    joint_feedback_msg.temperature = arm_state_.temperature;
    // Set voltage and temperature to empty arrays as we don't have this data
    pub_joint_feedback_->publish(joint_feedback_msg);
  }
  
}

void SlaveArmNode::ComputeAndPublishCompensation() {
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
  arm_->SetMotorCommand(cmd_);
}

void SlaveArmNode::DebugInfoCallback() {
  bool debug_info = this->get_parameter("debug_info").as_bool();
  if(not debug_info) return;
  RCLCPP_INFO(this->get_logger(), "Joint position (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
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
