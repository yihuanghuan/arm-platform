#include <manipulator/slave_arm_node.h>
#include <manipulator/arm/arm_factory.h>
#include <chrono>

namespace manipulator {

SlaveArmNode::SlaveArmNode()
    : Node("slave_arm_node"),
      got_feedback_(false) {
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<bool>("debug_info", false);
  this->declare_parameter<double>("debug_rate", 1.0);
  this->declare_parameter<double>("MAX_TORQUE", 3.0);
  this->declare_parameter<double>("GRAVITY", 9.81);
  this->declare_parameter<double>("uav_roll", 0.0);
  this->declare_parameter<double>("uav_pitch", 0.0);
  this->declare_parameter<double>("uav_yaw", 0.0);
  this->declare_parameter<double>("arm_roll", 0.0);
  this->declare_parameter<double>("arm_pitch", 0.0);
  this->declare_parameter<double>("arm_yaw", 0.0);
  this->declare_parameter<double>("FORCE_FEEDBACK_THRESHOLD", 0.5);
  this->declare_parameter<double>("FORCE_FEEDBACK_GAIN", 0.5);
  this->declare_parameter<bool>("publish_joint_state", true);
  this->declare_parameter<bool>("publish_joint_feedback", false);
  this->declare_parameter<std::string>("urdf_path", "/home/iusl/huaben_ws/src/ti5-description/urdf/ARM_1KG_STD.urdf");
  this->declare_parameter<std::string>("arm_type", "a_l1_gamma");
  this->declare_parameter<std::vector<double>>("p_gain", {30, 30, 30, 5, 5, 5, 1});
  this->declare_parameter<std::vector<double>>("d_gain", {1, 1, 1, 0.1, 0.1, 0.1, 0.1});

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

  std::string arm_type;
  this->get_parameter("arm_type", arm_type);
  arm_ = arm::ArmFactory::Instance().Create(arm_type);
  arm_->Init(port, 921600);
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

  bool publish_joint_state = this->get_parameter("publish_joint_state").as_bool();
  bool publish_joint_feedback = this->get_parameter("publish_joint_feedback").as_bool();

  if (publish_joint_state) {
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }
  
  if (publish_joint_feedback) {
    pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  }

  cmd_.current.resize(7);
  cmd_.position.resize(7);
  cmd_.velocity.resize(7);

  std::vector<double> p_gain, d_gain;
  this->get_parameter("p_gain", p_gain);
  this->get_parameter("d_gain", d_gain);
  cmd_.p = p_gain;
  cmd_.d = d_gain;

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
  arm_state_ = arm_->GetJointStates();
  
  bool publish_joint_state = this->get_parameter("publish_joint_state").as_bool();
  bool publish_joint_feedback = this->get_parameter("publish_joint_feedback").as_bool();
  
  if (!got_feedback_) return;

  std::vector<double> cmd_pos(7, 0.0);
  for (int i = 0; i < 7; ++i) {
    cmd_.position[i] = ground_joint_positions_[i];
    cmd_.velocity[i] = ground_joint_velocities_[i];
  }
  cmd_.header.stamp = this->now();
  arm_->SetMotorCommand(cmd_);

  std::array<double, 7> joint_positions;
  for (int i = 0; i < 7; ++i) {
    joint_positions[i] = arm_state_.position[i];
  }

  auto tau_comp = gravity_compensation_.Compute(joint_positions);

  if (publish_joint_state) {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    for (int i = 0; i < 7; ++i) {
      joint_state_msg.effort.push_back(tau_comp[i]);
      joint_state_msg.velocity.push_back(arm_state_.current[i]);
      joint_state_msg.position.push_back(arm_state_.position[i]);
    }
    pub_joint_state_->publish(joint_state_msg);
    
  }

  if (publish_joint_feedback) {
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
