//天空端
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
  //天空端需要订阅一个东西 1.地面端发布的关节实际位置
  sub_position_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/master/arm/joint_positions", 10,
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {  //这个函数把关节位置转发了，遥操作基本功能实现
        if (msg->data.size() >= 7) {
          std::array<double, 7> ground_joint_position;

          for (int i = 0; i < 7; ++i) {
            ground_joint_position[i] = msg->data[i];
          }
          dummy_interface::msg::MotorControl cmd;
          cmd.header.stamp = this->now();
          cmd.current.resize(7);
          for (int i = 0; i < 7; ++i) {
            cmd.position[i] = ground_joint_position[i];
            cmd.p[i] = 7.0;
            cmd.velocity[i] = 7.0;
            cmd.d[i] = 7.0;
          }

            arm_.SetMotorCommand(cmd);
        }
      });

  
  //天空端需要发布两个东西1.检测到的力矩 2.计算出的补偿力矩
  pub_joint_state_ = this->create_publisher<dummy_interface::msg::MotorState>("arm/joint_feedback", 10); //1
  pub_calculate_compentation_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/uav/arm/joint_compensation", 10); //2


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
  ComputeAndPublishCompensation();
  dummy_interface::msg::MotorState joint_state;
  joint_state.header.stamp = this->now();
  for (int i = 0; i < 7; ++i) {
    joint_state.position.push_back(arm_state_.position[i]);
    joint_state.current.push_back(arm_state_.current[i]);
  }

  pub_joint_state_->publish(joint_state);
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
    pub_calculate_compentation_->publish(tau_comp_msg);
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
