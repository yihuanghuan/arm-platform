#include <manipulator/ros_nodes/student_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/common_types.h>

#include <algorithm>
#include <cmath>
#include <chrono>

namespace manipulator {

// 学生接口节点的控制周期：10 ms 一次，即 100 Hz。
// demo 脚本发布的 /student/joint_command 不会直接驱动电机，
// 而是在这个周期里被 ArmPlatform 读取、平滑、限速后再下发。
static constexpr double kStudentControlPeriodMs = 10.0;

StudentArmNode::StudentArmNode() : Node("student_arm_node") {
  // 读取 launch/参数文件传入的安全参数。
  // command_timeout_sec_: 超过多久没收到新指令后进入保持当前位置。
  // publish_joint_feedback_: 是否额外发布电机级反馈。
  // joint_lower/upper_limits_: 学生指令的关节限位兜底。
  command_timeout_sec_ = GetParam<double>("command_timeout_sec", 1.0);
  publish_joint_feedback_ = GetParam<bool>("publish_joint_feedback", false);
  joint_lower_limits_ = GetParam<std::vector<double>>(
      "joint_lower_limits", {-3.14, -3.14, -3.14, -3.14, -3.14, -3.15});
  joint_upper_limits_ = GetParam<std::vector<double>>(
      "joint_upper_limits", {3.14, 3.14, 3.14, 3.14, 3.14, 3.15});
  joint_count_ = joint_lower_limits_.size();

  // 对外发布当前关节状态。
  // /joint_states 会被 RViz / robot_state_publisher 使用，所以 demo 运行时模型能跟着动。
  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // 可选发布电机反馈：位置、速度、电流、温度等，默认关闭。
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>(
      "/student/joint_feedback", 10);

  // 学生侧唯一需要直接发布的控制入口。
  // demo 脚本最终都把目标关节角写到 sensor_msgs::msg::JointState.position，
  // 然后发到 /student/joint_command。
  sub_student_cmd_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/student/joint_command", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        StudentCommandCallback(msg);
      });

  // ArmPlatform 是底层控制调度器：
  // 它持有具体 arm 对象和 controller，并在 ExecuteControlCycle() 里完成
  // “读状态 -> 算命令 -> 下发 -> 发布反馈”的闭环。
  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();

  // 固定 100 Hz 调用 ControlLoop()，即使没有新命令也会持续更新状态和检查超时。
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kStudentControlPeriodMs)),
      [this]() { ControlLoop(); });

  RCLCPP_INFO(this->get_logger(),
              "StudentArmNode started. Publish sensor_msgs/JointState to "
              "/student/joint_command to control the arm.");
}

StudentArmNode::~StudentArmNode() {
  control_timer_->cancel();
}

void StudentArmNode::SetArmPlatform() {
  // 1. 根据参数创建具体机械臂对象。
  // arm_type="sim" 时使用虚拟臂；arm_type="a_l1" 时走真实硬件。
  // 真机模式通常需要 motor_config_path / arm_config_path 来初始化电机映射和限位。
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "sim");
  std::string arm_version = GetParam<std::string>("arm_version", "gamma");
  std::string motor_config_path = GetParam<std::string>("motor_config_path", "");
  std::string arm_config_path = GetParam<std::string>("arm_config_path", "");

  auto arm = arm::ArmFactory::Instance().Create(arm_type);
  if (!motor_config_path.empty() && !arm_config_path.empty()) {
    arm->InitFromConfig(port, 921600, motor_config_path, arm_config_path, arm_version);
    RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized from config", arm_type.c_str());
  } else {
    arm->Init(port, 921600);
    RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  }
  arm_platform_->SetArm(std::move(arm));

  // 2. 学生模式使用 SmoothPositionController。
  // 它不会让 setpoint 瞬间跳到目标角度，而是按 max_velocity 限速逼近目标，
  // 因此这里是“关节速度安全层”的主要实现位置。
  auto controller = std::make_unique<controller::SmoothPositionController>();
  // 学生模式默认限低速，保证安全
  double max_velocity = GetParam<double>("max_velocity", 0.5);
  controller->SetMaxVelocity(max_velocity);

  // 3. 设置底层位置控制的 P/D 参数。
  // 对 sim 来说主要表现为目标位置更新；对真机来说会进入电机命令。
  std::vector<double> p_gain = GetParam<std::vector<double>>(
      "p_gain", {30, 30, 30, 5, 5, 5});
  std::vector<double> d_gain = GetParam<std::vector<double>>(
      "d_gain", {1, 1, 1, 0.1, 0.1, 0.1});
  controller->SetKpKd(p_gain, d_gain);
  arm_platform_->SetController(std::move(controller));
}

void StudentArmNode::StudentCommandCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
  // 这是 /student/joint_command 的回调函数。
  // 输入消息只被当作“关节空间目标”处理：msg->position[i] 就是 joint(i+1) 的目标角度。
  // 这里不做 IK，也不理解末端位置；IK 已经在 Python demo 侧完成。

  // 安全检查 1：维度必须等于 joint_count_。
  // 少发/多发关节都会导致后续控制器和机械臂状态维度不匹配，所以直接拒收。
  if (msg->position.size() != joint_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Rejected command: expected %zu joint positions, got %zu",
        joint_count_, msg->position.size());
    return;
  }

  // 安全检查 2：拒收 NaN / Inf。
  // 这类非法数如果进入控制器，可能污染 setpoint 并持续传播到电机命令。
  for (double p : msg->position) {
    if (!std::isfinite(p)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Rejected command: position contains NaN/Inf");
      return;
    }
  }

  planning::JointSetpoint setpoint;
  setpoint.q.resize(joint_count_);
  setpoint.dq.resize(joint_count_);
  for (size_t i = 0; i < joint_count_; ++i) {
    // 安全检查 3：关节限位钳制。
    // 注意这里不是拒收超限指令，而是把它 clamp 到允许范围内继续执行。
    double q = std::clamp(msg->position[i], joint_lower_limits_[i], joint_upper_limits_[i]);
    if (q != msg->position[i]) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Joint %zu command %.3f clamped to [%.3f, %.3f]",
          i + 1, msg->position[i], joint_lower_limits_[i], joint_upper_limits_[i]);
    }
    setpoint.q[i] = q;

    // velocity 是可选前馈速度。
    // 如果 demo 没填，或者填了非法值，就默认为 0。
    setpoint.dq[i] = (i < msg->velocity.size() && std::isfinite(msg->velocity[i]))
                         ? msg->velocity[i] : 0.0;
  }

  // 把经过安全检查/钳制后的关节目标交给 ArmPlatform。
  // 真正的限速和平滑会在后续 ControlLoop -> ExecuteControlCycle -> controller.Compute 中完成。
  arm_platform_->SetJointSetpoint(setpoint);

  // 记录最近一次有效命令时间，用于 ControlLoop 中的超时保持逻辑。
  got_command_ = true;
  holding_ = false;
  last_command_stamp_ = now().seconds();
}

void StudentArmNode::ControlLoop() {
  // 这个函数每 10 ms 被 timer 调用一次，是学生节点的运行主循环。

  // 安全检查 4：指令超时后保持当前位置。
  // 这样 demo 程序退出、卡死或网络中断时，机械臂不会继续追一个过期目标。
  if (got_command_ && !holding_ &&
      now().seconds() - last_command_stamp_ > command_timeout_sec_) {
    if (last_known_position_.size() == joint_count_) {
      planning::JointSetpoint hold;
      hold.q.resize(joint_count_);
      hold.dq = Eigen::VectorXd::Zero(joint_count_);
      for (size_t i = 0; i < joint_count_; ++i) {
        // 用最近一次从 arm feedback 得到的真实/仿真关节角作为保持目标。
        hold.q[i] = last_known_position_[i];
      }
      arm_platform_->SetJointSetpoint(hold);
    }
    holding_ = true;
    RCLCPP_INFO(this->get_logger(), "Command timeout, holding current position");
  }

  // 执行一次底层控制闭环：
  //   arm_->GetJointStates()
  //   controller_->Compute(...)
  //   arm_->SetMotorCommand(...)
  //   NotifyJointState()/NotifyMotorFeedback()
  // 这里传入的 dt 是控制周期秒数。
  arm_platform_->ExecuteControlCycle(kStudentControlPeriodMs / 1000.0);
}

void StudentArmNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  // ArmPlatform 每个控制周期会通过 IArmDataSubscriber 回调到这里。
  // 保存 last_known_position_ 是为了超时保持；发布 /joint_states 是为了外部可视化/监听。
  last_known_position_ = msg.position;
  msg.header.stamp = this->now();
  pub_joint_state_->publish(msg);
}

void StudentArmNode::UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) {
  // 电机反馈默认不发布，避免学生模式下产生不必要的话题流量。
  // 启动时传 publish_joint_feedback:=true 后才会发布 /student/joint_feedback。
  if (!publish_joint_feedback_) {
    return;
  }
  msg.header.stamp = this->now();
  pub_joint_feedback_->publish(msg);
}

void StudentArmNode::Init() {
  // 把当前 ROS 节点注册为 ArmPlatform 的数据订阅者。
  // 这样 ArmPlatform 在 NotifyJointState()/NotifyMotorFeedback() 时，
  // 会回调本类的 UpdateJointState()/UpdateMotorFeedback()。
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}

}  // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::StudentArmNode>();
  // 构造函数里还不能安全调用 shared_from_this()，所以订阅者注册放在 Init()。
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
