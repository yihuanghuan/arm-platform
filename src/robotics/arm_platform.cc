#include <algorithm>
#include <iostream>
#include <manipulator/robotics/arm_platform.h>

namespace manipulator {

ArmPlatform::ArmPlatform() = default;

ArmPlatform::~ArmPlatform() = default;

void ArmPlatform::SetArm(arm::AbsArm::UniPtr arm) {
  // 注入具体机械臂实现。
  // 上层节点可以传入 sim 虚拟臂，也可以传入 a_l1 真机实现；
  // ArmPlatform 后续只通过统一的 arm 接口读状态、发命令。
  arm_ = std::move(arm);
}

void ArmPlatform::SetController(controller::IArmController::UniPtr controller) {
  // 注入控制器。
  // 例如学生模式使用 SmoothPositionController 做限速位置控制；
  // master 模式可以使用 GravityController 输出重力补偿电流/力矩。
  controller_ = std::move(controller);
}

void ArmPlatform::SetPlanner(planning::AbsMotionPlanner::UniPtr planner) {
  // 注入轨迹规划器。
  // planner 的输出仍然是关节空间 setpoint，只是 setpoint 由轨迹生成器逐点产生，
  // 而不是像学生接口那样由外部 ROS topic 直接传入。
  planner_ = std::move(planner);

  // planner 一被设置就基于当前 arm_state_ 生成完整轨迹。
  // 注意：调用者需要保证此时 arm_state_ 已经有合理的当前关节状态。
  planner_->Plan(arm_state_);
}

void ArmPlatform::SetJointSetpoint(const planning::JointSetpoint& setpoint) {
  // 设置当前目标关节状态。
  // student_arm_node 收到 /student/joint_command 后会调用这里；
  // 后续 ExecuteControlCycle() 会把 joint_setpoint_ 交给 controller 计算电机命令。
  joint_setpoint_ = setpoint;
}

void ArmPlatform::SetCollisionAvoidance(collision::CollisionAvoidance::Ptr collision_avoidance) {
  // 注入碰撞检测器。
  // 它负责判断目标关节角对应的机械臂姿态是否靠近障碍物。
  collision_avoidance_ = collision_avoidance;
}

void ArmPlatform::EnableCollisionAvoidance(bool enable) {
  // 只控制是否启用碰撞检测；真正的检测对象由 SetCollisionAvoidance() 设置。
  collision_avoidance_enabled_ = enable;
}

void ArmPlatform::AddSubscribe(IArmDataSubscriber::SharedPtr subscriber) {
  // 注册数据订阅者。
  // 这里的 subscriber 通常是某个 ROS node，例如 StudentArmNode / SlaveArmNode。
  // ArmPlatform 不直接依赖 ROS publisher，而是通过 IArmDataSubscriber 回调把状态交出去。
  if (!subscriber) return;

  // 避免重复注册同一个 subscriber。
  auto it = std::find_if(subscribers_.begin(), subscribers_.end(),
    [&](const auto& w) {
        auto sp1 = w.lock();
        return sp1 && sp1 == subscriber;
    });

  if (it == subscribers_.end()) {
      subscribers_.push_back(subscriber);
  }
}

bool ArmPlatform::ExecuteControlCycle(double dt) {
  // ArmPlatform 的核心函数：执行一个控制周期。
  // 典型调用频率由外层 ROS timer 决定，例如 student_arm_node 是 100 Hz。
  //
  // 单周期主链路：
  //   1. 从 arm 读取当前关节状态
  //   2. 如果有 planner，则取下一帧轨迹点作为 joint_setpoint_
  //   3. 如果启用碰撞检测，则检查目标关节角是否有风险
  //   4. controller 根据当前状态和目标 setpoint 计算 MotorControl
  //   5. arm 下发 MotorControl
  //   6. 通知外部 subscriber 发布 joint state / motor feedback
  if (dt < 0.01) {
    throw std::runtime_error("Control cycle dt must be greater than 0.01");
  }
  if (!arm_) {
    throw std::runtime_error("Arm not set");
  }

  // 1. 读取机械臂当前状态。
  // sim 模式下来自虚拟臂内部状态；真机模式下来自电机反馈/协议解码结果。
  arm_state_ = arm_->GetJointStates();
  joint_names_ = arm_->GetJointNames();

  // 2. 如果设置了 planner，则本周期目标由 planner 产生。
  // 这会覆盖外部直接 SetJointSetpoint() 写入的目标。
  if (planner_) {
    joint_setpoint_ = planner_->GetTrajectoryPoint();
  }

  // 3. 碰撞检测只在显式启用且 collision_avoidance_ 存在时执行。
  // 当前策略比较保守：检测到风险就不下发新的控制命令。
  bool has_collision_risk = false;
  if (collision_avoidance_enabled_ && collision_avoidance_) {
    has_collision_risk = ApplyCollisionAvoidance();
  }

  // 4. 控制器把“当前状态 + 目标状态”转换成电机命令。
  // 不同 controller 可以输出不同类型的命令：
  //   SmoothPositionController 主要填 position/p/d；
  //   GravityController 主要填 current。
  if (controller_) {
    // 目标维度必须和机械臂当前关节数一致。
    // 如果没有目标 joint_setpoint_，q.size() 可以为 0，控制器自行决定如何处理。
    if (joint_setpoint_.q.size() > 0 and joint_setpoint_.q.size() != arm_state_.position.size()) {
      std::cout << "Joint setpoint size " << joint_setpoint_.q.size() 
        << " is not equal than joint position size " << arm_state_.position.size() << std::endl;
      return false;
    }

    if (not has_collision_risk) {
      // 5. 计算并下发命令。
      // cmd_ 的具体字段含义由底层 motor/protocol 解释。
      cmd_ = controller_->Compute(arm_state_, joint_setpoint_, dt);
      arm_->SetMotorCommand(cmd_);
    }

  } else {
    return false;
  }

  // 6. 把本周期读取到的状态通知给所有注册的订阅者。
  // 例如 StudentArmNode 会在 UpdateJointState() 里发布 /joint_states。
  NotifyJointState();
  NotifyMotorFeedback();

  // 返回值用于外层判断 planner 是否执行完毕。
  // 没有 planner 时通常一直返回 false。
  if(planner_ && planner_->IsDone()) {
    return true;
  }

  return false;
}

void ArmPlatform::NotifyJointState() {
  // 将内部 arm_state_ 转成标准 ROS JointState。
  // ArmPlatform 本身不创建 publisher，而是回调 subscribers_；
  // 这样核心控制层和 ROS 节点发布细节解耦。
  sensor_msgs::msg::JointState joint_state_msg;
  // joint_state_msg.header.stamp = rclcpp::Clock().now();
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.name.assign(joint_names_.begin(), joint_names_.end());
  joint_state_msg.position = arm_state_.position;
  joint_state_msg.velocity = arm_state_.velocity;

  // 这里把电流放到 effort 字段，方便使用标准 JointState 承载力/电流类反馈。
  joint_state_msg.effort = arm_state_.current;
  for (auto& subscriber : subscribers_) {
    auto shared_subscriber = subscriber.lock();
    if (shared_subscriber) {
      shared_subscriber->UpdateJointState(joint_state_msg);
    }
  }
}

void ArmPlatform::NotifyMotorFeedback() {
  // 发布更完整的电机级反馈，包括温度等 JointState 没有的字段。
  dummy_interface::msg::MotorState joint_feedback_msg;
  // joint_feedback_msg.header.stamp = rclcpp::Clock().now();
  joint_feedback_msg.position = arm_state_.position;
  joint_feedback_msg.velocity = arm_state_.velocity;
  joint_feedback_msg.current = arm_state_.current;
  joint_feedback_msg.temperature = arm_state_.temperature;
  for (auto& subscriber : subscribers_) {
    auto shared_subscriber = subscriber.lock();
    if (shared_subscriber) {
      shared_subscriber->UpdateMotorFeedback(joint_feedback_msg);
    }
  }
}

bool ArmPlatform::ApplyCollisionAvoidance() const {
  // 没有碰撞检测器，或当前没有目标关节角，则没有可检查对象。
  if (!collision_avoidance_ || joint_setpoint_.q.size() == 0) {
    return false;
  }

  // 当前实现里 current_q 预留给 AdjustTarget() 使用。
  // 目前检测到碰撞风险后采用“停止下发命令”的策略，暂不自动修改目标。
  Eigen::VectorXd current_q(arm_state_.position.size());
  for (size_t i = 0; i < arm_state_.position.size(); ++i) {
    current_q[i] = arm_state_.position[i];
  }

  // 检查的是目标姿态 target_q，而不是当前姿态。
  // 意思是：如果下一步目标会让机械臂靠近障碍物，就阻止这次控制命令。
  Eigen::VectorXd target_q = joint_setpoint_.q;

  collision::CollisionResult result = collision_avoidance_->CheckCollision(target_q);
  
  if (result.has_collision_risk) {
    // DO NOT MOVE when collision risk detected, consider adjusting target position later
    // Eigen::VectorXd adjusted_q = collision_avoidance_->AdjustTarget(current_q, target_q);
    // joint_setpoint_.q = adjusted_q;
    
    std::cout << "Collision risk detected! Min distance: " << result.min_distance 
              << ", adjusted target." << std::endl;
    return true;
  }
  
  return false;
}

void ArmPlatform::PrintDebugInfo() {
  // 给 ROS node 的 debug_timer 使用，周期性打印当前关节角和电流。
  std::cout << "Joint position (rad): [";
  for (size_t i = 0; i < arm_state_.position.size(); ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << arm_state_.position[i];
  }
  std::cout << "]" << std::endl;

  std::cout << "Joint currents (A): [";
  for (size_t i = 0; i < arm_state_.current.size(); ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << arm_state_.current[i];
  }
  std::cout << "]" << std::endl;
}

}  // namespace manipulator
