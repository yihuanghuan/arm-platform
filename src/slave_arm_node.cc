//地面端
#include <manipulator/slave_arm_node.h>
#include <chrono>
#include <algorithm>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <limits>

#include <manipulator/arm/arm_factory.h>

#include <Eigen/Dense>

// ===== Pinocchio =====
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/log.hpp>

namespace manipulator {

namespace {

// ==============================
// 工具函数：角度归一化、防跳变
// ==============================

inline double NormalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double Clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

// 把 target 映射到与 reference 最接近的等价角，再归一化到 [-pi, pi]
inline double NearestEquivalentAngle(double target, double reference) {
  target = NormalizeAngle(target);
  reference = NormalizeAngle(reference);

  double best = target;
  double min_dist = std::abs(target - reference);

  for (int k = -2; k <= 2; ++k) {
    double candidate = target + 2.0 * M_PI * static_cast<double>(k);
    double dist = std::abs(candidate - reference);
    if (dist < min_dist) {
      min_dist = dist;
      best = candidate;
    }
  }

  return NormalizeAngle(best);
}

// ==============================
// IK 返回结构体
// ==============================
struct IkResult6Dof {
  bool success{false};
  std::array<double, 6> q{{0., 0., 0., 0., 0., 0.}};
  int iterations{0};
  double final_error{0.0};
  std::string message;
};

// ==============================
// 文件内单独 IK 求解器
// ==============================
class PinocchioIKSolver6Dof {
 public:
  bool LoadModel(const std::string& urdf_path,
                 const std::vector<std::string>& active_joint_names,
                 const std::string& ee_frame) {
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);
    } catch (const std::exception& e) {
      last_error_ = std::string("Pinocchio buildModel failed: ") + e.what();
      return false;
    }

    if (!model_.existFrame(ee_frame)) {
      last_error_ = "End-effector frame not found in URDF: " + ee_frame;
      return false;
    }
    ee_frame_id_ = model_.getFrameId(ee_frame);

    active_joint_names_ = active_joint_names;
    active_joint_ids_.clear();
    active_q_indices_.clear();
    active_v_indices_.clear();

    for (const auto& joint_name : active_joint_names_) {
      if (!model_.existJointName(joint_name)) {
        last_error_ = "Active joint not found in URDF: " + joint_name;
        return false;
      }

      pinocchio::JointIndex jid = model_.getJointId(joint_name);
      const auto& jmodel = model_.joints[jid];

      if (jmodel.nq() != 1 || jmodel.nv() != 1) {
        last_error_ =
            "Joint is not 1-DoF, unsupported in this single-file solver: " +
            joint_name;
        return false;
      }

      active_joint_ids_.push_back(jid);
      active_q_indices_.push_back(jmodel.idx_q());
      active_v_indices_.push_back(jmodel.idx_v());
    }

    loaded_ = true;
    last_error_.clear();
    return true;
  }

  const std::string& GetLastError() const { return last_error_; }

  // 只解位置
  IkResult6Dof SolvePositionIK(
      const Eigen::Vector3d& target_position,
      const std::vector<double>& current_q_full,
      int max_iters = 200,
      double tol = 1e-4,
      double damping = 1e-3,
      double step_scale = 0.35,
      double max_delta_per_iter = 0.15) {
    IkResult6Dof result;

    if (!loaded_) {
      result.message = "IK solver model not loaded.";
      return result;
    }

    if (active_q_indices_.size() != 6) {
      result.message = "Active joint number is not 6.";
      return result;
    }

    Eigen::VectorXd q = pinocchio::neutral(model_);

    // 使用当前关节角作为初值，并归一化
    for (size_t i = 0; i < active_q_indices_.size(); ++i) {
      if (i < current_q_full.size()) {
        q[active_q_indices_[i]] = NormalizeAngle(current_q_full[i]);
      }
    }

    for (int iter = 0; iter < max_iters; ++iter) {
      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);

      const pinocchio::SE3& oMf = data_.oMf[ee_frame_id_];
      Eigen::Vector3d pos_err = target_position - oMf.translation();
      double err_norm = pos_err.norm();

      result.iterations = iter + 1;
      result.final_error = err_norm;

      if (err_norm < tol) {
        result.success = true;
        for (size_t i = 0; i < 6; ++i) {
          double raw_q = q[active_q_indices_[i]];
          double ref_q = (i < current_q_full.size()) ? current_q_full[i] : 0.0;
          result.q[i] = NearestEquivalentAngle(raw_q, ref_q);
        }
        result.message = "Position IK converged.";
        return result;
      }

      Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, model_.nv);
      J6.setZero();
      pinocchio::computeFrameJacobian(
          model_, data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

      Eigen::MatrixXd Jpos_full = J6.topRows<3>();

      Eigen::Matrix<double, 3, 6> J_active;
      J_active.setZero();
      for (size_t i = 0; i < 6; ++i) {
        J_active.col(i) = Jpos_full.col(active_v_indices_[i]);
      }

      // 阻尼最小二乘
      Eigen::Matrix3d A =
          J_active * J_active.transpose() +
          (damping * damping) * Eigen::Matrix3d::Identity();

      Eigen::Matrix<double, 6, 1> dq =
          J_active.transpose() * A.ldlt().solve(pos_err);

      dq *= step_scale;

      // 更新时加限幅 + 归一化
      for (size_t i = 0; i < 6; ++i) {
        double step = Clamp(dq[i], -max_delta_per_iter, max_delta_per_iter);
        q[active_q_indices_[i]] += step;
        q[active_q_indices_[i]] = NormalizeAngle(q[active_q_indices_[i]]);
      }
    }

    // 即使失败，也返回一个已归一化、且最接近当前状态的结果
    for (size_t i = 0; i < 6; ++i) {
      double raw_q = q[active_q_indices_[i]];
      double ref_q = (i < current_q_full.size()) ? current_q_full[i] : 0.0;
      result.q[i] = NearestEquivalentAngle(raw_q, ref_q);
    }
    result.success = false;
    result.message = "Position IK did not converge within max iterations.";
    return result;
  }

  // 解位置 + 姿态
  IkResult6Dof SolvePoseIK(
      const Eigen::Vector3d& target_position,
      const Eigen::Quaterniond& target_quat,
      const std::vector<double>& current_q_full,
      int max_iters = 200,
      double tol = 1e-4,
      double damping = 1e-3,
      double step_scale = 0.25,
      double max_delta_per_iter = 0.12) {
    IkResult6Dof result;

    if (!loaded_) {
      result.message = "IK solver model not loaded.";
      return result;
    }

    if (active_q_indices_.size() != 6) {
      result.message = "Active joint number is not 6.";
      return result;
    }

    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (size_t i = 0; i < active_q_indices_.size(); ++i) {
      if (i < current_q_full.size()) {
        q[active_q_indices_[i]] = NormalizeAngle(current_q_full[i]);
      }
    }

    const pinocchio::SE3 target_pose(
        target_quat.normalized().toRotationMatrix(),
        target_position);

    for (int iter = 0; iter < max_iters; ++iter) {
      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);

      const pinocchio::SE3& current_pose = data_.oMf[ee_frame_id_];
      pinocchio::SE3 dMi = current_pose.actInv(target_pose);

      Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMi).toVector();
      double err_norm = err.norm();

      result.iterations = iter + 1;
      result.final_error = err_norm;

      if (err_norm < tol) {
        result.success = true;
        for (size_t i = 0; i < 6; ++i) {
          double raw_q = q[active_q_indices_[i]];
          double ref_q = (i < current_q_full.size()) ? current_q_full[i] : 0.0;
          result.q[i] = NearestEquivalentAngle(raw_q, ref_q);
        }
        result.message = "Pose IK converged.";
        return result;
      }

      Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, model_.nv);
      J6.setZero();
      pinocchio::computeFrameJacobian(
          model_, data_, q, ee_frame_id_, pinocchio::LOCAL, J6);

      Eigen::Matrix<double, 6, 6> J_active;
      J_active.setZero();
      for (size_t i = 0; i < 6; ++i) {
        J_active.col(i) = J6.col(active_v_indices_[i]);
      }

      Eigen::Matrix<double, 6, 6> A =
          J_active * J_active.transpose() +
          (damping * damping) * Eigen::Matrix<double, 6, 6>::Identity();

      Eigen::Matrix<double, 6, 1> dq =
          -J_active.transpose() * A.ldlt().solve(err);

      dq *= step_scale;

      for (size_t i = 0; i < 6; ++i) {
        double step = Clamp(dq[i], -max_delta_per_iter, max_delta_per_iter);
        q[active_q_indices_[i]] += step;
        q[active_q_indices_[i]] = NormalizeAngle(q[active_q_indices_[i]]);
      }
    }

    for (size_t i = 0; i < 6; ++i) {
      double raw_q = q[active_q_indices_[i]];
      double ref_q = (i < current_q_full.size()) ? current_q_full[i] : 0.0;
      result.q[i] = NearestEquivalentAngle(raw_q, ref_q);
    }
    result.success = false;
    result.message = "Pose IK did not converge within max iterations.";
    return result;
  }

 private:
  bool loaded_{false};
  std::string last_error_;

  pinocchio::Model model_;
  pinocchio::Data data_{pinocchio::Model()};

  std::vector<std::string> active_joint_names_;
  std::vector<pinocchio::JointIndex> active_joint_ids_;
  std::vector<int> active_q_indices_;
  std::vector<int> active_v_indices_;

  pinocchio::FrameIndex ee_frame_id_{0};
};

// 文件级静态 IK 求解器
static PinocchioIKSolver6Dof g_ik_solver;
static bool g_ik_initialized = false;

// 最近一次 IK 解
static std::array<double, 6> g_last_ik_solution{{0., 0., 0., 0., 0., 0.}};
static bool g_last_ik_success = false;
static double g_last_ik_error = 0.0;

// 你这里默认使用 joint1~joint6
static const std::vector<std::string> kActiveJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

}  // namespace

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

  // ============== 新增 IK 参数 ==============
  this->declare_parameter<bool>("ik_enable", true);
  this->declare_parameter<bool>("ik_use_orientation", false);
  this->declare_parameter<std::string>("ee_frame", "link6");

  this->declare_parameter<double>("target_x", 0.30);
  this->declare_parameter<double>("target_y", 0.30);
  this->declare_parameter<double>("target_z", 0.20);

  this->declare_parameter<double>("target_qx", 0.0);
  this->declare_parameter<double>("target_qy", 0.0);
  this->declare_parameter<double>("target_qz", 0.0);
  this->declare_parameter<double>("target_qw", 1.0);

  this->declare_parameter<int>("ik_max_iters", 200);
  this->declare_parameter<double>("ik_tol", 1e-4);
  this->declare_parameter<double>("ik_damping", 1e-3);
  this->declare_parameter<double>("ik_step_scale", 0.35);
  this->declare_parameter<double>("ik_max_delta_per_iter", 0.15);
  // =========================================

  std::string port;
  this->get_parameter("port_name", port);

  std::string urdf_path;
  this->get_parameter("urdf_path", urdf_path);
  if (!gravity_compensation_.LoadModel(urdf_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from: %s", urdf_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "URDF model loaded successfully from: %s", urdf_path.c_str());
  }

  // ===== 新增：IK 模型加载 =====
  std::string ee_frame;
  this->get_parameter("ee_frame", ee_frame);
  if (!g_ik_solver.LoadModel(urdf_path, kActiveJointNames, ee_frame)) {
    RCLCPP_ERROR(this->get_logger(), "IK model load failed: %s",
                 g_ik_solver.GetLastError().c_str());
  } else {
    g_ik_initialized = true;
    RCLCPP_INFO(this->get_logger(),
                "IK model loaded successfully. ee_frame = %s",
                ee_frame.c_str());
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
  cmd_.position.resize(7);

  RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());

  bool publish_joint_feedback = this->get_parameter("publish_joint_feedback").as_bool();

  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  if (publish_joint_feedback) {
    pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  }

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

  // 保留你原来的重力补偿逻辑
  ComputeAndPublishCompensation();

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.position = arm_state_.position;
  joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  joint_state_msg.effort = arm_state_.current;
  joint_state_msg.velocity = arm_state_.velocity;
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

  // ==========================================
  // 新增：IK 解算
  // 只解算，不发送到硬件
  // ==========================================
  bool ik_enable = this->get_parameter("ik_enable").as_bool();
  if (ik_enable && g_ik_initialized) {
    const bool ik_use_orientation =
        this->get_parameter("ik_use_orientation").as_bool();

    const double tx = this->get_parameter("target_x").as_double();
    const double ty = this->get_parameter("target_y").as_double();
    const double tz = this->get_parameter("target_z").as_double();

    const double qx = this->get_parameter("target_qx").as_double();
    const double qy = this->get_parameter("target_qy").as_double();
    const double qz = this->get_parameter("target_qz").as_double();
    const double qw = this->get_parameter("target_qw").as_double();

    const int ik_max_iters = this->get_parameter("ik_max_iters").as_int();
    const double ik_tol = this->get_parameter("ik_tol").as_double();
    const double ik_damping = this->get_parameter("ik_damping").as_double();
    const double ik_step_scale = this->get_parameter("ik_step_scale").as_double();
    const double ik_max_delta_per_iter =
        this->get_parameter("ik_max_delta_per_iter").as_double();

    Eigen::Vector3d target_pos(tx, ty, tz);

    // 当前关节角作为初值，但先归一化到 [-pi, pi]
    std::vector<double> current_q6(6, 0.0);
    for (size_t i = 0; i < 6 && i < arm_state_.position.size(); ++i) {
      current_q6[i] = NormalizeAngle(arm_state_.position[i]);
    }

    IkResult6Dof ik_result;
    if (ik_use_orientation) {
      Eigen::Quaterniond target_quat(qw, qx, qy, qz);
      ik_result = g_ik_solver.SolvePoseIK(
          target_pos,
          target_quat,
          current_q6,
          ik_max_iters,
          ik_tol,
          ik_damping,
          ik_step_scale,
          ik_max_delta_per_iter);
    } else {
      ik_result = g_ik_solver.SolvePositionIK(
          target_pos,
          current_q6,
          ik_max_iters,
          ik_tol,
          ik_damping,
          ik_step_scale,
          ik_max_delta_per_iter);
    }

    // 再做一次基于当前关节角的连续化处理，保证输出稳定
    for (size_t i = 0; i < 6; ++i) {
      ik_result.q[i] = NearestEquivalentAngle(ik_result.q[i], current_q6[i]);
    }

    g_last_ik_solution = ik_result.q;
    g_last_ik_success = ik_result.success;
    g_last_ik_error = ik_result.final_error;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "IK [%s] iters=%d err=%.6f q=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
        ik_result.success ? "OK" : "FAIL",
        ik_result.iterations,
        ik_result.final_error,
        g_last_ik_solution[0], g_last_ik_solution[1], g_last_ik_solution[2],
        g_last_ik_solution[3], g_last_ik_solution[4], g_last_ik_solution[5]);
  }
}

void SlaveArmNode::ComputeAndPublishCompensation() {
  std::array<double, 7> joint_positions;
  for (int i = 0; i < 7; ++i) {
    joint_positions[i] = arm_state_.position[i];
  }

  auto tau_comp = gravity_compensation_.Compute(joint_positions);
  std_msgs::msg::Float64MultiArray tau_comp_msg;
  cmd_.header.stamp = this->now();

  for (int i = 0; i < 7; ++i) {
    cmd_.current[i] = tau_comp[i];
    cmd_.position[i] = g_last_ik_solution[i];
    cmd_.p[i] = 10;
    cmd_.velocity[i] = 0.5;
    cmd_.d[i] = 0.5;
    tau_comp_msg.data.push_back(tau_comp[i]);
  }

  arm_->SetMotorCommand(cmd_);
}

void SlaveArmNode::DebugInfoCallback() {
  bool debug_info = this->get_parameter("debug_info").as_bool();
  if (!debug_info) return;

  RCLCPP_INFO(this->get_logger(),
              "Joint position (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.position[0], arm_state_.position[1], arm_state_.position[2],
              arm_state_.position[3], arm_state_.position[4], arm_state_.position[5],
              arm_state_.position[6]);

  RCLCPP_INFO(this->get_logger(),
              "Joint currents (A): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              arm_state_.current[0], arm_state_.current[1], arm_state_.current[2],
              arm_state_.current[3], arm_state_.current[4], arm_state_.current[5],
              arm_state_.current[6]);

  RCLCPP_INFO(this->get_logger(),
              "Last IK [%s] err=%.6f q[6] (rad): [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
              g_last_ik_success ? "OK" : "FAIL",
              g_last_ik_error,
              g_last_ik_solution[0], g_last_ik_solution[1], g_last_ik_solution[2],
              g_last_ik_solution[3], g_last_ik_solution[4], g_last_ik_solution[5]);
}

}  // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::SlaveArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}