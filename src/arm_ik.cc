#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>   // log3

#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iostream>

/**
 * One-shot IK node (name-order fixed + optional gripper 90deg rotation)
 * ---------------------------------------------------------------------
 * 输入:
 *   - /joint_states            : 当前关节角
 *   - ee_desired_pose          : 目标末端位姿
 *
 * 输出:
 *   - joint_states (可选)      : 绝对关节位置命令
 *   - arm/joint_command_debug  : 调试输出
 *
 * 核心:
 *   每次从当前关节角 q_ 出发，数值迭代求解一个 q_sol，
 *   直接发布 q_sol，而不是 qdot * dt 积分。
 *
 * 重点:
 *   - IK内部固定按 [joint1..joint6] 计算
 *   - 发布时按真实收到的 /joint_states.name 顺序重排
 *   - 支持在目标姿态上附加“夹爪旋转90度”
 */

class CartesianToJointHardware : public rclcpp::Node
{
public:
  CartesianToJointHardware() : Node("arm_cartesian_to_joint_node")
  {
    // ---------------- Parameters ----------------
    declare_parameter<std::string>(
      "urdf_path",
      "/home/crz/clean_moveit_ws/install/my_robot_description/"
      "share/my_robot_description/urdf/robot.urdf");
    declare_parameter<std::string>("ee_frame", "link6");

    // Loop rate
    declare_parameter<double>("dt", 0.05);

    // DLS damping
    declare_parameter<double>("damping", 1e-3);

    // IK solver params
    declare_parameter<int>("ik_max_iters", 100);
    declare_parameter<double>("ik_alpha", 0.5);
    declare_parameter<double>("ik_pos_tol", 1e-4);
    declare_parameter<double>("ik_rot_tol", 1e-3);
    declare_parameter<double>("ik_max_step", 0.1);

    // Orientation mode:
    //  - "track_pose"   : 跟踪目标姿态
    //  - "hold_initial" : 保持启动时姿态
    declare_parameter<std::string>("orientation_mode", "track_pose");

    // true: 只有收到新目标才求解一次
    // false: 每个update周期都重新求解
    declare_parameter<bool>("solve_only_on_new_pose", true);

    // 如果没收到 ee_desired_pose，是否使用手动目标
    declare_parameter<bool>("use_manual_target_if_no_pose", true);
    declare_parameter<double>("target_x", 0.40);
    declare_parameter<double>("target_y", 0.00);
    declare_parameter<double>("target_z", 0.00);
    declare_parameter<double>("target_qx", 0.0);
    declare_parameter<double>("target_qy", 0.0);
    declare_parameter<double>("target_qz", 0.0);
    declare_parameter<double>("target_qw", 1.0);

    // 末端夹爪额外旋转 90°
    declare_parameter<bool>("enable_gripper_rotate_90", true);
    declare_parameter<std::string>("gripper_rotate_axis", "y");  // x / y / z

    // Output topics
    declare_parameter<bool>("publish_to_joint_states", true);
    declare_parameter<std::string>("joint_command_topic", "joint_states");
    declare_parameter<std::string>("debug_command_topic", "arm/joint_command_debug");

    // Optional joint limits
    declare_parameter<bool>("enable_joint_limits", false);
    declare_parameter<std::vector<double>>("joint_min", std::vector<double>());
    declare_parameter<std::vector<double>>("joint_max", std::vector<double>());

    // Read parameters
    get_parameter("urdf_path", urdf_path_);
    get_parameter("ee_frame", ee_frame_name_);
    get_parameter("dt", dt_);
    get_parameter("damping", damping_);
    get_parameter("ik_max_iters", ik_max_iters_);
    get_parameter("ik_alpha", ik_alpha_);
    get_parameter("ik_pos_tol", ik_pos_tol_);
    get_parameter("ik_rot_tol", ik_rot_tol_);
    get_parameter("ik_max_step", ik_max_step_);
    get_parameter("orientation_mode", orientation_mode_);
    get_parameter("solve_only_on_new_pose", solve_only_on_new_pose_);

    get_parameter("use_manual_target_if_no_pose", use_manual_target_if_no_pose_);
    get_parameter("target_x", target_x_);
    get_parameter("target_y", target_y_);
    get_parameter("target_z", target_z_);
    get_parameter("target_qx", target_qx_);
    get_parameter("target_qy", target_qy_);
    get_parameter("target_qz", target_qz_);
    get_parameter("target_qw", target_qw_);

    get_parameter("enable_gripper_rotate_90", enable_gripper_rotate_90_);
    get_parameter("gripper_rotate_axis", gripper_rotate_axis_);

    get_parameter("publish_to_joint_states", publish_to_joint_states_);
    get_parameter("joint_command_topic", joint_command_topic_);
    get_parameter("debug_command_topic", debug_command_topic_);
    get_parameter("enable_joint_limits", enable_joint_limits_);
    get_parameter("joint_min", joint_min_);
    get_parameter("joint_max", joint_max_);

    // ---------------- Pinocchio model ----------------
    pinocchio::urdf::buildModel(urdf_path_, model_);
    data_ = pinocchio::Data(model_);
    nq_ = model_.nq;
    nv_ = model_.nv;

    if (nq_ < 6 || nv_ < 6)
    {
      RCLCPP_FATAL(get_logger(), "Model nq/nv too small: nq=%d nv=%d", nq_, nv_);
      throw std::runtime_error("Bad model DOF");
    }

    q_.setZero(nq_);
    q_des_.setZero(nq_);

    ee_frame_id_ = model_.getFrameId(ee_frame_name_);
    if (ee_frame_id_ == (pinocchio::FrameIndex)(-1))
    {
      RCLCPP_FATAL(get_logger(), "EE frame '%s' not found", ee_frame_name_.c_str());
      throw std::runtime_error("Bad EE frame");
    }

    // IK内部固定顺序
    fixed_joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // ---------------- ROS interfaces ----------------
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "ee_desired_pose", 10,
      std::bind(&CartesianToJointHardware::poseCallback, this, std::placeholders::_1));

    sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      std::bind(&CartesianToJointHardware::jointStateCallback, this, std::placeholders::_1));

    pub_joint_cmd_ = create_publisher<sensor_msgs::msg::JointState>(joint_command_topic_, 10);
    pub_joint_cmd_debug_ = create_publisher<sensor_msgs::msg::JointState>(debug_command_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&CartesianToJointHardware::update, this));

    RCLCPP_INFO(get_logger(), "One-shot IK node started");
    RCLCPP_INFO(get_logger(), "ee_frame=%s dt=%.3f damping=%.2e",
                ee_frame_name_.c_str(), dt_, damping_);
    RCLCPP_INFO(get_logger(),
                "IK: max_iters=%d alpha=%.3f pos_tol=%.2e rot_tol=%.2e max_step=%.3f",
                ik_max_iters_, ik_alpha_, ik_pos_tol_, ik_rot_tol_, ik_max_step_);
    RCLCPP_INFO(get_logger(), "orientation_mode=%s", orientation_mode_.c_str());
    RCLCPP_INFO(get_logger(), "solve_only_on_new_pose=%s",
                solve_only_on_new_pose_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "enable_gripper_rotate_90=%s axis=%s",
                enable_gripper_rotate_90_ ? "true" : "false",
                gripper_rotate_axis_.c_str());
  }

private:
  // ---------------- Utility ----------------
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  static Eigen::Quaterniond quatMsgToEigen(const geometry_msgs::msg::Quaternion& q)
  {
    Eigen::Quaterniond qq(q.w, q.x, q.y, q.z);
    if (qq.norm() < 1e-12)
      qq = Eigen::Quaterniond::Identity();
    return qq.normalized();
  }

  Eigen::Matrix3d buildGripperRotate90Matrix() const
  {
    const double angle = M_PI / 2.0;

    if (gripper_rotate_axis_ == "x" || gripper_rotate_axis_ == "X")
    {
      return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }
    else if (gripper_rotate_axis_ == "y" || gripper_rotate_axis_ == "Y")
    {
      return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }
    else
    {
      return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
  }

  // ---------------- Callbacks ----------------
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    desired_pose_ = *msg;
    pose_received_ = true;
    new_pose_received_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty())
      return;

    std::lock_guard<std::mutex> lk(mutex_);

    if (!joint_map_ready_)
    {
      std::unordered_map<std::string, int> incoming_name_to_idx;
      for (size_t i = 0; i < msg->name.size(); ++i)
        incoming_name_to_idx[msg->name[i]] = static_cast<int>(i);

      incoming_joint_names_ = msg->name;
      fixed_to_incoming_index_.assign(6, -1);

      for (int j = 0; j < 6; ++j)
      {
        auto it = incoming_name_to_idx.find(fixed_joint_names_[j]);
        if (it == incoming_name_to_idx.end())
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "/joint_states missing required joint: %s",
            fixed_joint_names_[j].c_str());
          return;
        }

        joint_index_[j] = it->second;
        fixed_to_incoming_index_[j] = it->second;
      }

      joint_map_ready_ = true;

      std::ostringstream oss;
      oss << "Joint mapping ready. Incoming order: ";
      for (const auto& n : incoming_joint_names_)
        oss << n << " ";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }

    for (int j = 0; j < 6; ++j)
    {
      int idx = joint_index_[j];
      if (idx >= 0 && idx < static_cast<int>(msg->position.size()))
        q_(j) = msg->position[idx];
    }

    if (!q_des_init_)
    {
      q_des_ = q_;

      pinocchio::forwardKinematics(model_, data_, q_);
      pinocchio::updateFramePlacements(model_, data_);
      R_hold_ = data_.oMf[ee_frame_id_].rotation();
      R_hold_inited_ = true;

      q_des_init_ = true;
      RCLCPP_INFO(get_logger(), "q_des initialized from current joint state; R_hold initialized");
    }

    has_joint_state_ = true;
  }

  // ---------------- Build target pose ----------------
  bool buildTargetPose(Eigen::Vector3d& p_ref, Eigen::Matrix3d& R_ref)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    bool have_target = false;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (pose_received_)
      {
        pose_msg = desired_pose_;
        have_target = true;
      }
    }

    if (!have_target)
    {
      if (!use_manual_target_if_no_pose_)
        return false;

      pose_msg.pose.position.x = target_x_;
      pose_msg.pose.position.y = target_y_;
      pose_msg.pose.position.z = target_z_;
      pose_msg.pose.orientation.x = target_qx_;
      pose_msg.pose.orientation.y = target_qy_;
      pose_msg.pose.orientation.z = target_qz_;
      pose_msg.pose.orientation.w = target_qw_;
      have_target = true;
    }

    p_ref = Eigen::Vector3d(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);

    // 修正这里的逻辑：
    // track_pose   -> 使用目标姿态
    // hold_initial -> 使用启动时姿态
    if (orientation_mode_ == "hold_initial")
    {
      R_ref = quatMsgToEigen(pose_msg.pose.orientation).toRotationMatrix();
    }
    else
    {
      if (!R_hold_inited_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "R_hold not initialized yet");
        return false;
      }
      R_ref = R_hold_;
    }

    // 在目标姿态基础上，再附加一个夹爪旋转90度
    // 这里采用右乘，表示绕末端自身局部坐标轴旋转
    if (enable_gripper_rotate_90_)
    {
      Eigen::Matrix3d R_gripper_90 = buildGripperRotate90Matrix();
      R_ref = R_ref * R_gripper_90;
    }

    return true;
  }

  // ---------------- Main update ----------------
  void update()
  {
    if (!has_joint_state_ || !q_des_init_ || !joint_map_ready_)
      return;

    if (solve_only_on_new_pose_)
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!new_pose_received_ && pose_received_)
        return;
    }

    Eigen::VectorXd q_seed;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      q_seed = q_;
    }

    Eigen::Vector3d p_ref;
    Eigen::Matrix3d R_ref;
    if (!buildTargetPose(p_ref, R_ref))
      return;

    Eigen::VectorXd q_sol = q_seed;

    bool converged = false;
    double final_pos_err = 0.0;
    double final_rot_err = 0.0;
    int used_iters = 0;

    for (int iter = 0; iter < ik_max_iters_; ++iter)
    {
      pinocchio::forwardKinematics(model_, data_, q_sol);
      pinocchio::updateFramePlacements(model_, data_);
      const pinocchio::SE3& oMe = data_.oMf[ee_frame_id_];

      const Eigen::Vector3d p_curr = oMe.translation();
      const Eigen::Matrix3d R_curr = oMe.rotation();

      const Eigen::Vector3d e_pos = p_ref - p_curr;
      const Eigen::Matrix3d R_err = R_ref * R_curr.transpose();
      const Eigen::Vector3d e_rot = pinocchio::log3(R_err);

      final_pos_err = e_pos.norm();
      final_rot_err = e_rot.norm();
      used_iters = iter + 1;

      if (final_pos_err < ik_pos_tol_ && final_rot_err < ik_rot_tol_)
      {
        converged = true;
        break;
      }

      Eigen::Matrix<double, 6, 1> err6;
      err6.head<3>() = e_pos;
      err6.tail<3>() = e_rot;

      Eigen::MatrixXd J6(6, nv_);
      J6.setZero();
      pinocchio::computeFrameJacobian(
        model_, data_, q_sol, ee_frame_id_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J6);

      Eigen::MatrixXd JJt = (J6 * J6.transpose()).eval();
      JJt += damping_ * Eigen::MatrixXd::Identity(6, 6);
      Eigen::MatrixXd Jsharp = J6.transpose() * JJt.inverse();

      Eigen::VectorXd dq = ik_alpha_ * (Jsharp * err6);

      for (int i = 0; i < 6; ++i)
        dq(i) = clamp(dq(i), -ik_max_step_, ik_max_step_);

      q_sol += dq;

      if (enable_joint_limits_ && joint_min_.size() == 6 && joint_max_.size() == 6)
      {
        for (int i = 0; i < 6; ++i)
          q_sol(i) = clamp(q_sol(i), joint_min_[i], joint_max_[i]);
      }
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      q_des_ = q_sol;
      new_pose_received_ = false;
    }

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();
    cmd.name = incoming_joint_names_;
    cmd.position.resize(incoming_joint_names_.size(), 0.0);

    for (int i = 0; i < 6; ++i)
    {
      int incoming_idx = fixed_to_incoming_index_[i];
      if (incoming_idx >= 0 && incoming_idx < static_cast<int>(cmd.position.size()))
        cmd.position[incoming_idx] = q_sol(i);
    }

    std::cout << "ONE-SHOT IK CMD (reordered): ";
    for (size_t i = 0; i < cmd.name.size(); ++i)
      std::cout << "[" << cmd.name[i] << ": " << cmd.position[i] << "] ";
    std::cout << std::endl;

    pub_joint_cmd_debug_->publish(cmd);

    if (publish_to_joint_states_)
      pub_joint_cmd_->publish(cmd);

    if (converged)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "IK converged in %d iters, pos_err=%.6e, rot_err=%.6e",
        used_iters, final_pos_err, final_rot_err);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "IK not fully converged in %d iters, pos_err=%.6e, rot_err=%.6e",
        used_iters, final_pos_err, final_rot_err);
    }
  }

private:
  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmd_debug_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  geometry_msgs::msg::PoseStamped desired_pose_;
  bool pose_received_{false};
  bool new_pose_received_{false};

  bool has_joint_state_{false};
  bool joint_map_ready_{false};
  bool q_des_init_{false};

  std::vector<std::string> fixed_joint_names_;
  std::vector<std::string> incoming_joint_names_;
  std::vector<int> fixed_to_incoming_index_;

  int joint_index_[6]{-1, -1, -1, -1, -1, -1};

  Eigen::VectorXd q_;
  Eigen::VectorXd q_des_;

  bool R_hold_inited_{false};
  Eigen::Matrix3d R_hold_{Eigen::Matrix3d::Identity()};

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_;
  int nq_{0}, nv_{0};

  std::string urdf_path_;
  std::string ee_frame_name_;

  double dt_{0.05};
  double damping_{1e-3};

  int ik_max_iters_{100};
  double ik_alpha_{0.5};
  double ik_pos_tol_{1e-4};
  double ik_rot_tol_{1e-3};
  double ik_max_step_{0.1};

  std::string orientation_mode_{"track_pose"};
  bool solve_only_on_new_pose_{true};

  bool use_manual_target_if_no_pose_{true};
  double target_x_{0.40};
  double target_y_{0.00};
  double target_z_{0.00};
  double target_qx_{0.0};
  double target_qy_{0.0};
  double target_qz_{0.0};
  double target_qw_{1.0};

  bool enable_gripper_rotate_90_{false};
  std::string gripper_rotate_axis_{"z"};

  bool publish_to_joint_states_{true};
  std::string joint_command_topic_{"joint_states"};
  std::string debug_command_topic_{"arm/joint_command_debug"};

  bool enable_joint_limits_{false};
  std::vector<double> joint_min_;
  std::vector<double> joint_max_;
};
