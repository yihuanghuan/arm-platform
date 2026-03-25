#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>

namespace manipulator {

/**
 * @brief Gravity compensation algorithm for robotic arm
 * 
 * This class computes gravity compensation torques for a 7-DOF robotic arm
 * based on forward kinematics and mass properties of each link.
 * It also provides collision detection based on force feedback.
 */
class GravityCompensation {
 public:
  GravityCompensation();
  ~GravityCompensation() = default;

  /**
   * @brief Set gravity compensation parameters
   * @param G_GAIN_0 Gain for base joints (joints 0-1)
   * @param G_GAIN_1 Gain for arm joints (joints 2-3)
   * @param G_GAIN_2 Gain for wrist joints (joints 4-6)
   * @param MAX_TORQUE Maximum torque limit for each joint
   * @param GRAVITY Gravity acceleration (default: 9.81 m/s²)
   * @param FORCE_FEEDBACK_THRESHOLD Force threshold for collision detection
   * @param FORCE_FEEDBACK_GAIN Gain for force feedback control
   */
  void SetParams(double G_GAIN_0, double G_GAIN_1, double G_GAIN_2, 
                double MAX_TORQUE, double GRAVITY,double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN);

  /**
   * @brief Set UAV pose for gravity compensation
   * @param pose UAV pose (roll=x, pitch=y, yaw=z) in radians
   */
  void SetUavPose(const geometry_msgs::msg::Point& pose);

  /**
   * @brief Compute gravity compensation torques
   * @param joint_positions Current joint positions in radians
   * @return Array of 7 compensation torques in Nm
   */
  std::array<double, 7> Compute(const std::array<double, 7>& joint_positions);

  /**
   * @brief Collision detection based on force feedback
   * @param tau_comp Computed gravity compensation torques
   * @param joint_currents_ Current joint currents from feedback
   * @param compensation_torques Expected compensation torques
   * @return Modified torques with collision detection applied
   */
  std::array<double, 7> collision_detection(const std::array<double, 7>& tau_comp,const std::array<double, 7>& joint_currents_,const std::array<double, 7>& compensation_torques);
 private:
  /**
   * @brief Compute DH transformation matrix
   * @param a Link length
   * @param alpha Link twist
   * @param d Link offset
   * @param theta Joint angle
   * @return 4x4 transformation matrix
   */
  Eigen::Matrix4d DhTransform(double a, double alpha, double d, double theta);

  /**
   * @brief Compute forward kinematics and link center of mass positions
   * @param q Joint positions
   * @param link_coms Output: center of mass positions for each link
   * @param transforms Output: transformation matrices for each link
   */
  void ForwardKinematics(const std::array<double, 7>& q, 
                      std::vector<Eigen::Vector3d>& link_coms,
                      std::vector<Eigen::Matrix4d>& transforms);

  // Physical dimensions of the arm (in meters)
  static constexpr double L_BASE = 99.9 / 1000.0;
  static constexpr double D_BASE = 30.88 / 1000.0;
  static constexpr double L_ARM = 192.43 / 1000.0;
  static constexpr double D_ELBOW = 49.44 / 1000.0;
  static constexpr double L_FOREARM = 163.7 / 1000.0;
  static constexpr double L_WRIST = 59.2 / 1000.0;

  // Mass properties (in kg)
  const std::array<double, 6> MOTOR_MASSES_ = {0.325, 0.362, 0.3, 0.15, 0.2, 0.2};
  const std::array<double, 6> LINK_MASSES_ = {0.03, 0.03, 0.05, 0.05, 0.05, 0.05};

  // DH parameters for each joint: {a, alpha, d, theta_offset}
  const std::array<std::array<double, 4>, 6> DH_PARAMS_ = {{
      {D_BASE,    M_PI / 2,    L_BASE,     M_PI / 2},
      {-L_ARM,    M_PI,        0,          -M_PI / 2},
      {-D_ELBOW,  M_PI / 2,    0,          0},
      {0,         M_PI / 2,    L_FOREARM,  M_PI},
      {0,         M_PI / 2,    0,          0},
      {0,         0,            -L_WRIST,   0}
  }};

  // Center of mass positions for each link (in link frame)
  const std::array<Eigen::Vector3d, 6> LINK_COMS_ = {{
      Eigen::Vector3d(0, D_BASE / 2, L_BASE / 2),
      Eigen::Vector3d(0, L_ARM / 2, 0),
      Eigen::Vector3d(-D_ELBOW / 2, 0, 0),
      Eigen::Vector3d(0, 0, L_FOREARM / 2),
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, -L_WRIST / 2)
  }};

  // Gravity compensation parameters
  struct CompensationParams {
    double G_GAIN_0;
    double G_GAIN_1;
    double G_GAIN_2;
    double MAX_TORQUE;
    double GRAVITY;
    double FORCE_FEEDBACK_THRESHOLD;
    double FORCE_FEEDBACK_GAIN;
  };
  CompensationParams params_;

  // UAV pose (roll=x, pitch=y, yaw=z in radians)
  geometry_msgs::msg::Point uav_pose_;
};

} // namespace manipulator
