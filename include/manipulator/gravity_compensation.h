#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>

namespace manipulator {

class GravityCompensation {
 public:
  GravityCompensation();
  ~GravityCompensation() = default;

  void SetParams(double G_GAIN_0, double G_GAIN_1, double G_GAIN_2, 
                double MAX_TORQUE, double GRAVITY);
  void SetUavPose(const geometry_msgs::msg::Point& pose);
  std::array<double, 7> Compute(const std::array<double, 7>& joint_positions);

 private:
  Eigen::Matrix4d DhTransform(double a, double alpha, double d, double theta);
  void ForwardKinematics(const std::array<double, 7>& q, 
                      std::vector<Eigen::Vector3d>& link_coms,
                      std::vector<Eigen::Matrix4d>& transforms);

  static constexpr double L_BASE = 99.9 / 1000.0;
  static constexpr double D_BASE = 30.88 / 1000.0;
  static constexpr double L_ARM = 192.43 / 1000.0;
  static constexpr double D_ELBOW = 49.44 / 1000.0;
  static constexpr double L_FOREARM = 163.7 / 1000.0;
  static constexpr double L_WRIST = 59.2 / 1000.0;

  const std::array<double, 6> MOTOR_MASSES_ = {0.325, 0.362, 0.3, 0.15, 0.2, 0.2};
  const std::array<double, 6> LINK_MASSES_ = {0.03, 0.03, 0.05, 0.05, 0.05, 0.05};

  const std::array<std::array<double, 4>, 6> DH_PARAMS_ = {{
      {D_BASE,    M_PI / 2,    L_BASE,     M_PI / 2},
      {-L_ARM,    M_PI,        0,          -M_PI / 2},
      {-D_ELBOW,  M_PI / 2,    0,          0},
      {0,         M_PI / 2,    L_FOREARM,  M_PI},
      {0,         M_PI / 2,    0,          0},
      {0,         0,            -L_WRIST,   0}
  }};

  const std::array<Eigen::Vector3d, 6> LINK_COMS_ = {{
      Eigen::Vector3d(0, D_BASE / 2, L_BASE / 2),
      Eigen::Vector3d(0, L_ARM / 2, 0),
      Eigen::Vector3d(-D_ELBOW / 2, 0, 0),
      Eigen::Vector3d(0, 0, L_FOREARM / 2),
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, -L_WRIST / 2)
  }};

  struct CompensationParams {
    double G_GAIN_0;
    double G_GAIN_1;
    double G_GAIN_2;
    double MAX_TORQUE;
    double GRAVITY;
  };
  CompensationParams params_;

  geometry_msgs::msg::Point uav_pose_;
};

} // namespace manipulator
