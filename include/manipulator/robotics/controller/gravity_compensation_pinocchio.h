#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace manipulator {

class GravityCompensationPinocchio {
 public:
  GravityCompensationPinocchio();
  ~GravityCompensationPinocchio() = default;

  bool LoadModel(const std::string& urdf_path);

  void SetParams(double MAX_TORQUE, double GRAVITY, double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN);
  void SetCollisionCoeffs(const std::array<double, 7>& coeffs);

  void SetUavPose(const geometry_msgs::msg::Point& pose);

  void SetRotationAngle(double roll, double pitch, double yaw);

  std::array<double, 7> Compute(const std::array<double, 7>& joint_positions);

  std::array<double, 7> collision_detection(const std::array<double, 7>& tau_comp,
                                           const std::array<double, 7>& joint_currents_,
                                           const std::array<double, 7>& compensation_torques);

 private:
  void UpdateGravityVector();

  pinocchio::Model model_;
  pinocchio::Data data_;

  struct CompensationParams {
    double MAX_TORQUE;
    double GRAVITY;
    double FORCE_FEEDBACK_THRESHOLD;
    double FORCE_FEEDBACK_GAIN;
    std::array<double, 7> collision_coeffs;
  };
  CompensationParams params_;

  geometry_msgs::msg::Point uav_pose_;
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector3d gravity_vector_;
  bool model_loaded_;
};

}