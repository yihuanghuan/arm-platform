#include <manipulator/gravity_compensation_pinocchio.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <iostream>

namespace manipulator {

GravityCompensationPinocchio::GravityCompensationPinocchio()
    : model_loaded_(false) {
  params_.MAX_TORQUE = 3.0;
  params_.GRAVITY = 9.81;
  params_.FORCE_FEEDBACK_THRESHOLD = 0.2;
  params_.FORCE_FEEDBACK_GAIN = 0.2;

  uav_pose_.x = 0.0;
  uav_pose_.y = 0.0;
  uav_pose_.z = 0.0;

  rotation_matrix_ = Eigen::Matrix3d::Identity();
  gravity_vector_ = Eigen::Vector3d(0, 0, -params_.GRAVITY);
}

bool GravityCompensationPinocchio::LoadModel(const std::string& urdf_path) {
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    model_loaded_ = true;
    UpdateGravityVector();
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load URDF model: " << e.what() << std::endl;
    model_loaded_ = false;
    return false;
  }
}

void GravityCompensationPinocchio::SetParams(double MAX_TORQUE, double GRAVITY,
                                           double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN) {
  params_.MAX_TORQUE = MAX_TORQUE;
  params_.GRAVITY = GRAVITY;
  params_.FORCE_FEEDBACK_THRESHOLD = FORCE_FEEDBACK_THRESHOLD;
  params_.FORCE_FEEDBACK_GAIN = FORCE_FEEDBACK_GAIN;
  UpdateGravityVector();
}

void GravityCompensationPinocchio::SetUavPose(const geometry_msgs::msg::Point& pose) {
  uav_pose_ = pose;
  UpdateGravityVector();
}

void GravityCompensationPinocchio::SetRotationAngle(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  
  rotation_matrix_ = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  UpdateGravityVector();
}

void GravityCompensationPinocchio::UpdateGravityVector() {
  Eigen::Vector3d gravity_world(0, 0, -params_.GRAVITY);
  gravity_vector_ = rotation_matrix_ * gravity_world;
  
  if (model_loaded_) {
    model_.gravity.linear(gravity_vector_);
  }
}

std::array<double, 7> GravityCompensationPinocchio::Compute(const std::array<double, 7>& joint_positions) {
  std::array<double, 7> tau_comp = {0, 0, 0, 0, 0, 0, 0};

  if (!model_loaded_) {
    std::cerr << "Warning: Model not loaded, returning zero torques" << std::endl;
    return tau_comp;
  }

  try {
    Eigen::VectorXd q(model_.nq);
    for (int i = 0; i < std::min(7, static_cast<int>(model_.nq)); ++i) {
      q(i) = joint_positions[i];
    }

    pinocchio::computeGeneralizedGravity(model_, data_, q);

    std::array<double, 7> tau_gravity = {0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < std::min(7, static_cast<int>(model_.nv)); ++i) {
      tau_gravity[i] = data_.g(i);
    }

    for (int i = 0; i < 7; ++i) {
      tau_comp[i] = tau_gravity[i];
      if (tau_comp[i] > params_.MAX_TORQUE) tau_comp[i] = params_.MAX_TORQUE;
      else if (tau_comp[i] < -params_.MAX_TORQUE) tau_comp[i] = -params_.MAX_TORQUE;
    }
    // tau_comp[2] *= -1;
  } catch (const std::exception& e) {
    std::cerr << "Error computing gravity compensation: " << e.what() << std::endl;
  }

  return tau_comp;
}

std::array<double, 7> GravityCompensationPinocchio::collision_detection(
    const std::array<double, 7>& tau_comp,
    const std::array<double, 7>& joint_currents_,
    const std::array<double, 7>& compensation_torques) {
  std::array<double, 7> tau_current;
  tau_current = tau_comp;
  
  for (int i = 0; i < 7; ++i) {
    if (std::abs(joint_currents_[i] - compensation_torques[i]) > params_.FORCE_FEEDBACK_THRESHOLD) {
      tau_current[i] += -params_.FORCE_FEEDBACK_GAIN * (joint_currents_[i] - compensation_torques[i]);
    } else {
      tau_current[i] += 0.0;
    }
  }
  
  return tau_current;
}

}