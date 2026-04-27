#include <manipulator/controller/gravity_controller.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <iostream>

namespace manipulator::controller {

GravityController::GravityController()
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

bool GravityController::LoadModel(const std::string& urdf_path) {
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

void GravityController::SetParams(double MAX_TORQUE, double GRAVITY,
                                           double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN) {
  params_.MAX_TORQUE = MAX_TORQUE;
  params_.GRAVITY = GRAVITY;
  params_.FORCE_FEEDBACK_THRESHOLD = FORCE_FEEDBACK_THRESHOLD;
  params_.FORCE_FEEDBACK_GAIN = FORCE_FEEDBACK_GAIN;
  params_.collision_coeffs = {1.0, 1.0, 1.0, 0.3, 0.3, 0.3, 0.3};
  UpdateGravityVector();
}

void GravityController::SetCollisionCoeffs(const std::array<double, 7>& coeffs) {
  params_.collision_coeffs = coeffs;
}

void GravityController::SetUavPose(const geometry_msgs::msg::Point& pose) {
  uav_pose_ = pose;
  UpdateGravityVector();
}

void GravityController::SetRotationAngle(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  
  rotation_matrix_ = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  UpdateGravityVector();
}

void GravityController::UpdateGravityVector() {
  Eigen::Vector3d gravity_world(0, 0, -params_.GRAVITY);
  gravity_vector_ = rotation_matrix_ * gravity_world;
  
  if (model_loaded_) {
    model_.gravity.linear(gravity_vector_);
  }
}

JointCommand GravityController::Compute(const JointStates& joint_states,
    const JointSetpoint& joint_setpoint,
    double dt) {

  if (!model_loaded_) {
    throw std::runtime_error("Model not loaded");
  }
  std::array<double, 7> tau_comp = {0, 0, 0, 0, 0, 0, 0};
  JointCommand cmd;
  size_t num_joints = joint_states.position.size();
  cmd.position.clear();
  cmd.velocity.clear();
  cmd.current.resize(num_joints);
  cmd.p.resize(num_joints);
  cmd.d.resize(num_joints);
  try {
    Eigen::VectorXd q(model_.nq);
    for (int i = 0; i < std::min(7, static_cast<int>(model_.nq)); ++i) {
      q(i) = joint_states.position[i];
    }

    pinocchio::computeGeneralizedGravity(model_, data_, q);

    std::array<double, 7> tau_gravity = {0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < std::min(7, static_cast<int>(model_.nv)); ++i) {
      tau_gravity[i] = data_.g(i);
    }

    for (int i = 0; i < 7; ++i) {
      tau_comp[i] = tau_gravity[i];
      if(i==0) tau_comp[i] *= 1.5;
      if (tau_comp[i] > params_.MAX_TORQUE) tau_comp[i] = params_.MAX_TORQUE;
      else if (tau_comp[i] < -params_.MAX_TORQUE) tau_comp[i] = -params_.MAX_TORQUE;

      cmd.current[i] = tau_comp[i];
    }
  
    // tau_comp[2] *= -1;
  } catch (const std::exception& e) {
    std::cerr << "Error computing gravity compensation: " << e.what() << std::endl;
  }

  return cmd;
}

std::array<double, 7> GravityController::collision_detection(
    const std::array<double, 7>& tau_comp,
    const std::array<double, 7>& joint_currents_,
    const std::array<double, 7>& compensation_torques) {
  std::array<double, 7> tau_current;
  tau_current = tau_comp;
  for (int i = 0; i < 7; ++i) {
    if (std::abs(joint_currents_[i] - compensation_torques[i]) > params_.FORCE_FEEDBACK_THRESHOLD) {
      tau_current[i] += -params_.collision_coeffs[i] * params_.FORCE_FEEDBACK_GAIN * (joint_currents_[i] - compensation_torques[i]);
    } else {
      tau_current[i] += 0.0;
    }
  }
  
  return tau_current;
}

}