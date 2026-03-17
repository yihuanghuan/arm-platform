#include <manipulator/gravity_compensation.h>
#include <cmath>

namespace manipulator {

GravityCompensation::GravityCompensation() {
  uav_pose_.x = 0.0;
  uav_pose_.y = 0.0;
  uav_pose_.z = 0.0;
  
  params_.G_GAIN_0 = 0.5;
  params_.G_GAIN_1 = 0.5;
  params_.G_GAIN_2 = 1.0;
  params_.MAX_TORQUE = 3.0;
  params_.GRAVITY = 9.81;
}

void GravityCompensation::SetParams(double G_GAIN_0, double G_GAIN_1, double G_GAIN_2,
                                double MAX_TORQUE, double GRAVITY) {
  params_.G_GAIN_0 = G_GAIN_0;
  params_.G_GAIN_1 = G_GAIN_1;
  params_.G_GAIN_2 = G_GAIN_2;
  params_.MAX_TORQUE = MAX_TORQUE;
  params_.GRAVITY = GRAVITY;
}

void GravityCompensation::SetUavPose(const geometry_msgs::msg::Point& pose) {
  uav_pose_ = pose;
}

Eigen::Matrix4d GravityCompensation::DhTransform(double a, double alpha, double d, double theta) {
  double ct = std::cos(theta), st = std::sin(theta);
  double ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d T;
  T <<  ct , -st * ca , st * sa , a * ct,
        st , ct * ca , -ct * sa , a * st,
        0.0 , sa , ca , d ,
        0.0 , 0.0 , 0.0 , 1.0;
  return T;
}

void GravityCompensation::ForwardKinematics(const std::array<double, 7>& q,
                                     std::vector<Eigen::Vector3d>& link_coms,
                                     std::vector<Eigen::Matrix4d>& transforms) {
  transforms.clear();
  link_coms.clear();
  Eigen::Matrix4d T;
  T.setIdentity();
  transforms.push_back(T);

  for (int i = 0; i < 6; ++i) {
    double a = DH_PARAMS_[i][0], alpha = DH_PARAMS_[i][1];
    double d = DH_PARAMS_[i][2], theta0 = DH_PARAMS_[i][3];
    double theta = q[i] + theta0;
    Eigen::Vector3d com_local = LINK_COMS_[i];
    Eigen::Vector3d com_global = T.block<3,3>(0,0) * com_local + T.block<3,1>(0,3);
    link_coms.push_back(com_global);
    T = T * DhTransform(a, alpha, d, theta);
    transforms.push_back(T);
  }
}

std::array<double, 7> GravityCompensation::Compute(const std::array<double, 7>& joint_positions) {
  double roll = uav_pose_.y;
  double pitch = uav_pose_.z;

  std::vector<Eigen::Matrix4d> transforms;
  std::vector<Eigen::Vector3d> link_coms;
  ForwardKinematics(joint_positions, link_coms, transforms);

  std::array<Eigen::Vector3d, 7> origins;
  std::array<Eigen::Vector3d, 7> z_axes;
  for (int i = 0; i < 7; ++i) {
    if (i < static_cast<int>(transforms.size())) {
      origins[i] = transforms[i].block<3,1>(0,3);
      z_axes[i] = transforms[i].block<3,1>(0,2);
    } else {
      origins[i] = Eigen::Vector3d::Zero();
      z_axes[i] = Eigen::Vector3d::UnitZ();
    }
  }

  std::array<double, 7> tau_gravity = {0, 0, 0, 0, 0, 0, 0};

  for (int j = 0; j < 7; ++j) {
    for (int i = j; i < 6; ++i) {
      Eigen::Vector3d r_motor = origins[i] - origins[j];
      Eigen::Vector3d J_col = z_axes[j].cross(r_motor);
      double f = MOTOR_MASSES_[i] * params_.GRAVITY;
      Eigen::Vector3d f_gravity_motor = Eigen::Vector3d(0, 0, f);
      tau_gravity[j] += J_col.dot(f_gravity_motor);
    }

    for (int i = j; i < 6; ++i) {
      Eigen::Vector3d r_link = link_coms[i] - origins[j];
      Eigen::Vector3d J_col = z_axes[j].cross(r_link);
      double f = LINK_MASSES_[i] * params_.GRAVITY;
      Eigen::Vector3d f_gravity_link = Eigen::Vector3d(0, 0, f);
      tau_gravity[j] += J_col.dot(f_gravity_link);
    }
  }

  tau_gravity[0] = std::sin(roll) * params_.G_GAIN_0;
  tau_gravity[1] *= params_.G_GAIN_1;
  tau_gravity[2] *= params_.G_GAIN_2;

  std::array<double, 7> tau_comp;
  for (int i = 0; i < 7; ++i) {
    tau_comp[i] = -tau_gravity[i];
    if (tau_comp[i] > params_.MAX_TORQUE) tau_comp[i] = params_.MAX_TORQUE;
    else if (tau_comp[i] < -params_.MAX_TORQUE) tau_comp[i] = -params_.MAX_TORQUE;
  }

  return tau_comp;
}

} // namespace manipulator
