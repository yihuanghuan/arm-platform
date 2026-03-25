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
  params_.FORCE_FEEDBACK_THRESHOLD = 0.2; 
  params_.FORCE_FEEDBACK_GAIN = 0.2; 
}

void GravityCompensation::SetParams(double G_GAIN_0, double G_GAIN_1, double G_GAIN_2,
                                double MAX_TORQUE, double GRAVITY,double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN) {
  params_.G_GAIN_0 = G_GAIN_0;
  params_.G_GAIN_1 = G_GAIN_1;
  params_.G_GAIN_2 = G_GAIN_2;
  params_.MAX_TORQUE = MAX_TORQUE;
  params_.GRAVITY = GRAVITY;
  params_.FORCE_FEEDBACK_THRESHOLD = FORCE_FEEDBACK_THRESHOLD;
  params_.FORCE_FEEDBACK_GAIN = FORCE_FEEDBACK_GAIN;

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
//这里的力反馈实质上是对碰撞的检测，重点在于区分机械臂检测到的力矩是重力造成的还是碰撞造成的。所以无人机上的机械臂也需要进行重力补偿计算。
//远端重力补偿力矩和检测到力矩差值过大就说明发生了碰撞
std::array<double, 7> GravityCompensation::collision_detection(const std::array<double, 7>& tau_comp,const std::array<double, 7>& joint_currents_,const std::array<double, 7>& compensation_torques) {
    std::array<double,7> tau_current;
    tau_current = tau_comp;
    for(int i = 0;i < 7;i++){
        if(std::abs(joint_currents_[i] - compensation_torques[i]) > 0.2){
          tau_current[i] += - 0.2 * (joint_currents_[i] - compensation_torques[i]);
        }
        else{
            tau_current[i] += 0.0f; 
        }
    }
    return tau_current;
}
} // namespace manipulator
