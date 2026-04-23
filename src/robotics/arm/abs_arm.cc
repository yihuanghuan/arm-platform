#include <manipulator/robotics/arm/abs_arm.h>
#include <stdexcept>
#include <algorithm>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

auto mylogger = rclcpp::get_logger("AbsArm");

namespace manipulator::arm {
AbsArm::AbsArm() {
  
}
AbsArm::AbsArm(bus::IBus::UniquePtr bus) : bus_(std::move(bus)){

}

void AbsArm::Init(const std::string& port, uint32_t baud) {
  throw std::runtime_error("Init not implemented for this arm type");
}

void AbsArm::SetBus(bus::IBus::UniquePtr bus) {
  bus_ = std::move(bus);
}

bool AbsArm::SetMotorCommand(const dummy_interface::msg::MotorControl& cmd) {
  for (auto& [_, motor] : motor_map_ ) {
    motor->UpdateCommand(cmd);
  }
  bus_->Send(); 
  return true;
}

std::pair<double, double> AbsArm::GetSCurvePosition(double start_pos, double end_pos, 
                                                      double t, double duration) const {
  if (t <= 0) {
    return {start_pos, 0.0};
  }
  if (t >= duration) {
    return {end_pos, 0.0};
  }
  
  double tau = t / duration;
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double tau4 = tau3 * tau;
  double tau5 = tau4 * tau;
  
  double pos = start_pos + (end_pos - start_pos) * (10 * tau3 - 15 * tau4 + 6 * tau5);
  double vel = (end_pos - start_pos) / duration * (30 * tau2 - 60 * tau3 + 30 * tau4);
  
  return {pos, vel};
}

bool AbsArm::WaitUntilCommandReached(const dummy_interface::msg::MotorControl& cmd, 
                                     double timeout_sec, double tolerance) {
  UpdateJointStates();
  std::vector<double> start_positions = joint_states_.position;
  std::vector<double> end_positions = cmd.position;
  
  double scurve_duration = 0.3;
  double dt = 0.001;
  int num_points = static_cast<int>(scurve_duration / dt);
  
  std::vector<std::vector<double>> scurve_positions(num_points + 1);
  std::vector<std::vector<double>> scurve_velocities(num_points + 1);
  
  for (int i = 0; i <= num_points; ++i) {
    double t = i * dt;
    scurve_positions[i].resize(start_positions.size());
    scurve_velocities[i].resize(start_positions.size());
    
    for (size_t j = 0; j < start_positions.size(); ++j) {
      auto [pos, vel] = GetSCurvePosition(start_positions[j], end_positions[j], t, scurve_duration);
      scurve_positions[i][j] = pos;
      scurve_velocities[i][j] = vel;
    }
  }
  
  auto start_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count();
  
  while (elapsed < timeout_sec) {
    elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count();
    
    double t_elapsed = elapsed;
    if (t_elapsed > scurve_duration) t_elapsed = scurve_duration;
    
    int idx = static_cast<int>(t_elapsed / dt);
    if (idx > num_points) idx = num_points;
    
    dummy_interface::msg::MotorControl scurve_cmd = cmd;
    scurve_cmd.position = scurve_positions[idx];
    scurve_cmd.velocity = scurve_velocities[idx];
    
    bool all_reached = true;
    UpdateJointStates();
    // RCLCPP_INFO(mylogger, "joint_states_.velocity: %f %f %f %f %f %f", joint_states_.velocity[0], joint_states_.velocity[1], joint_states_.velocity[2], joint_states_.velocity[3], joint_states_.velocity[4], joint_states_.velocity[5]);
    for (size_t i = 0; i < joint_states_.position.size(); ++i) {
      double pos_err = std::abs(joint_states_.position[i] - end_positions[i]);
      if (pos_err > 0.2 or std::abs(joint_states_.velocity[i]) > 0.1) {
        all_reached = false;
        break;
      }
    } 
    if (all_reached) return true;
    SetMotorCommand(scurve_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  return false;
}

bool AbsArm::SetJointStates(const sensor_msgs::msg::JointState& states) {
  for (auto& [joint_name, motor] : motor_map_ ) {
    sensor_msgs::msg::JointState state;
    state.header = states.header;
    auto name_ind = std::find(states.name.begin(), states.name.end(), joint_name);
    if (name_ind != states.name.end() and motor) {
      state.name.push_back(joint_name);
      size_t index = std::distance(states.name.begin(), name_ind);
      state.position.push_back(states.position[index]);
      state.velocity.push_back(states.velocity[index]);
      state.effort.push_back(states.effort[index]);
      motor->SetState(state);
    }
  }
  bus_->Send(); 
  return true;
}

void AbsArm::AddMotor(const std::string& name, motor::IMotor::SharedPtr motor) {
  motor_map_.emplace(name, motor);
  joint_states_.position.push_back(0.0);
  joint_states_.velocity.push_back(0.0);
  joint_states_.current.push_back(0.0);
  joint_states_.voltage.push_back(0.0);
  joint_states_.temperature.push_back(0.0);
  joint_states_.name.push_back(name);
}

void AbsArm::RemoveMotor(const std::string& name) {
  motor_map_.erase(name);
  auto it = std::find(joint_states_.name.begin(), joint_states_.name.end(), name);
  if (it != joint_states_.name.end()) {
    size_t index = std::distance(joint_states_.name.begin(), it);
    joint_states_.name.erase(joint_states_.name.begin() + index);
    joint_states_.position.erase(joint_states_.position.begin() + index);
    joint_states_.velocity.erase(joint_states_.velocity.begin() + index);
    joint_states_.current.erase(joint_states_.current.begin() + index);
    joint_states_.voltage.erase(joint_states_.voltage.begin() + index);
    joint_states_.temperature.erase(joint_states_.temperature.begin() + index);
  }
}

void AbsArm::UpdateJointStates() {
  bus_->Read();
  for (uint8_t i = 0; i < motor_map_.size(); i++) {
    std::string name = joint_states_.name[i];
    auto motor = motor_map_[name];
    joint_states_.position[i] = motor->GetPosition();
    joint_states_.velocity[i] = motor->GetVelocity();
    joint_states_.current[i] = motor->GetCurrent();
    joint_states_.voltage[i] = motor->GetVoltage();
    joint_states_.temperature[i] = motor->GetTemperature();
  }
}
JointState& AbsArm::GetJointStates() {
  UpdateJointStates();
  return joint_states_;
}
} // namespace manipulator::arm
