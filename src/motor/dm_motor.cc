#include <manipulator/motor/dm_motor.h>
#include <rclcpp/rclcpp.hpp>

auto logger = rclcpp::get_logger("Controller");
namespace manipulator::motor {
DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
                 float kp, float kd, CoordinateSystem coord_system)
 : protocol_(protocol), id_(id), default_kp_(kp), default_kd_(kd), vel_set_(0), coord_system_(coord_system) {

}

void DMMotor::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_);
  temperature_ = protocol_->GetTemperature(id_);
  
  if (coord_system_ == CoordinateSystem::LeftHand) {
    position_ = -position_;
    velocity_ = -velocity_;
    torque_ = -torque_;
  }
  
  if (not is_received_) pos_set_ = position_;
  is_received_ = true;
}

void DMMotor::SetState(const sensor_msgs::msg::JointState& state) {
  dummy_interface::msg::MotorControl cmd;

  double pos = state.position[0];
  double vel = state.velocity[0];
  
  if (coord_system_ == CoordinateSystem::LeftHand) {
    pos = -pos;
    vel = -vel;
  }

  cmd.position.push_back(pos);
  cmd.velocity.push_back(vel);
  cmd.p.push_back(default_kp_);
  cmd.d.push_back(default_kd_);
  cmd.current.push_back(0.1);
  UpdateCommand(cmd);
}
void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (not is_received_) return;

  uint8_t cmd_ind = id_;
  if (cmd.position.size() == 1) cmd_ind = 0;
  
  double current = cmd.current[cmd_ind];
  if (coord_system_ == CoordinateSystem::LeftHand) {
    current = -current;
  }
  protocol_->SetKp(id_, cmd.p[cmd_ind]);
  protocol_->SetKd(id_, cmd.d[cmd_ind]);
  protocol_->SetCurrent(id_, current);

  if(cmd.position.empty()) return;

  
  double pos_err = cmd.position[cmd_ind] - position_;
  if (abs(pos_err) < 0.01) {
    pos_set_ = position_;
    return;
  }

  double delta = pos_err * DT_;
  if (std::abs(delta) < 0.01) {
    delta = (delta >= 0) ? 0.01 : -0.01;
  }
  pos_set_ += delta;
  pos_set_ = std::clamp(pos_set_, std::min(cmd.position[cmd_ind], position_), std::max(cmd.position[cmd_ind], position_));
  
  if (id_ == 6) pos_set_ = cmd.position[id_];
  
  double vel_cmd = cmd.velocity[id_];
  if (coord_system_ == CoordinateSystem::LeftHand) {
    vel_cmd = -vel_cmd;
  }
  double pos_set_send = pos_set_;
  if (coord_system_ == CoordinateSystem::LeftHand) {
    pos_set_send = -pos_set_;
  }
  
  // RCLCPP_INFO(logger, "pos_set_: %f, cmd.position[id_]: %f, position_: %f", pos_set_, cmd.position[id_], position_);
  protocol_->SetPosition(id_, pos_set_send);
  protocol_->SetVelocity(id_, vel_cmd);
}

double DMMotor::GetPosition() const {
  return position_;
}

double DMMotor::GetVelocity() const {
  return velocity_;
}

double DMMotor::GetCurrent() const {
  return torque_;
}

double DMMotor::GetTemperature() const {
  return temperature_;
}

double DMMotor::GetVoltage() const {
  return voltage_;
}
}