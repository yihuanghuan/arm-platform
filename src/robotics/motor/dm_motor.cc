#include <manipulator/robotics/motor/dm_motor.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

auto logger = rclcpp::get_logger("Controller");
namespace manipulator::motor {

DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
                 CoordinateSystem coord_system)
 : protocol_(protocol), id_(id), vel_set_(0), coord_system_(coord_system),
   lower_limit_(0), upper_limit_(0), pos_limit_(0) {
}

void DMMotor::SetRateTorque(double rate_torque) {
  rate_torque_ = rate_torque;
}

void DMMotor::SetJointLimit(double lower_limit, double upper_limit) {
  lower_limit_ = lower_limit;
  upper_limit_ = upper_limit;
}

double DMMotor::GetRatedTorque() const {
  return rate_torque_;
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
  
  is_received_ = true;
}

void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  // Motor is not ready if not received state
  if (not is_received_) return;
  if (cmd.position.empty() and cmd.current.empty()) return;

  // set cmd indx to 0 if only one position or current is given
  uint8_t cmd_ind = id_;
  if (cmd.position.size() == 1) cmd_ind = 0;

  protocol_->SetKp(cmd_ind, cmd.p[cmd_ind]);
  protocol_->SetKd(cmd_ind, cmd.d[cmd_ind]);
  // safe torque limit
  SetCurrent(cmd.current[cmd_ind], cmd_ind);

  if(position_ < lower_limit_) {
    pos_limit_ = lower_limit_;
    StopMotor(cmd_ind);
    return;
  }
  if(position_ > upper_limit_) {
    pos_limit_ = upper_limit_;
    StopMotor(cmd_ind);
    return;
  }

  if(cmd.position.empty() or cmd.velocity.empty()) return;

  // set position to current position if limit is reached
  SetPositionAndVelocity(cmd.position[cmd_ind], cmd.velocity[cmd_ind], cmd_ind);
  
}

void DMMotor::SetCurrent(double current, uint8_t id) const {
  if (coord_system_ == CoordinateSystem::LeftHand) {
    current = -current;
  }
  current = std::clamp(current, -rate_torque_, rate_torque_);
  protocol_->SetCurrent(id, current);
}

void DMMotor::SetPositionAndVelocity(double pos, double vel, uint8_t id) {

  double pos_err = pos - position_;
  pos_set_ = pos;
  double vel_cmd = vel;
  double pos_set_send = pos_set_;

  if (coord_system_ == CoordinateSystem::LeftHand) {
    vel_cmd = -vel_cmd;
    pos_set_send = -pos_set_;
  }
  
  protocol_->SetPosition(id, pos_set_send);
  protocol_->SetVelocity(id, vel_cmd);
}

void DMMotor::StopMotor(uint8_t id) const {
  protocol_->SetKp(id, 10);
  protocol_->SetKd(id, 0.1);
  if (id >2) {
    protocol_->SetKp(id, 0.3);
    protocol_->SetKd(id, 0.5);
  }
  protocol_->SetPosition(id, pos_limit_);
  protocol_->SetCurrent(id, 0);
  protocol_->SetVelocity(id, 0);
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