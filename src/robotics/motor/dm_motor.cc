#include <manipulator/robotics/motor/dm_motor.h>
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
  
  is_received_ = true;
}

void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (not is_received_) return;

  uint8_t cmd_ind = id_;
  if (cmd.position.size() == 1) cmd_ind = 0;
  
  double current = cmd.current[cmd_ind];
  if (coord_system_ == CoordinateSystem::LeftHand) {
    current = -current;
  }
  protocol_->SetKp(cmd_ind, cmd.p[cmd_ind]);
  protocol_->SetKd(cmd_ind, cmd.d[cmd_ind]);
  protocol_->SetCurrent(cmd_ind, current);

  if(cmd.position.empty()) return;

  double pos_err = cmd.position[cmd_ind] - position_;
  // if (abs(pos_err) < 0.01) {
  //   return;
  // }

  pos_set_ = cmd.position[cmd_ind];
  
  double vel_cmd = cmd.velocity[cmd_ind];
  double pos_set_send = pos_set_;

  if (coord_system_ == CoordinateSystem::LeftHand) {
    vel_cmd = -vel_cmd;
    pos_set_send = -pos_set_;
  }
  
  protocol_->SetPosition(cmd_ind, pos_set_send);
  protocol_->SetVelocity(cmd_ind, vel_cmd);
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
