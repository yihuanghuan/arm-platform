#include <manipulator/motor/dm_motor.h>
#include <rclcpp/rclcpp.hpp>

auto logger = rclcpp::get_logger("Controller");
namespace manipulator::motor {
DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
                 float kp, float kd, bool is_mirror)
 : protocol_(protocol), id_(id), default_kp_(kp), default_kd_(kd), vel_set_(0), is_mirror_(is_mirror) {

}

void DMMotor::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  // avoid sudden jitter caused by incorrent initial set value
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_); // DM motors return torque instead of current
  temperature_ = protocol_->GetTemperature(id_);
  if(is_mirror_) {
    torque_ = -torque_;
    position_ = -position_;
    velocity_ = -velocity_;
  }
}

void DMMotor::SetState(const sensor_msgs::msg::JointState& state) {
  dummy_interface::msg::MotorControl cmd;

  cmd.position.push_back(is_mirror_ ? -state.position[0] : state.position[0]);
  cmd.velocity.push_back(is_mirror_ ? -state.velocity[0] : state.velocity[0]);
  cmd.p.push_back(default_kp_);
  cmd.d.push_back(default_kd_);
  UpdateCommand(cmd);
}

void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (not is_received_) return;
  // Use constant Kp and Kd values; only a one-time publication is needed.
  protocol_->SetKp(id_, cmd.p[id_]);
  protocol_->SetKd(id_, cmd.d[id_]);
  
  double pos_err = cmd.position[id_] - pos_set_;
  if (abs(pos_err) < 0.01 and abs(vel_set_) < 0.1) {
    vel_set_ = 0;
    return;
  }

  double decel_distance = (vel_set_ * vel_set_) / (2 * MAX_ACCELERATION_);
  float direction = (pos_err > 0) ? 1.0f : -1.0f;

  if (abs(pos_err) <= decel_distance + 0.01) {
    // deceleration
    if (abs(vel_set_) > 0.01) {
      vel_set_ -= direction * MAX_ACCELERATION_ * DT_;
      if (direction * vel_set_ < 0) vel_set_ = 0;
    }
  } else {
    if(abs(vel_set_) < MAX_VELOCITY_) {
      vel_set_ += direction * MAX_ACCELERATION_ * DT_;
      vel_set_ = std::min(std::max(-MAX_VELOCITY_, vel_set_), MAX_VELOCITY_);
    }
  }

  if(id_ < 6) pos_set_ +=  vel_set_ * DT_;
  else pos_set_ = cmd.position[id_]; // joint7(gripper) requires rapid movement
  protocol_->SetPosition(id_, is_mirror_ ? -cmd.position[id_] : cmd.position[id_]);
  protocol_->SetVelocity(id_, is_mirror_ ? -cmd.velocity[id_] : cmd.velocity[id_]);
  protocol_->SetCurrent(id_, is_mirror_ ? -cmd.current[id_] : cmd.current[id_]);

}
}