#include <manipulator/motor/dm_motor.h>

namespace manipulator::motor {
DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, double kp, double kd)
 : protocol_(protocol), id_(id), kp_(kp), kd_(kd), vel_set_(0) {

}

void DMMotor::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  // avoid sudden jitter caused by incorrent initial set value
  if (not is_controlled_) pos_set_ = position_;
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_); // DM motors return torque instead of current
  temperature_ = protocol_->GetTemperature(id_);
}

void DMMotor::UpdateCommand(const sensor_msgs::msg::JointState& state) {
  if (not is_controlled_) {
    // Use constant Kp and Kd values; only a one-time publication is needed.
    protocol_->SetKp(id_, kp_);
    protocol_->SetKd(id_, kd_);
  }
  is_controlled_ = true;

  double pos_err = state.position[0] - pos_set_;
  if (abs(pos_err) < 0.01  and abs(vel_set_) < 0.1) {
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
  else pos_set_ = state.position[0]; // joint7(gripper) requires rapid movement
  protocol_->SetPosition(id_, pos_set_);
  protocol_->SetVelocity(id_, vel_set_);
  // protocol_->SetCurrent(id_, state.effort[0]);

}
}