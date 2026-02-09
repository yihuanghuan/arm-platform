#include <manipulator/arm/abs_arm.h>

namespace manipulator::arm {
AbsArm::AbsArm() {
  
}
AbsArm::AbsArm(bus::IBus::UniquePtr bus) : bus_(std::move(bus)){

}

void AbsArm::SetBus(bus::IBus::UniquePtr bus) {
  bus_ = std::move(bus);
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
      // state.effort.push_back(states.effort[index]);
      motor->UpdateCommand(state);
    }
  }
  bus_->Send(); 
  return true;
}

void AbsArm::AddMotor(const std::string& name, motor::IMotor::SharedPtr motor) {
  motor_map_.emplace(name, motor);
}

void AbsArm::RemoveMotor(const std::string& name) {
  motor_map_.erase(name);
}

void AbsArm::GetJointStates() {
  bus_->Read();
}
}