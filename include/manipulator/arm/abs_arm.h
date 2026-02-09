#pragma once
#include <vector>
#include <memory>
#include <map>
#include <string>

#include <sensor_msgs/msg/joint_state.hpp>

#include <manipulator/arm/i_arm.h>
#include <manipulator/motor/i_motor.h>
#include <manipulator/bus/i_bus.h>

namespace manipulator::arm {

class AbsArm : public IArm {
 public:
  AbsArm();
  explicit AbsArm(bus::IBus::UniquePtr bus);
    
  // 禁用拷贝（因为UniquePtr不可拷贝）
  AbsArm(const AbsArm&) = delete;
  AbsArm& operator=(const AbsArm&) = delete;   
  ~AbsArm() = default;

  void SetBus(bus::IBus::UniquePtr bus);

  // IArm interface implementation
  bool SetJointStates(const sensor_msgs::msg::JointState& state) override;
  // TODO：return joint states if necessary
  virtual void GetJointStates() = 0;

  // Additional methods
  void AddMotor(const std::string& name, motor::IMotor::SharedPtr motor);
  void RemoveMotor(const std::string& name);

// protected:
//     virtual bool CheckJointLimits() const;
 private:
  bus::IBus::UniquePtr bus_;
  // std::vector<JointLimit> joint_limits_;
  std::map<std::string, motor::IMotor::SharedPtr> motor_map_;

};

} // namespace manipulator::arm