#pragma once

#include <manipulator/arm/abs_arm.h>
#include <manipulator/protocol/protocol_v1.h>
namespace manipulator::arm {

class AL1Beta final : public AbsArm {
 public:
  struct JointState {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> current;
    std::vector<double> voltage;
    std::vector<double> temperature;

    JointState() : position(7), velocity(7), current(7),
      voltage(7), temperature(7) {}
  };

  AL1Beta(const AL1Beta&) = delete;
  AL1Beta& operator=(const AL1Beta&) = delete;
  AL1Beta(AL1Beta&&) = delete;
  AL1Beta& operator=(AL1Beta&&) = delete;

  static AL1Beta& Instance() {
    static AL1Beta instance;
    return instance;
  }

  void Init(const std::string& port, uint32_t baud);
  // void GetJointStates() override {}
  void GetState(JointState& state);  

 private:
  AL1Beta();
  protocol::ProtocolV1::SharedPtr protocol_;

};

} // namespace manipulator::arm
