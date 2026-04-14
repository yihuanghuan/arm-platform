#pragma once

#include <manipulator/arm/abs_arm.h>
#include <manipulator/protocol/protocol_v1.h>

namespace manipulator::arm {

/**
 * @brief AL1Beta robotic arm implementation
 * 
 * This class implements the AL1Beta 7-DOF robotic arm
 * with serial communication protocol. It provides singleton
 * pattern for hardware access and manages joint state feedback.
 */
class AL1Beta final : public AbsArm {
 public:
  AL1Beta();

  /**
   * @brief Initialize arm hardware
   * @param port Serial port device path (e.g., "/dev/ttyUSB0")
   * @param baud Baud rate for serial communication
   */
  void Init(const std::string& port, uint32_t baud); 

 private:
  protocol::ProtocolV1::SharedPtr protocol_;
};

} // namespace manipulator::arm
