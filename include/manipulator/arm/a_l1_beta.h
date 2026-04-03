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
  // Delete copy and move constructors for singleton pattern
  AL1Beta(const AL1Beta&) = delete;
  AL1Beta& operator=(const AL1Beta&) = delete;
  AL1Beta(AL1Beta&&) = delete;
  AL1Beta& operator=(AL1Beta&&) = delete;

  /**
   * @brief Get singleton instance of AL1Beta
   * @return Reference to the singleton instance
   */
  static AL1Beta& Instance() {
    static AL1Beta instance;
    return instance;
  }

  /**
   * @brief Initialize arm hardware
   * @param port Serial port device path (e.g., "/dev/ttyUSB0")
   * @param baud Baud rate for serial communication
   */
  void Init(const std::string& port, uint32_t baud);  

 private:
  /**
   * @brief Private constructor for singleton pattern
   */
  AL1Beta();

  /**
   * @brief Protocol handler for serial communication
   */
  protocol::ProtocolV1::SharedPtr protocol_;
};

} // namespace manipulator::arm
