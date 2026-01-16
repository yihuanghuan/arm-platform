#pragma once
#include <vector>
#include <cstdint>

namespace arm_platform::bus {

class IMotorBus {
public:
    virtual ~IMotorBus() = default;

    // Send raw bytes to motor
    virtual void Send(const std::vector<uint8_t>& data) = 0;

    // Receive raw bytes from motor
    virtual std::vector<uint8_t> Receive() = 0;
};
} // namespace arm_platform::bus
