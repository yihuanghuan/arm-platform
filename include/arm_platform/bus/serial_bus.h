#pragma once
#include <arm_platform/bus/i_motor_bus.h>

#include <serial/serial.h>

namespace arm_platform::bus {

class SerialBus : public IMotorBus {
public:
    SerialBus(const std::string& port, uint32_t baud);
    ~SerialBus() override;
    void Send(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> Receive() override;

private:
    serial::Serial serial_;
};

} // namespace arm_platform::bus
