#pragma once
#include <manipulator/bus/abs_bus.h>

#include <serial/serial.h>

namespace manipulator::bus {

class SerialBus : public AbsBus {
public:
    SerialBus(const std::string& port, uint32_t baud, 
              protocol::ProtocolFactory::UniquePtr protocol_factory);
    ~SerialBus() override;
    void SendCore(const std::vector<uint8_t>& data) override;
    void ReadCore(std::vector<uint8_t>& data) override;

private:
    serial::Serial serial_;
};

} // namespace manipulator::bus
