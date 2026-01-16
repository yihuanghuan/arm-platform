#include <arm_platform/bus/serial_bus.h>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace arm_platform::bus {

SerialBus::SerialBus(const std::string& port_name, uint32_t baudrate) {
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial_.setPort(port_name);
    serial_.setBaudrate(baudrate);
    serial_.setTimeout(to);

    try {
        serial_.open();
    } catch (serial::IOException& e) {
        throw std::runtime_error("Unable to open serial port " + port_name);
    }

}

SerialBus::~SerialBus() {
  if (serial_.isOpen()) serial_.close();
}

void SerialBus::Send(const std::vector<uint8_t>& data) {
  if (serial_.isOpen()) {
    serial_.write(data.data(), data.size());
  }
}

std::vector<uint8_t> SerialBus::Receive() {
    size_t avail = serial_.available();
    std::vector<uint8_t> buf(avail);
    serial_.read(buf.data(), avail);

    return buf;
}

}  // namespace arm_platform
