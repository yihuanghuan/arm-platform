#pragma once
#include <vector>
#include <memory>
#include <manipulator/bus/i_bus.h>
#include <manipulator/protocol/protocol_factory.h>

namespace manipulator::bus {

class AbsBus : public IBus {
 public:
  AbsBus(protocol::ProtocolFactory::UniquePtr protocol_factory);
  virtual ~AbsBus() = default;
  // IBus interface implementation
  void Send() override final;
  void Read() override final;

 protected:
  virtual void SendCore(const std::vector<uint8_t>& data) = 0;
  virtual void ReadCore(std::vector<uint8_t>& data) = 0;

 private:
  protocol::ProtocolFactory::UniquePtr protocol_factory_;
};

} // namespace manipulator::bus