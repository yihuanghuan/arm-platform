#include <manipulator/bus/abs_bus.h>

namespace manipulator::bus {
AbsBus::AbsBus(protocol::ProtocolFactory::UniquePtr protocol_factory) 
 : protocol_factory_(std::move(protocol_factory)) {
  
}
void AbsBus::Send() {
  protocol_factory_->IterateEach([this](const auto& name, auto& protocol) {
    std::vector<uint8_t> bytes;
    protocol->Pop(bytes);
    SendCore(bytes);
  });
}

void AbsBus::Read() {
  std::vector<uint8_t> bytes;
  ReadCore(bytes);
  protocol_factory_->IterateEach([this, &bytes](const auto& name, auto& protocol) {
    for(auto byte : bytes) protocol->Feed(byte); //TODO: this is not right when there are multiple protocols
  });
}

}