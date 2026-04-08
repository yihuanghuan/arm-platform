#include <manipulator/protocol/protocol_factory.h>

namespace manipulator::protocol {
void ProtocolFactory::Add(std::vector<uint8_t> header, IProtocol::SharedPtr protocol)
{
    protocols_[header] = protocol;
}

IProtocol::SharedPtr ProtocolFactory::GetProtocol(const std::string& protocol_name,
                                                 const std::vector<uint8_t>& header) {
  if (auto it = protocols_.find(header); it != protocols_.end()) {
      return it->second;
  }
  return nullptr;
}

}