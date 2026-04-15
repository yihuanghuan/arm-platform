#pragma once
#include <map>
#include <vector>
#include <memory>
#include <manipulator/robotics/protocol/i_protocol.h>

namespace manipulator::protocol {
class ProtocolFactory {
 public:
  using UniquePtr = std::unique_ptr<ProtocolFactory>;

  ProtocolFactory() = default;
  IProtocol::SharedPtr GetProtocol(const std::string& protocol_name,
                                   const std::vector<uint8_t>& header);
  void Add(std::vector<uint8_t> header, IProtocol::SharedPtr protocol);

  template <typename F>
  void IterateEach(F&& f) {
    for (auto& [name, protocol] : protocols_) {
      f(name, protocol);
    }
  }
  
 private:
  std::map<std::vector<uint8_t>, IProtocol::SharedPtr> protocols_;
}; 
}