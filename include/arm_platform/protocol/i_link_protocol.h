#pragma once
#include <cstdint>
#include <arm_platform/arm_types.h>

namespace arm_platform::protocol {

class ILinkProtocol {
public:
    virtual ~ILinkProtocol() = default;

    virtual bool Feed(uint8_t byte) = 0;
    virtual bool HasFrame() const = 0;
    virtual const JointStateArray& PopFrame() = 0;
    virtual std::vector<uint8_t> MakeCmd(const std::vector<double>& data, uint8_t cmd_id) = 0;
};

} // namespace arm_platform::protocol
