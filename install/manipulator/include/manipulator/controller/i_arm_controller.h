#pragma once
#include <memory>
#include <arm_platform/arm_types.h>

namespace arm_platform::controller {

class IArmController {
public:
    virtual ~IArmController() = default;

    // Set high-level command
    virtual void SetCommand(const arm_platform::JointCommand& cmd) = 0;

    // Update controller, generate low-level motor commands
    virtual void Update(arm_platform::JointStateArray& state) = 0;

    // Get current joint states
    // virtual arm_platform::JointStateArray getState() const = 0;
};
} // namespace arm_platform::controller
