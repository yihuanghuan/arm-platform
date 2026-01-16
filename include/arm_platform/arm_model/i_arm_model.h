#pragma once
#include <vector>
#include <memory>
// #include <arm_platform/i_motor.h>

namespace arm_platform::arm_model {

class IArmModel {
public:
    virtual ~IArmModel() = default;

    // Degrees of freedom
    virtual size_t Dof() const = 0;

    // Access joint states
    // virtual std::vector<motor::JointState> getJointStates() const = 0;
};

} // namespace arm_platform::arm_model
