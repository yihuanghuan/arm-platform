#pragma once
#include <vector>
#include <memory>

namespace arm_platform::motor {

class IMotor {
public:
    virtual ~IMotor() = default;

    // Set control commands (optional: only set what this motor supports)
    virtual void setPosition(double pos) = 0;
    virtual void setVelocity(double vel) = 0;
    virtual void setCurrent(double cur) = 0;
    virtual void setPID(double P, double D) = 0;

    // Update motor state from hardware feedback
    virtual void updateState() = 0;

    // Get current joint state
    virtual JointState getJointState() const = 0;

    // Motor unique ID
    virtual int id() const = 0;
};

} // namespace arm_platform::motor
