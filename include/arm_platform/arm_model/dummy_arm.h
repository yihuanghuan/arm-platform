#pragma once

#include <arm_platform/arm_model/i_arm_model.h>

namespace arm_platform::arm_model {

class DummyArm : public IArmModel {
public:
    DummyArm() {}
    size_t Dof() const override {
        return 7;
    }
    // std::vector<motor::JointState> getJointStates() const override;

// private:
//     std::vector<std::shared_ptr<motor::IMotor>> motors_;
};

} // namespace arm_platform::arm_model
