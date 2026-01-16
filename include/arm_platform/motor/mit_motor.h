#pragma once
#include <arm_platform/motor/i_motor.h>

namespace arm_platform::motor {

class MITMotor : public IMotor {
public:
    MITMotor(int id);
    void setPosition(double pos) override;
    void setVelocity(double vel) override;
    void setCurrent(double cur) override;
    void setPID(double P, double D) override;
    void updateState() override;
    JointState getJointState() const override;
    int id() const override;

private:
    int id_;
    JointState state_;
    double P_, D_;
    double target_pos_, target_vel_, target_cur_;
};

} // namespace arm_platform::motor
