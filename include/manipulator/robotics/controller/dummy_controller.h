#pragma once
#include <arm_platform/controller/i_arm_controller.h>
#include <arm_platform/arm_model/i_arm_model.h>
#include <arm_platform/protocol/i_link_protocol.h>
#include <arm_platform/bus/i_motor_bus.h>

namespace arm_platform::controller {

class DummyController : public IArmController {
public:
    DummyController(std::shared_ptr<arm_platform::arm_model::IArmModel> model,
                    std::shared_ptr<arm_platform::protocol::ILinkProtocol> protocol,
                    std::shared_ptr<arm_platform::bus::IMotorBus> bus);

    void SetCommand(const arm_platform::JointCommand& cmd) override;
    void Update(arm_platform::JointStateArray& state) override;

private:
    std::shared_ptr<arm_platform::arm_model::IArmModel> model_;
    std::shared_ptr<arm_platform::protocol::ILinkProtocol> protocol_;
    std::shared_ptr<arm_platform::bus::IMotorBus> bus_;
    std::vector<double> target_pos_;
};

} // namespace arm_platform::controller
