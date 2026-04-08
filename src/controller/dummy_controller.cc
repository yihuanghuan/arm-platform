#include <arm_platform/controller/dummy_controller.h>

#include <algorithm>
#include <chrono>
#include <thread>

// TODO：this inject dependency to concrete protocol, need to be removed later
#include <arm_platform/protocol/dummy_link_protocol.h>

namespace arm_platform::controller {

using DummyCmdID = arm_platform::protocol::DummyLinkProtocol::WriteCmdId;

DummyController::DummyController(
    std::shared_ptr<arm_model::IArmModel> model,
    std::shared_ptr<protocol::ILinkProtocol> protocol,
    std::shared_ptr<bus::IMotorBus> bus)
    : model_(std::move(model)),
      protocol_(std::move(protocol)),
      bus_(std::move(bus)),
      target_pos_(model_->Dof(), 0.0) {}

// Set a desired command (position/velocity/current)
void DummyController::SetCommand(const JointCommand& cmd) {
    // Send command to protocol
    if (!cmd.position.empty()) {
        auto data = protocol_->MakeCmd(cmd.position, static_cast<uint8_t>(DummyCmdID::kPositionSet));
        bus_->Send(data);
    }

    if (!cmd.velocity.empty()) {
        auto vel_data = protocol_->MakeCmd(cmd.velocity, static_cast<uint8_t>(DummyCmdID::kVelocitySet));
        bus_->Send(vel_data);
    }

    if (!cmd.current.empty()) {
        auto cur_data = protocol_->MakeCmd(cmd.current, static_cast<uint8_t>(DummyCmdID::kCurrentSet));
        bus_->Send(cur_data);
    }

    // Optional: P/D gains if protocol supports
    if (!cmd.p.empty()) {
        auto p_data = protocol_->MakeCmd(cmd.p, static_cast<uint8_t>(DummyCmdID::kPSet));
        bus_->Send(p_data);
    }
    if (!cmd.d.empty()) {
        auto d_data = protocol_->MakeCmd(cmd.d, static_cast<uint8_t>(DummyCmdID::kDSet));
        bus_->Send(d_data);
    }
}

// Update internal state from protocol
void DummyController::Update(arm_platform::JointStateArray& state) {
  // If protocol has a new frame, decode it
    auto buf = bus_->Receive();
    for (auto byte : buf) {
        protocol_->Feed(byte);
    }
    if(protocol_->HasFrame()) {
        state = protocol_->PopFrame();
        // model_->UpdateState(state);
    }
}

// Return current joint states
// JointStateArray DummyController::GetState() const {
//     return model_->GetState();
// }

}  // namespace arm_platform::controller
