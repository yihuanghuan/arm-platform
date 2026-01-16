#pragma once
#include <arm_platform/protocol/i_link_protocol.h>
#include <array>
#include <vector>
#include <mutex>

namespace arm_platform::protocol {

/**
 * MotorA binary protocol implementation.
 * This is a full refactor of your original Link_* logic.
 */
class DummyLinkProtocol : public ILinkProtocol {
public:
    DummyLinkProtocol();

     // ----------------------------
    // Public enum for write command IDs
    // ----------------------------
    enum class WriteCmdId : uint8_t {
        kPositionSet = 0x10,  // Send position
        kVelocitySet = 0x11,  // Send velocity
        kCurrentSet  = 0x12,  // Send current
        kPSet        = 0x13,  // Send P gain
        kDSet        = 0x14,  // Send D gain
        kHeartbeat   = 0x01   // Heartbeat
    };

    bool Feed(uint8_t byte) override;
    bool HasFrame() const override;
    const JointStateArray& PopFrame() override;

    std::vector<uint8_t> MakeCmd(const std::vector<double>& data, uint8_t cmd_id) override;
private:
    struct LinkFrame {
        uint8_t head = 0xFE;
        uint8_t addr = 0x55;
        uint8_t cmd = 0;
        uint8_t dataLen = 0;
        uint8_t dataBuf[128];
        uint8_t sumCheck = 0;
        uint8_t addCheck = 0;
    };

    void Check(LinkFrame& f);
    void DecodeFrame();

    void DecodePosition();
    void DecodeVelocity();
    void DecodeCurrent();
    void DecodeVoltage();
    void DecodeTemperature();
    void DecodeHeartbeat();

    std::vector<uint8_t> MakePositionCmd(const std::vector<double>& pos);
    std::vector<uint8_t> MakeVelocityCmd(const std::vector<double>& vel);
    std::vector<uint8_t> MakeCurrentCmd(const std::vector<double>& cur);
    std::vector<uint8_t> MakePCmd(const std::vector<double>& p);
    std::vector<uint8_t> MakeDCmd(const std::vector<double>& d);
    std::vector<uint8_t> MakeHeartbeatCmd();

    LinkFrame recv_;
    uint16_t recv_step_ = 0;

    JointStateArray last_state_;
    bool has_new_frame_ = false;

    // Heartbeat monitoring
    size_t heartbeat_count_ = 0;
    uint8_t connect_flag_ = 0;
};

}
