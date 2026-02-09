#pragma once
#include <vector>
#include <memory>
#include <manipulator/protocol/abs_protocol.h>
// #include "rclcpp/rclcpp.hpp"

namespace manipulator::protocol {

class ProtocolV1 : public AbsProtocol {
 public:
  using SharedPtr = std::shared_ptr<ProtocolV1>;
  ProtocolV1();
  virtual ~ProtocolV1() = default;

  // IProtocol interface implementation
  void Pop(std::vector<uint8_t>& data) override;
  void Feed(const uint8_t byte) override;

  double GetPosition(const uint8_t id) const { return positions_[id]; }
  double GetVelocity(const uint8_t id) const { return velocities_[id]; }
  double GetCurrent(const uint8_t id) const { return currents_[id]; }
  double GetTemperature(const uint8_t id) const {return temperatures_[id]; }
  double GetVoltage(const uint8_t id) const { return voltages_[id]; }

  void SetPosition(const uint8_t id, const double position) { desired_positions_[id] = position; pos_cmd_updated_=true; }
  void SetVelocity(const uint8_t id, const double velocity) { desired_velocities_[id] = velocity; vel_cmd_updated_=true; }
  void SetCurrent(const uint8_t id, const double current) { desired_currents_[id] = current; cur_cmd_updated_=true; }
  void SetKp(const uint8_t id, const double kp) { desired_kps_[id] = kp; kp_cmd_updated_=true; }
  void SetKd(const uint8_t id, const double kd) { desired_kds_[id] = kd; kd_cmd_updated_=true; }

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

  void MakeFrame(uint8_t cmd, const void* data, 
                 size_t len, std::vector<uint8_t>& out);


  std::vector<uint8_t> bytes_to_send_;
  LinkFrame recv_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> currents_;
  std::vector<double> temperatures_;
  std::vector<double> voltages_;

  std::vector<float> desired_positions_;
  std::vector<float> desired_velocities_;
  std::vector<float> desired_currents_;
  std::vector<float> desired_kps_;
  std::vector<float> desired_kds_;

  uint16_t recv_step_ = 0;
  bool pos_cmd_updated_ = false;
  bool vel_cmd_updated_ = false;
  bool cur_cmd_updated_ = false;
  bool kp_cmd_updated_ = false;
  bool kd_cmd_updated_ = false;

};
} // namespace manipulator::protocol