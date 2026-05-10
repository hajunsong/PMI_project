#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <dynamixel_sdk/dynamixel_sdk.h>

// ── XM540-W270 제어 테이블 주소 (Protocol 2.0) ────────────────────────────────
namespace XM540 {
  constexpr uint16_t ADDR_OPERATING_MODE        = 11;
  constexpr uint16_t ADDR_TORQUE_ENABLE         = 64;
  constexpr uint16_t ADDR_LED                   = 65;
  constexpr uint16_t ADDR_GOAL_VELOCITY         = 104;
  constexpr uint16_t ADDR_PROFILE_ACCELERATION  = 108;
  constexpr uint16_t ADDR_PROFILE_VELOCITY      = 112;
  constexpr uint16_t ADDR_GOAL_POSITION         = 116;
  constexpr uint16_t ADDR_PRESENT_VELOCITY      = 128;
  constexpr uint16_t ADDR_PRESENT_POSITION      = 132;
  constexpr uint16_t ADDR_PRESENT_TEMPERATURE   = 146;

  constexpr uint8_t  TORQUE_ENABLE              = 1;
  constexpr uint8_t  TORQUE_DISABLE             = 0;

  constexpr uint8_t  OP_POSITION               = 3;
  constexpr uint8_t  OP_VELOCITY               = 1;
  constexpr uint8_t  OP_CURRENT_CTRL_POSITION  = 5;

  constexpr int32_t  MIN_POSITION              = 0;
  constexpr int32_t  MAX_POSITION              = 4095;
  constexpr int32_t  CENTER_POSITION           = 2048;
}

// ── 통신 설정 ────────────────────────────────────────────────────────────────
namespace CommConfig {
  constexpr int      PROTOCOL_VERSION  = 2;
  constexpr int      BAUDRATE          = 1000000;
  constexpr float    COMM_SUCCESS_RATE = 0.0f;
  constexpr int      NUM_MOTORS        = 4;
}

// ── 모터 ID 목록 ──────────────────────────────────────────────────────────────
constexpr std::array<uint8_t, CommConfig::NUM_MOTORS> MOTOR_IDS = {1, 2, 3, 4};

// ── DxlController ─────────────────────────────────────────────────────────────
class DxlController {
public:
  explicit DxlController(const std::string& port, int baudrate = CommConfig::BAUDRATE);
  ~DxlController();

  bool open();
  void close();

  bool enableTorque(uint8_t id, bool enable);
  bool enableTorqueAll(bool enable);

  bool setOperatingMode(uint8_t id, uint8_t mode);
  bool setOperatingModeAll(uint8_t mode);

  bool setGoalPosition(uint8_t id, int32_t position);
  bool setGoalPositionAll(const std::array<int32_t, CommConfig::NUM_MOTORS>& positions);

  int32_t getPresentPosition(uint8_t id);
  bool    getPresentPositionAll(std::array<int32_t, CommConfig::NUM_MOTORS>& positions);

  bool isConnected() const { return connected_; }

private:
  std::string                port_;
  int                        baudrate_;
  bool                       connected_ = false;

  dynamixel::PortHandler*    port_handler_   = nullptr;
  dynamixel::PacketHandler*  packet_handler_ = nullptr;
};
