// offset 0   : id_op_mode (uint8)
// offset 1   : servo_state (uint8)
// offset 2   : present_position (float64, 8 bytes)
// offset 10  : encoder_position (float64, 8 bytes)
// offset 18  : present_velocity (float64, 8 bytes)
// offset 26  : present_current (float64, 8 bytes)
// offset 34  : goal_position (float64, 8 bytes)
// offset 42  : goal_velocity (float64, 8 bytes)
// offset 50  : goal_current (float64, 8 bytes)
// offset 58  : error_state (uint8)

#include "dxl_controller.h"

#include <iostream>

DxlController::DxlController(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {}

DxlController::~DxlController() {
  close();
}

bool DxlController::open() {
  port_handler_   = dynamixel::PortHandler::getPortHandler(port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(CommConfig::PROTOCOL_VERSION);

  if (!port_handler_->openPort()) {
    std::cerr << "[DxlController] 포트 열기 실패: " << port_ << "\n";
    return false;
  }
  if (!port_handler_->setBaudRate(baudrate_)) {
    std::cerr << "[DxlController] Baudrate 설정 실패: " << baudrate_ << "\n";
    return false;
  }

  connected_ = true;
  std::cout << "[DxlController] 포트 연결 성공: " << port_
            << " @ " << baudrate_ << " bps\n";
  return true;
}

void DxlController::close() {
  if (connected_) {
    enableTorqueAll(false);
    port_handler_->closePort();
    connected_ = false;
    std::cout << "[DxlController] 포트 닫힘\n";
  }
}

bool DxlController::enableTorque(uint8_t id, bool enable) {
  uint8_t dxl_error = 0;
  int result = packet_handler_->write1ByteTxRx(
      port_handler_, id,
      XM540::ADDR_TORQUE_ENABLE,
      enable ? XM540::TORQUE_ENABLE : XM540::TORQUE_DISABLE,
      &dxl_error);

  if (result != COMM_SUCCESS) {
    std::cerr << "[DxlController] ID " << (int)id << " 토크 설정 통신 오류: "
              << packet_handler_->getTxRxResult(result) << "\n";
    return false;
  }
  if (dxl_error) {
    std::cerr << "[DxlController] ID " << (int)id << " 토크 설정 하드웨어 오류: "
              << packet_handler_->getRxPacketError(dxl_error) << "\n";
    return false;
  }
  return true;
}

bool DxlController::enableTorqueAll(bool enable) {
  bool ok = true;
  for (uint8_t id : MOTOR_IDS) {
    ok &= enableTorque(id, enable);
  }
  return ok;
}

bool DxlController::setOperatingMode(uint8_t id, uint8_t mode) {
  // Operating Mode 변경 전 반드시 토크 비활성화
  enableTorque(id, false);

  uint8_t dxl_error = 0;
  int result = packet_handler_->write1ByteTxRx(
      port_handler_, id,
      XM540::ADDR_OPERATING_MODE, mode, &dxl_error);

  if (result != COMM_SUCCESS) {
    std::cerr << "[DxlController] ID " << (int)id << " 모드 설정 통신 오류: "
              << packet_handler_->getTxRxResult(result) << "\n";
    return false;
  }
  if (dxl_error) {
    std::cerr << "[DxlController] ID " << (int)id << " 모드 설정 하드웨어 오류: "
              << packet_handler_->getRxPacketError(dxl_error) << "\n";
    return false;
  }
  return true;
}

bool DxlController::setOperatingModeAll(uint8_t mode) {
  bool ok = true;
  for (uint8_t id : MOTOR_IDS) {
    ok &= setOperatingMode(id, mode);
  }
  return ok;
}

bool DxlController::setGoalPosition(uint8_t id, int32_t position) {
  uint8_t dxl_error = 0;
  int result = packet_handler_->write4ByteTxRx(
      port_handler_, id,
      XM540::ADDR_GOAL_POSITION,
      static_cast<uint32_t>(position),
      &dxl_error);

  if (result != COMM_SUCCESS) {
    std::cerr << "[DxlController] ID " << (int)id << " 목표 위치 전송 오류: "
              << packet_handler_->getTxRxResult(result) << "\n";
    return false;
  }
  if (dxl_error) {
    std::cerr << "[DxlController] ID " << (int)id << " 목표 위치 하드웨어 오류: "
              << packet_handler_->getRxPacketError(dxl_error) << "\n";
    return false;
  }
  return true;
}

bool DxlController::setGoalPositionAll(
    const std::array<int32_t, CommConfig::NUM_MOTORS>& positions) {
  bool ok = true;
  for (int i = 0; i < CommConfig::NUM_MOTORS; ++i) {
    ok &= setGoalPosition(MOTOR_IDS[i], positions[i]);
  }
  return ok;
}

int32_t DxlController::getPresentPosition(uint8_t id) {
  uint32_t raw     = 0;
  uint8_t  error   = 0;
  int result = packet_handler_->read4ByteTxRx(
      port_handler_, id,
      XM540::ADDR_PRESENT_POSITION, &raw, &error);

  if (result != COMM_SUCCESS || error) return -1;
  return static_cast<int32_t>(raw);
}

bool DxlController::getPresentPositionAll(
    std::array<int32_t, CommConfig::NUM_MOTORS>& positions) {
  bool ok = true;
  for (int i = 0; i < CommConfig::NUM_MOTORS; ++i) {
    int32_t pos = getPresentPosition(MOTOR_IDS[i]);
    if (pos < 0) ok = false;
    positions[i] = pos;
  }
  return ok;
}
