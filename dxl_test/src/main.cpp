#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace XM540 {
constexpr float PROTOCOL_VERSION = 2.0f;
constexpr int BAUDRATE = 2000000;
constexpr const char* DEVICENAME = "/dev/ttyUSB0";

constexpr uint8_t TORQUE_ENABLE = 1;
constexpr uint8_t TORQUE_DISABLE = 0;
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;

// XM540-W270 Control Table (Protocol 2.0)
constexpr uint16_t ADDR_HARDWARE_ERROR_STATUS = 70;  // 1 byte
constexpr uint16_t ADDR_GOAL_CURRENT = 102;          // 2 bytes (102~103)
constexpr uint16_t ADDR_GOAL_VELOCITY = 104;         // 4 bytes (104~107)
constexpr uint16_t ADDR_GOAL_POSITION = 116;         // 4 bytes (116~119)
constexpr uint16_t ADDR_MOVING = 122;                // 1 byte
constexpr uint16_t ADDR_PRESENT_CURRENT = 126;       // 2 bytes (126~127)
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;      // 4 bytes (128~131)
constexpr uint16_t ADDR_PRESENT_POSITION = 132;      // 4 bytes (132~135)

constexpr uint16_t ADDR_INDIRECT_ADDRESS_1 = 168;  // 2 bytes each
constexpr uint16_t ADDR_INDIRECT_DATA_1 = 224;     // mapped read window

constexpr uint16_t LEN_TORQUE_ENABLE = 1;
constexpr uint16_t LEN_PRESENT_POSITION = 4;
constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
constexpr uint16_t LEN_PRESENT_CURRENT = 2;
constexpr uint16_t LEN_HARDWARE_ERROR_STATUS = 1;
constexpr uint16_t LEN_GOAL_POSITION = 4;
constexpr uint16_t LEN_GOAL_VELOCITY = 4;
constexpr uint16_t LEN_GOAL_CURRENT = 2;
constexpr uint16_t LEN_MOVING = 1;

constexpr uint16_t LEN_SYNC_READ_INDIRECT =
    LEN_TORQUE_ENABLE + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY +
    LEN_PRESENT_CURRENT + LEN_HARDWARE_ERROR_STATUS + LEN_GOAL_POSITION +
    LEN_GOAL_VELOCITY + LEN_GOAL_CURRENT + LEN_MOVING;
}  // namespace XM540

constexpr std::array<uint8_t, 4> MOTOR_IDS = {1, 2, 3, 4};
constexpr const char* READ_PERIOD_CONFIG_PATH =
    "/home/keti/PMI_project/dxl_test/read_period_ms.conf";
constexpr int DEFAULT_READ_PERIOD_MS = 10;

struct MotorSnapshot {
  uint8_t torque_enabled = 0;
  int32_t present_position = 0;
  int32_t present_velocity = 0;
  int16_t present_current = 0;
  uint8_t hardware_error_status = 0;
  int32_t goal_position = 0;
  int32_t goal_velocity = 0;
  int16_t goal_current = 0;
  uint8_t moving = 0;
  bool valid = false;
};

struct PeriodStats {
  uint64_t cycles = 0;
  uint64_t issues = 0;
};

bool setTorque(dynamixel::PortHandler* port,
               dynamixel::PacketHandler* packet,
               uint8_t id,
               bool enable) {
  uint8_t dxl_error = 0;
  const int result = packet->write1ByteTxRx(
      port, id, XM540::ADDR_TORQUE_ENABLE,
      enable ? XM540::TORQUE_ENABLE : XM540::TORQUE_DISABLE, &dxl_error);

  if (result != COMM_SUCCESS) {
    std::cerr << "[ID " << static_cast<int>(id) << "] Torque 통신 오류: "
              << packet->getTxRxResult(result) << '\n';
    return false;
  }
  if (dxl_error != 0) {
    std::cerr << "[ID " << static_cast<int>(id) << "] Torque 패킷 오류: "
              << packet->getRxPacketError(dxl_error) << '\n';
    return false;
  }
  return true;
}

bool mapIndirectByteRange(dynamixel::PortHandler* port,
                          dynamixel::PacketHandler* packet,
                          uint8_t id,
                          uint16_t& indirect_addr,
                          uint16_t source_addr,
                          uint16_t length) {
  for (uint16_t i = 0; i < length; ++i) {
    const uint16_t target_addr = source_addr + i;

    uint8_t dxl_error = 0;
    const int result = packet->write2ByteTxRx(
        port, id, indirect_addr, target_addr, &dxl_error);

    if (result != COMM_SUCCESS) {
      std::cerr << "[ID " << static_cast<int>(id)
                << "] Indirect Address 매핑 통신 오류: "
                << packet->getTxRxResult(result) << '\n';
      return false;
    }
    if (dxl_error != 0) {
      std::cerr << "[ID " << static_cast<int>(id)
                << "] Indirect Address 매핑 패킷 오류: "
                << packet->getRxPacketError(dxl_error) << '\n';
      return false;
    }

    indirect_addr += 2;
  }
  return true;
}

bool setupIndirectForRequestedFields(dynamixel::PortHandler* port,
                                     dynamixel::PacketHandler* packet,
                                     uint8_t id) {
  uint16_t indirect_addr = XM540::ADDR_INDIRECT_ADDRESS_1;

  // 요청 순서:
  // 1) Torque Enable(64, 1B)
  // 2) Present Position(132, 4B)
  // 3) Present Velocity(128, 4B)
  // 4) Present Current(126, 2B)
  // 5) Hardware Error Status(70, 1B)
  // 6) Goal Position(116, 4B)
  // 7) Goal Velocity(104, 4B)
  // 8) Goal Current(102, 2B)
  // 9) Moving(122, 1B)
  bool ok = true;
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_TORQUE_ENABLE,
                             XM540::LEN_TORQUE_ENABLE);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_PRESENT_POSITION,
                             XM540::LEN_PRESENT_POSITION);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_PRESENT_VELOCITY,
                             XM540::LEN_PRESENT_VELOCITY);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_PRESENT_CURRENT,
                             XM540::LEN_PRESENT_CURRENT);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_HARDWARE_ERROR_STATUS,
                             XM540::LEN_HARDWARE_ERROR_STATUS);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_GOAL_POSITION,
                             XM540::LEN_GOAL_POSITION);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_GOAL_VELOCITY,
                             XM540::LEN_GOAL_VELOCITY);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_GOAL_CURRENT,
                             XM540::LEN_GOAL_CURRENT);
  ok &= mapIndirectByteRange(port, packet, id, indirect_addr,
                             XM540::ADDR_MOVING,
                             XM540::LEN_MOVING);
  return ok;
}

int16_t toSigned16(uint16_t raw) {
  return static_cast<int16_t>(raw);
}

int32_t toSigned32(uint32_t raw) {
  return static_cast<int32_t>(raw);
}

int loadReadPeriodFromConfig(int min_read_period_ms,
                             int max_read_period_ms,
                             int fallback_read_period_ms) {
  std::ifstream ifs(READ_PERIOD_CONFIG_PATH);
  if (!ifs.is_open()) {
    return fallback_read_period_ms;
  }

  int loaded_ms = fallback_read_period_ms;
  if (!(ifs >> loaded_ms)) {
    return fallback_read_period_ms;
  }

  if (loaded_ms < min_read_period_ms) return min_read_period_ms;
  if (loaded_ms > max_read_period_ms) return max_read_period_ms;
  return loaded_ms;
}

bool saveReadPeriodToConfig(int read_period_ms) {
  std::ofstream ofs(READ_PERIOD_CONFIG_PATH, std::ios::trunc);
  if (!ofs.is_open()) {
    std::cerr << "주기 설정 저장 실패: " << READ_PERIOD_CONFIG_PATH << '\n';
    return false;
  }
  ofs << read_period_ms << '\n';
  return true;
}

int main() {
  auto* port_handler = dynamixel::PortHandler::getPortHandler(XM540::DEVICENAME);
  auto* packet_handler =
      dynamixel::PacketHandler::getPacketHandler(XM540::PROTOCOL_VERSION);

  if (!port_handler->openPort()) {
    std::cerr << "포트 열기 실패: " << XM540::DEVICENAME << '\n';
    return -1;
  }
  if (!port_handler->setBaudRate(XM540::BAUDRATE)) {
    std::cerr << "Baudrate 설정 실패: " << XM540::BAUDRATE << '\n';
    port_handler->closePort();
    return -1;
  }

  bool ok = true;
  for (uint8_t id : MOTOR_IDS) {
    ok &= setTorque(port_handler, packet_handler, id, false);
    ok &= setupIndirectForRequestedFields(port_handler, packet_handler, id);
    ok &= setTorque(port_handler, packet_handler, id, true);
  }
  if (!ok) {
    std::cerr << "초기 설정 실패\n";
    for (uint8_t id : MOTOR_IDS) {
      setTorque(port_handler, packet_handler, id, false);
    }
    port_handler->closePort();
    return -1;
  }

  dynamixel::GroupSyncRead group_sync_read(
      port_handler, packet_handler,
      XM540::ADDR_INDIRECT_DATA_1,
      XM540::LEN_SYNC_READ_INDIRECT);

  for (uint8_t id : MOTOR_IDS) {
    if (!group_sync_read.addParam(id)) {
      std::cerr << "[ID " << static_cast<int>(id)
                << "] GroupSyncRead addParam 실패\n";
      for (uint8_t release_id : MOTOR_IDS) {
        setTorque(port_handler, packet_handler, release_id, false);
      }
      port_handler->closePort();
      return -1;
    }
  }

  constexpr uint16_t OFFSET_TORQUE_ENABLE = 0;
  constexpr uint16_t OFFSET_PRESENT_POSITION =
      OFFSET_TORQUE_ENABLE + XM540::LEN_TORQUE_ENABLE;     // 1
  constexpr uint16_t OFFSET_PRESENT_VELOCITY =
      OFFSET_PRESENT_POSITION + XM540::LEN_PRESENT_POSITION;  // 5
  constexpr uint16_t OFFSET_PRESENT_CURRENT =
      OFFSET_PRESENT_VELOCITY + XM540::LEN_PRESENT_VELOCITY;  // 9
  constexpr uint16_t OFFSET_HARDWARE_ERROR_STATUS =
      OFFSET_PRESENT_CURRENT + XM540::LEN_PRESENT_CURRENT;    // 11
  constexpr uint16_t OFFSET_GOAL_POSITION =
      OFFSET_HARDWARE_ERROR_STATUS + XM540::LEN_HARDWARE_ERROR_STATUS;  // 12
  constexpr uint16_t OFFSET_GOAL_VELOCITY =
      OFFSET_GOAL_POSITION + XM540::LEN_GOAL_POSITION;                  // 16
  constexpr uint16_t OFFSET_GOAL_CURRENT =
      OFFSET_GOAL_VELOCITY + XM540::LEN_GOAL_VELOCITY;                  // 20
  constexpr uint16_t OFFSET_MOVING =
      OFFSET_GOAL_CURRENT + XM540::LEN_GOAL_CURRENT;                    // 22

  // 안정성 우선으로 자동 튜닝할 하한/상한
  const int min_read_period_ms = 1;
  const int max_read_period_ms = 20;
  // 이전 자동 튜닝 결과가 있으면 기본 주기로 사용
  int read_period_ms = loadReadPeriodFromConfig(
      min_read_period_ms, max_read_period_ms, DEFAULT_READ_PERIOD_MS);
  int current_read_period_ms = read_period_ms;
  int recommended_period_ms = current_read_period_ms;

  // 출력은 저주기로 제한해 콘솔 I/O가 통신을 방해하지 않도록 함
  const int print_interval_ms = 100;
  const int control_window_ms = 1000;
  const double degrade_issue_ratio = 0.02;
  const double improve_issue_ratio = 0.005;
  const double recommendation_issue_ratio = 0.01;
  const uint64_t min_recommendation_cycles = 200;

  std::atomic<bool> running{true};
  std::array<MotorSnapshot, MOTOR_IDS.size()> snapshots{};
  std::array<PeriodStats, 21> period_stats{};

  std::cout << "4개 모터 주기 읽기 시작 (target period: " << read_period_ms
            << " ms, 종료: Enter)\n"
            << "설정 파일: " << READ_PERIOD_CONFIG_PATH << '\n';

  std::thread reader_thread([&]() {
    auto last_print_time = std::chrono::steady_clock::now();
    auto window_start_time = std::chrono::steady_clock::now();
    int window_cycles = 0;
    int window_comm_failures = 0;
    int window_deadline_misses = 0;
    int consecutive_stable_windows = 0;

    while (running.load()) {
      const auto cycle_start = std::chrono::steady_clock::now();

      const int txrx_result = group_sync_read.txRxPacket();
      if (txrx_result != COMM_SUCCESS) {
        ++window_comm_failures;
      } else {
        for (size_t i = 0; i < MOTOR_IDS.size(); ++i) {
          const uint8_t id = MOTOR_IDS[i];
          if (!group_sync_read.isAvailable(id, XM540::ADDR_INDIRECT_DATA_1,
                                           XM540::LEN_SYNC_READ_INDIRECT)) {
            ++window_comm_failures;
            snapshots[i].valid = false;
            continue;
          }

          snapshots[i].torque_enabled = static_cast<uint8_t>(group_sync_read.getData(
              id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_TORQUE_ENABLE,
              XM540::LEN_TORQUE_ENABLE));

          snapshots[i].present_position = toSigned32(static_cast<uint32_t>(
              group_sync_read.getData(id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_PRESENT_POSITION,
                                      XM540::LEN_PRESENT_POSITION)));

          snapshots[i].present_velocity = toSigned32(static_cast<uint32_t>(
              group_sync_read.getData(id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_PRESENT_VELOCITY,
                                      XM540::LEN_PRESENT_VELOCITY)));

          snapshots[i].present_current = toSigned16(static_cast<uint16_t>(
              group_sync_read.getData(id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_PRESENT_CURRENT,
                                      XM540::LEN_PRESENT_CURRENT)));

          snapshots[i].hardware_error_status = static_cast<uint8_t>(group_sync_read.getData(
              id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_HARDWARE_ERROR_STATUS,
              XM540::LEN_HARDWARE_ERROR_STATUS));

          snapshots[i].goal_position = toSigned32(static_cast<uint32_t>(
              group_sync_read.getData(id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_GOAL_POSITION,
                                      XM540::LEN_GOAL_POSITION)));

          snapshots[i].goal_velocity = toSigned32(static_cast<uint32_t>(
              group_sync_read.getData(id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_GOAL_VELOCITY,
                                      XM540::LEN_GOAL_VELOCITY)));

          snapshots[i].goal_current = toSigned16(static_cast<uint16_t>(
              group_sync_read.getData(id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_GOAL_CURRENT,
                                      XM540::LEN_GOAL_CURRENT)));

          snapshots[i].moving = static_cast<uint8_t>(group_sync_read.getData(
              id, XM540::ADDR_INDIRECT_DATA_1 + OFFSET_MOVING,
              XM540::LEN_MOVING));
          snapshots[i].valid = true;
        }
      }

      ++window_cycles;
      std::this_thread::sleep_until(
          cycle_start + std::chrono::milliseconds(current_read_period_ms));

      const auto after_sleep = std::chrono::steady_clock::now();
      if (after_sleep - cycle_start >
          std::chrono::milliseconds(current_read_period_ms + 1)) {
        ++window_deadline_misses;
      }

      if (after_sleep - last_print_time >= std::chrono::milliseconds(print_interval_ms)) {
        std::cout << "[period " << current_read_period_ms << " ms]"
                  << " comm_fail: " << window_comm_failures
                  << " deadline_miss: " << window_deadline_misses << '\n';
        for (size_t i = 0; i < MOTOR_IDS.size(); ++i) {
          if (!snapshots[i].valid) {
            std::cout << "  ID " << static_cast<int>(MOTOR_IDS[i]) << " : invalid\n";
            continue;
          }
          std::cout << "  ID " << static_cast<int>(MOTOR_IDS[i])
                    << " | torque_enable: " << static_cast<int>(snapshots[i].torque_enabled)
                    << " | present_position: " << snapshots[i].present_position
                    << " | present_velocity: " << snapshots[i].present_velocity
                    << " | present_current: " << snapshots[i].present_current
                    << " | hardware_error: 0x" << std::hex
                    << static_cast<int>(snapshots[i].hardware_error_status) << std::dec
                    << " | goal_position: " << snapshots[i].goal_position
                    << " | goal_velocity: " << snapshots[i].goal_velocity
                    << " | goal_current: " << snapshots[i].goal_current
                    << " | moving: " << static_cast<int>(snapshots[i].moving)
                    << '\n';
        }
        last_print_time = after_sleep;
      }

      if (after_sleep - window_start_time >= std::chrono::milliseconds(control_window_ms)) {
        const int window_issues = window_comm_failures + window_deadline_misses;
        const double issue_ratio =
            (window_cycles > 0) ? static_cast<double>(window_issues) / window_cycles : 0.0;

        period_stats[current_read_period_ms].cycles += static_cast<uint64_t>(window_cycles);
        period_stats[current_read_period_ms].issues += static_cast<uint64_t>(window_issues);

        int best_period = current_read_period_ms;
        bool found_recommendation = false;
        for (int p = min_read_period_ms; p <= max_read_period_ms; ++p) {
          const auto& s = period_stats[p];
          if (s.cycles < min_recommendation_cycles) continue;
          const double ratio = static_cast<double>(s.issues) / s.cycles;
          if (ratio <= recommendation_issue_ratio) {
            best_period = p;
            found_recommendation = true;
            break;
          }
        }
        if (!found_recommendation) {
          // 아직 충분한 샘플이 없는 경우, 누적 오류율이 가장 낮은 주기를 임시 추천
          double best_ratio = std::numeric_limits<double>::max();
          for (int p = min_read_period_ms; p <= max_read_period_ms; ++p) {
            const auto& s = period_stats[p];
            if (s.cycles == 0) continue;
            const double ratio = static_cast<double>(s.issues) / s.cycles;
            if (ratio < best_ratio) {
              best_ratio = ratio;
              best_period = p;
            }
          }
        }
        recommended_period_ms = best_period;

        // 오류/지연이 보이면 즉시 완화, 충분히 안정되면 천천히 단축
        if (issue_ratio > degrade_issue_ratio && current_read_period_ms < max_read_period_ms) {
          ++current_read_period_ms;
          consecutive_stable_windows = 0;
        } else if (issue_ratio < improve_issue_ratio && current_read_period_ms > min_read_period_ms) {
          ++consecutive_stable_windows;
          if (consecutive_stable_windows >= 2) {
            --current_read_period_ms;
            consecutive_stable_windows = 0;
          }
        } else {
          consecutive_stable_windows = 0;
        }

        window_cycles = 0;
        window_comm_failures = 0;
        window_deadline_misses = 0;
        window_start_time = after_sleep;

        std::cout << "[auto-tune] 추천 주기: " << recommended_period_ms
                  << " ms (현재 적용: " << current_read_period_ms << " ms)\n";
      }
    }
  });

  std::string stop_input;
  std::getline(std::cin, stop_input);
  running.store(false);
  if (reader_thread.joinable()) {
    reader_thread.join();
  }

  std::cout << "자동 튜닝 종료. 추천 주기: " << recommended_period_ms << " ms\n";
  if (saveReadPeriodToConfig(recommended_period_ms)) {
    std::cout << "추천 주기 저장 완료: " << READ_PERIOD_CONFIG_PATH << '\n';
  }
  for (int p = min_read_period_ms; p <= max_read_period_ms; ++p) {
    const auto& s = period_stats[p];
    if (s.cycles == 0) continue;
    const double ratio = static_cast<double>(s.issues) / s.cycles;
    std::cout << "  - " << p << " ms: cycles=" << s.cycles
              << ", issue_ratio=" << ratio << '\n';
  }

  for (uint8_t id : MOTOR_IDS) {
    setTorque(port_handler, packet_handler, id, false);
  }
  group_sync_read.clearParam();
  port_handler->closePort();

  return 0;
}
