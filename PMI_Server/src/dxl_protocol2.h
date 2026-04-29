#ifndef DXL_PROTOCOL2_H
#define DXL_PROTOCOL2_H

// DYNAMIXEL Protocol 2.0 (XM540-W270) via ROBOTIS DynamixelSDK.
// Telemetry: per-motor Read (PacketHandler::readTxRx). Commands: GroupSyncWrite.

#include "pmi_protocol.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace dynamixel {
class PortHandler;
class PacketHandler;
} // namespace dynamixel

class DxlBus {
public:
    DxlBus() = default;
    ~DxlBus();

    DxlBus(const DxlBus &) = delete;
    DxlBus &operator=(const DxlBus &) = delete;

    bool open(const char *devicePath, int baudRate);
    void close();
    bool isOpen() const;

    /// Read control table for IDs 1–4 (SDK Read transactions).
    bool syncReadTelemetry(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]);

    /// Torque / operating mode via GroupSyncWrite.
    void handlePmiClientCommand(uint8_t cmd);

private:
    void closeUnlocked();
    bool readMotorTelemetryUnlocked(uint8_t id, pmi::ServoTelemetry &out);
    bool openAmt21PortUnlocked(const std::string &devicePath, int baudRate);
    bool readAmt21AngleDegUnlocked(uint8_t nodeAddress, double &angleDegOut);

    dynamixel::PortHandler *port_ = nullptr;
    dynamixel::PacketHandler *packet_ = nullptr;
    int amt21_fd_ = -1;
    std::array<uint32_t, 3> amt21_fail_count_{{0, 0, 0}};
    std::array<bool, 3> amt21_in_fail_state_{{false, false, false}};
    std::array<std::chrono::steady_clock::time_point, 3> amt21_last_fail_log_{};

    mutable std::mutex mutex_;
};

#endif
