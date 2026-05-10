#ifndef DXL_PROTOCOL2_H
#define DXL_PROTOCOL2_H

// DYNAMIXEL Protocol 2.0 (XM540-W270) via ROBOTIS DynamixelSDK.
// Telemetry: persistent GroupSyncRead of a 23-byte Indirect Data block (1 broadcast transaction for
// all 4 motors per loop). Requires a U2D2 / native USB-RS485 adapter on /dev/ttyUSB*; USB-CDC bridges
// (OpenCM9.04 / /dev/ttyACM*) bunch back-to-back status packets and cause -3002 framing errors.
// Commands: persistent GroupSyncWrite to indirect-data goal block (one bus transaction per goal type;
// writes have no per-ID response so broadcast is safe here).
// Operating Mode (addr 11) is in EEPROM and cannot be Indirect-mapped — it is cached locally
// and updated on every mode-set path so per-loop reads don't pay an extra transaction for it.

#include "pmi_protocol.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>

namespace dynamixel {
class PortHandler;
class PacketHandler;
class GroupSyncRead;
class GroupSyncWrite;
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

    /// Broadcast GroupSyncRead of the 23-byte indirect-data block (1 transaction for all 4 motors).
    /// Requires a U2D2 or equivalent USB-RS485 adapter on /dev/ttyUSB*.
    bool syncReadTelemetry(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]);

    /// Torque / operating mode via GroupSyncWrite.
    void handlePmiClientCommand(uint8_t cmd);
    bool writeGoalPositionDeg(const std::array<double, pmi::kTelemetryAxisCount> &motorDeg);
    bool writeGoalVelocityDegPerSec(const std::array<double, pmi::kTelemetryAxisCount> &motorDegPerSec);
    /// Goal Current in Current Control Mode (XM540-W270 addr 102); `motorAmp` = motor-shaft current [A], ±.
    bool writeGoalCurrentAmp(const std::array<double, pmi::kTelemetryAxisCount> &motorAmp);
    bool setAxisTorque(size_t axisIndex, bool enable);
    bool setAxisOperatingMode(size_t axisIndex, uint8_t mode);
    bool setAxisGoalCurrentRaw(size_t axisIndex, int16_t goalCurrentRaw);

private:
    void closeUnlocked();
    bool openAmt21PortUnlocked(const std::string &devicePath, int baudRate);
    bool readAmt21AngleDegUnlocked(uint8_t nodeAddress, double &angleDegOut);
    /// Lock-already-held variant used by syncReadTelemetry / captureZeroOffsetUnlocked.
    bool syncReadRawTelemetryUnlocked(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]);
    bool captureZeroOffsetUnlocked();
    void applyZeroOffsetUnlocked(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]) const;
    void loadZeroOffsetFromFileUnlocked();
    bool saveZeroOffsetToFileUnlocked() const;
    std::string zeroOffsetFilePath() const;

    /// Write Indirect Address registers (RAM-area sources only) on every motor.
    /// Must be called when torque is OFF; reboot resets these mappings.
    bool setupIndirectMappingsUnlocked();
    /// Recreate the persistent GroupSyncWrite instances after the SDK port has been (re-)opened.
    void rebuildSyncGroupsUnlocked();
    /// Free all persistent group instances.
    void destroySyncGroupsUnlocked();
    /// One-shot read of operating mode for every axis into op_mode_cache_ (used post-open / post-reboot).
    void refreshOpModeCacheUnlocked();

    dynamixel::PortHandler *port_ = nullptr;
    dynamixel::PacketHandler *packet_ = nullptr;
    dynamixel::GroupSyncRead  *sync_read_telemetry_      = nullptr;
    dynamixel::GroupSyncWrite *sync_write_goal_current_  = nullptr;
    dynamixel::GroupSyncWrite *sync_write_goal_velocity_ = nullptr;
    dynamixel::GroupSyncWrite *sync_write_goal_position_ = nullptr;
    int amt21_fd_ = -1;
    std::array<uint32_t, 3> amt21_fail_count_{{0, 0, 0}};
    std::array<bool, 3> amt21_in_fail_state_{{false, false, false}};
    std::array<std::chrono::steady_clock::time_point, 3> amt21_last_fail_log_{};
    std::array<double, pmi::kTelemetryAxisCount> motor_offset_deg_{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, pmi::kTelemetryAxisCount> goal_offset_deg_{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, pmi::kTelemetryAxisCount> encoder_offset_deg_{
        {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()}};
    /// Operating Mode (addr 11) lives in EEPROM and cannot be mapped to an Indirect Address;
    /// kept in sync with every kCmdMode* / setAxisOperatingMode call instead of being re-read each loop.
    std::array<uint8_t, pmi::kTelemetryAxisCount> op_mode_cache_{{3, 3, 3, 3}};
    bool zero_offset_valid_ = false;
    bool indirect_setup_ok_ = false;
    std::string zero_offset_path_;

    mutable std::mutex mutex_;
};

#endif
