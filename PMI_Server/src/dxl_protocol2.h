#ifndef DXL_PROTOCOL2_H
#define DXL_PROTOCOL2_H

// DYNAMIXEL Protocol 2.0 (XM540-W270) via ROBOTIS DynamixelSDK.
// Telemetry: per-motor Read (PacketHandler::readTxRx). Commands: GroupSyncWrite.

#include "pmi_protocol.h"

#include <cstdint>
#include <mutex>

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

    dynamixel::PortHandler *port_ = nullptr;
    dynamixel::PacketHandler *packet_ = nullptr;

    mutable std::mutex mutex_;
};

#endif
