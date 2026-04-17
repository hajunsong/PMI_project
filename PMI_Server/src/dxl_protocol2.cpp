// DYNAMIXEL SDK: PortHandler, PacketHandler, readTxRx / read1ByteTxRx for telemetry; GroupSyncWrite for commands.

#include "dxl_protocol2.h"

#include <array>
#include <cstring>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace {

constexpr uint8_t kMotorIds[pmi::kTelemetryAxisCount] = {1, 2, 3, 4};

constexpr uint16_t kAddrOperatingMode = 11;
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrHardwareError = 70;
constexpr uint16_t kAddrGoalCurrent = 102;
constexpr uint16_t kAddrPresentCurrent = 126;

// XM540-W270 (Protocol 2.0) raw → SI-style for client: deg, deg/s, A. See e-Manual control table.
constexpr double kPulseToDeg = 360.0 / 4096.0; // Present / Goal Position resolution
constexpr double kVelRawToRpm = 0.229;       // Present / Goal Velocity
constexpr double kRpmToDegPerSec = 6.0;      // rpm → deg/s (×360/60)
constexpr double kCurRawToMa = 2.69;         // Present / Goal Current (when used as current)
constexpr double kMaToA = 0.001;

int32_t readI32Le(const uint8_t *p)
{
    uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16)
        | (static_cast<uint32_t>(p[3]) << 24);
    int32_t v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

int16_t readI16Le(const uint8_t *p)
{
    uint16_t u = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    int16_t v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

} // namespace

DxlBus::~DxlBus()
{
    close();
}

bool DxlBus::isOpen() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return port_ != nullptr;
}

void DxlBus::closeUnlocked()
{
    if (port_) {
        port_->closePort();
        delete port_;
        port_ = nullptr;
    }
    if (packet_) {
        delete packet_;
        packet_ = nullptr;
    }
}

bool DxlBus::open(const char *devicePath, int baudRate)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeUnlocked();
    }

    port_ = dynamixel::PortHandler::getPortHandler(devicePath);
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_ || !packet_) {
        closeUnlocked();
        return false;
    }
    if (!port_->openPort()) {
        closeUnlocked();
        return false;
    }
    if (!port_->setBaudRate(baudRate)) {
        closeUnlocked();
        return false;
    }
    return true;
}

void DxlBus::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    closeUnlocked();
}

bool DxlBus::readMotorTelemetryUnlocked(uint8_t id, pmi::ServoTelemetry &out)
{
    uint8_t dxl_error = 0;
    uint8_t b1 = 0;
    if (packet_->read1ByteTxRx(port_, id, kAddrOperatingMode, &b1, &dxl_error) != COMM_SUCCESS)
        return false;
    const uint8_t op = b1;

    uint8_t tq = 0;
    if (packet_->read1ByteTxRx(port_, id, kAddrTorqueEnable, &tq, &dxl_error) != COMM_SUCCESS)
        return false;

    uint8_t hw = 0;
    if (packet_->read1ByteTxRx(port_, id, kAddrHardwareError, &hw, &dxl_error) != COMM_SUCCESS)
        return false;

    uint8_t goalBlock[18]{};
    if (packet_->readTxRx(port_, id, kAddrGoalCurrent, 18, goalBlock, &dxl_error) != COMM_SUCCESS)
        return false;
    const int16_t goalCurRaw = readI16Le(goalBlock);
    const int32_t goalVelRaw = readI32Le(goalBlock + 2);
    const int32_t goalPosRaw = readI32Le(goalBlock + 14);

    uint8_t presBlock[10]{};
    if (packet_->readTxRx(port_, id, kAddrPresentCurrent, 10, presBlock, &dxl_error) != COMM_SUCCESS)
        return false;
    const int16_t presCurRaw = readI16Le(presBlock);
    const int32_t presVelRaw = readI32Le(presBlock + 2);
    const int32_t presPosRaw = readI32Le(presBlock + 6);

    out.id_op_mode = pmi::packTelemetryIdOp(id, op);
    out.servo_state = tq ? 1 : 0;
    out.present_position = static_cast<double>(presPosRaw) * kPulseToDeg;
    out.present_velocity = static_cast<double>(presVelRaw) * kVelRawToRpm * kRpmToDegPerSec;
    out.present_current = static_cast<double>(presCurRaw) * kCurRawToMa * kMaToA;
    out.goal_position = static_cast<double>(goalPosRaw) * kPulseToDeg;
    out.goal_velocity = static_cast<double>(goalVelRaw) * kVelRawToRpm * kRpmToDegPerSec;
    out.goal_current = static_cast<double>(goalCurRaw) * kCurRawToMa * kMaToA;
    out.error_state = hw;
    return true;
}

bool DxlBus::syncReadTelemetry(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_)
        return false;

    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        if (!readMotorTelemetryUnlocked(kMotorIds[i], axes[i]))
            return false;
    }
    return true;
}

void DxlBus::handlePmiClientCommand(uint8_t cmd)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_)
        return;

    const auto torqueAll = [&](uint8_t v) {
        std::array<uint8_t, pmi::kTelemetryAxisCount> bytes{};
        bytes.fill(v);
        dynamixel::GroupSyncWrite gsw(port_, packet_, kAddrTorqueEnable, 1);
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            (void)gsw.addParam(kMotorIds[i], &bytes[i]);
        }
        (void)gsw.txPacket();
    };
    const auto modeAll = [&](uint8_t mode) {
        std::array<uint8_t, pmi::kTelemetryAxisCount> bytes{};
        bytes.fill(mode);
        dynamixel::GroupSyncWrite gsw(port_, packet_, kAddrOperatingMode, 1);
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            (void)gsw.addParam(kMotorIds[i], &bytes[i]);
        }
        (void)gsw.txPacket();
    };

    switch (cmd) {
    case pmi::kCmdPing:
        break;
    case pmi::kCmdServoOn:
        torqueAll(1);
        break;
    case pmi::kCmdStop:
        torqueAll(0);
        break;
    case pmi::kCmdModeCurrent:
        torqueAll(0);
        modeAll(0);
        break;
    case pmi::kCmdModeVelocity:
        torqueAll(0);
        modeAll(1);
        break;
    case pmi::kCmdModeExtendedPos:
        torqueAll(0);
        modeAll(4);
        break;
    default:
        break;
    }
}
