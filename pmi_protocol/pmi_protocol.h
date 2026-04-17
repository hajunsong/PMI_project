#ifndef PMI_PROTOCOL_H
#define PMI_PROTOCOL_H

// 공통 모듈: PMI_Client / PMI_Server 가 동일 파일을 참조합니다.
// PMI = Position Marking Indicator

// PMI custom byte framing (client <-> server). SOF/EOF values are project-defined
// until hardware documentation assigns fixed magic bytes.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace pmi {

constexpr uint8_t kCmdPing = 0x01;
constexpr uint8_t kCmdServoOn = 0x10;
constexpr uint8_t kCmdStop = 0x11;
constexpr uint8_t kCmdModeCurrent = 0x20;
constexpr uint8_t kCmdModeVelocity = 0x21;
constexpr uint8_t kCmdModeExtendedPos = 0x22;

constexpr uint8_t kSof1 = 0xAA;
constexpr uint8_t kSof2 = 0x55;
constexpr uint8_t kFrameEof = 0xFE;

constexpr size_t kTelemetryBlockBytes = 51;
constexpr size_t kTelemetryAxisCount = 4;
constexpr size_t kServerPayloadBytes = kTelemetryBlockBytes * kTelemetryAxisCount;

constexpr size_t kServerFrameBytes = 2 + 1 + kServerPayloadBytes + 1 + 1;

// When the TCP receive queue grows faster than the UI consumes it, keep only the most
// recent complete server frame (plus any trailing partial bytes) to avoid stale telemetry.
constexpr size_t kServerRxPruneCapBytes = 16384;

struct ServoTelemetry {
    /// Packed byte: high nibble = ID, low nibble = operating mode (Op). See `packTelemetryIdOp`.
    uint8_t id_op_mode = 0;
    uint8_t servo_state = 0;
    /// SI-style fields (PMI_Server converts from DYNAMIXEL raw): degrees, deg/s, amperes.
    double present_position = 0.0;
    double present_velocity = 0.0;
    double present_current = 0.0;
    double goal_position = 0.0;
    double goal_velocity = 0.0;
    double goal_current = 0.0;
    uint8_t error_state = 0;
};

constexpr uint8_t packTelemetryIdOp(uint8_t id, uint8_t op) noexcept
{
    return static_cast<uint8_t>(((id & 0x0Fu) << 4) | (op & 0x0Fu));
}

constexpr uint8_t telemetryIdFromIdOp(uint8_t id_op_mode) noexcept
{
    return static_cast<uint8_t>((id_op_mode >> 4) & 0x0Fu);
}

constexpr uint8_t telemetryOpModeFromIdOp(uint8_t id_op_mode) noexcept
{
    return static_cast<uint8_t>(id_op_mode & 0x0Fu);
}

uint8_t checksumClientPayload(uint8_t cmd, uint8_t len, const uint8_t *data);
uint8_t checksumServerPayload(uint8_t lenByte, const uint8_t *payload, size_t payloadLen);

std::vector<uint8_t> buildClientFrame(uint8_t cmd, const std::vector<uint8_t> &data);
std::vector<uint8_t> buildServerFrame(const ServoTelemetry axes[kTelemetryAxisCount]);

bool parseServerFrame(const uint8_t *frame, size_t frameLen, ServoTelemetry axesOut[kTelemetryAxisCount]);

using ClientFrameHandler = std::function<void(uint8_t cmd, const std::vector<uint8_t> &payload)>;
using ServerFrameHandler = std::function<void(const ServoTelemetry axes[kTelemetryAxisCount])>;

void feedClientRxStream(std::vector<uint8_t> &rx, const ClientFrameHandler &onFrame);
void feedServerRxStream(std::vector<uint8_t> &rx, const ServerFrameHandler &onFrame);

void pruneServerRxToLatestCompleteFrame(std::vector<uint8_t> &rx);

} // namespace pmi

#endif
