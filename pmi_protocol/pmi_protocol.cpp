#include "pmi_protocol.h"

#include <cstring>

namespace pmi {
namespace {

void writeF64LE(uint8_t *dst, double v)
{
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    std::uint64_t u = 0;
    std::memcpy(&u, &v, 8);
    for (int b = 0; b < 8; ++b)
        dst[b] = static_cast<uint8_t>((u >> (8 * b)) & 0xFFu);
}

double readF64LE(const uint8_t *src)
{
    std::uint64_t u = 0;
    for (int b = 0; b < 8; ++b)
        u |= static_cast<std::uint64_t>(src[b]) << (8 * b);
    double v;
    std::memcpy(&v, &u, 8);
    return v;
}

bool appendTelemetryBlock(uint8_t *dst, size_t dstCap, size_t &off, const ServoTelemetry &t)
{
    if (off + kTelemetryBlockBytes > dstCap)
        return false;
    uint8_t *p = dst + off;
    p[0] = t.id_op_mode;
    p[1] = t.servo_state;
    writeF64LE(p + 2, t.present_position);
    writeF64LE(p + 10, t.present_velocity);
    writeF64LE(p + 18, t.present_current);
    writeF64LE(p + 26, t.goal_position);
    writeF64LE(p + 34, t.goal_velocity);
    writeF64LE(p + 42, t.goal_current);
    p[50] = t.error_state;
    off += kTelemetryBlockBytes;
    return true;
}

} // namespace

uint8_t checksumClientPayload(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    uint16_t s = static_cast<uint16_t>(cmd) + static_cast<uint16_t>(len);
    for (size_t i = 0; i < static_cast<size_t>(len); ++i)
        s = static_cast<uint16_t>(s + data[i]);
    return static_cast<uint8_t>(s & 0xFF);
}

uint8_t checksumServerPayload(uint8_t lenByte, const uint8_t *payload, size_t payloadLen)
{
    uint16_t s = static_cast<uint16_t>(lenByte);
    for (size_t i = 0; i < payloadLen; ++i)
        s = static_cast<uint16_t>(s + payload[i]);
    return static_cast<uint8_t>(s & 0xFF);
}

std::vector<uint8_t> buildClientFrame(uint8_t cmd, const std::vector<uint8_t> &data)
{
    const size_t n = data.size() > 255 ? 255 : data.size();
    const uint8_t len = static_cast<uint8_t>(n);
    std::vector<uint8_t> out;
    out.reserve(static_cast<size_t>(6) + n);
    out.push_back(kSof1);
    out.push_back(kSof2);
    out.push_back(cmd);
    out.push_back(len);
    out.insert(out.end(), data.begin(), data.begin() + static_cast<std::ptrdiff_t>(n));
    out.push_back(checksumClientPayload(cmd, len, n ? data.data() : nullptr));
    out.push_back(kFrameEof);
    return out;
}

std::vector<uint8_t> buildServerFrame(const ServoTelemetry axes[kTelemetryAxisCount])
{
    std::vector<uint8_t> frame;
    frame.resize(kServerFrameBytes);
    size_t o = 0;
    frame[o++] = kSof1;
    frame[o++] = kSof2;
    const uint8_t lenByte = static_cast<uint8_t>(kServerPayloadBytes);
    frame[o++] = lenByte;
    for (size_t i = 0; i < kTelemetryAxisCount; ++i) {
        if (!appendTelemetryBlock(frame.data(), frame.size(), o, axes[i]))
            return {};
    }
    const uint8_t cs = checksumServerPayload(lenByte, frame.data() + 3, kServerPayloadBytes);
    frame[o++] = cs;
    frame[o++] = kFrameEof;
    return frame;
}

bool parseServerFrame(const uint8_t *frame, size_t frameLen, ServoTelemetry axesOut[kTelemetryAxisCount])
{
    if (frameLen != kServerFrameBytes)
        return false;
    if (frame[0] != kSof1 || frame[1] != kSof2)
        return false;
    const uint8_t lenByte = frame[2];
    if (lenByte != static_cast<uint8_t>(kServerPayloadBytes))
        return false;
    const uint8_t cs = frame[3 + kServerPayloadBytes];
    const uint8_t eof = frame[3 + kServerPayloadBytes + 1];
    if (eof != kFrameEof)
        return false;
    const uint8_t expectCs = checksumServerPayload(lenByte, frame + 3, kServerPayloadBytes);
    if (cs != expectCs)
        return false;

    const uint8_t *p = frame + 3;
    for (size_t a = 0; a < kTelemetryAxisCount; ++a) {
        const uint8_t *b = p + a * kTelemetryBlockBytes;
        ServoTelemetry &t = axesOut[a];
        t.id_op_mode = b[0];
        t.servo_state = b[1];
        t.present_position = readF64LE(b + 2);
        t.present_velocity = readF64LE(b + 10);
        t.present_current = readF64LE(b + 18);
        t.goal_position = readF64LE(b + 26);
        t.goal_velocity = readF64LE(b + 34);
        t.goal_current = readF64LE(b + 42);
        t.error_state = b[50];
    }
    return true;
}

void feedServerRxStream(std::vector<uint8_t> &rx, const ServerFrameHandler &onFrame)
{
    size_t i = 0;
    while (i + kServerFrameBytes <= rx.size()) {
        if (rx[i] != kSof1 || rx[i + 1] != kSof2) {
            ++i;
            continue;
        }
        ServoTelemetry axes[kTelemetryAxisCount]{};
        if (parseServerFrame(rx.data() + i, kServerFrameBytes, axes)) {
            if (onFrame)
                onFrame(axes);
            rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(i + kServerFrameBytes));
            i = 0;
            continue;
        }
        ++i;
    }
    if (i > 0)
        rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(i));
}

void pruneServerRxToLatestCompleteFrame(std::vector<uint8_t> &rx)
{
    size_t lastStart = static_cast<size_t>(-1);
    const size_t n = rx.size();
    if (n < kServerFrameBytes) {
        if (n > kServerRxPruneCapBytes)
            rx.erase(rx.begin(), rx.end() - static_cast<std::ptrdiff_t>(kServerRxPruneCapBytes));
        return;
    }
    for (size_t i = 0; i + kServerFrameBytes <= n; ++i) {
        if (rx[i] != kSof1 || rx[i + 1] != kSof2)
            continue;
        ServoTelemetry tmp[kTelemetryAxisCount]{};
        if (parseServerFrame(rx.data() + i, kServerFrameBytes, tmp))
            lastStart = i;
    }
    if (lastStart != static_cast<size_t>(-1)) {
        if (lastStart > 0)
            rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(lastStart));
        return;
    }
    if (n > kServerRxPruneCapBytes)
        rx.erase(rx.begin(), rx.end() - static_cast<std::ptrdiff_t>(kServerRxPruneCapBytes));
}

void feedClientRxStream(std::vector<uint8_t> &rx, const ClientFrameHandler &onFrame)
{
    size_t i = 0;
    while (i + 6 <= rx.size()) {
        if (rx[i] != kSof1 || rx[i + 1] != kSof2) {
            ++i;
            continue;
        }
        const uint8_t cmd = rx[i + 2];
        const uint8_t len = rx[i + 3];
        const size_t frameTotal = static_cast<size_t>(6) + len;
        if (i + frameTotal > rx.size())
            break;
        const uint8_t *dataPtr = (len > 0) ? (rx.data() + i + 4) : nullptr;
        const uint8_t cs = rx[i + 4 + len];
        const uint8_t eof = rx[i + 5 + len];
        const uint8_t expectCs = checksumClientPayload(cmd, len, dataPtr);
        if (eof == kFrameEof && cs == expectCs) {
            std::vector<uint8_t> payload;
            if (len > 0)
                payload.assign(rx.begin() + static_cast<std::ptrdiff_t>(i + 4),
                    rx.begin() + static_cast<std::ptrdiff_t>(i + 4 + len));
            if (onFrame)
                onFrame(cmd, payload);
            rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(i + frameTotal));
            i = 0;
            continue;
        }
        ++i;
    }
    if (i > 0)
        rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(i));
}

} // namespace pmi
