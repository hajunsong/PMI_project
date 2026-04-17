#include "amt21/amt21_driver.hpp"

#include <unistd.h>

namespace amt21 {

namespace {

/// RS485 반이중: 송신 후 트랜시버 반전·슬레이브 응답까지 (데이터시트 µs~ms 단위). USB 변환기는 ms에 가깝게.
void postTxDelay()
{
    ::usleep(2000);
}

constexpr int kReadTimeoutMs = 150;

uint16_t le16(const uint8_t *p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

} // namespace

Amt21Driver::Amt21Driver(Rs485Port &port)
    : port_(&port)
{
}

std::optional<uint16_t> Amt21Driver::readPositionRawFrame()
{
    const uint8_t cmd = cmdReadPosition(nodeBase_);
    port_->flushIo();
    if (!port_->writeBytes(&cmd, 1))
        return std::nullopt;
    postTxDelay();
    uint8_t rx[2]{};
    if (!port_->readExact(rx, 2, kReadTimeoutMs))
        return std::nullopt;
    return le16(rx);
}

std::optional<uint16_t> Amt21Driver::readPosition14()
{
    for (int attempt = 0; attempt < kAmt21ReadRetries; ++attempt) {
        const auto raw = readPositionRawFrame();
        if (!raw) {
            if (attempt + 1 < kAmt21ReadRetries)
                ::usleep(kAmt21ReadRetryDelayUs);
            continue;
        }
        if (const auto dec = decodePosition14(*raw))
            return dec;
        if (attempt + 1 < kAmt21ReadRetries)
            ::usleep(kAmt21ReadRetryDelayUs);
    }
    return std::nullopt;
}

std::optional<uint16_t> Amt21Driver::readPosition12()
{
    for (int attempt = 0; attempt < kAmt21ReadRetries; ++attempt) {
        const auto raw = readPositionRawFrame();
        if (!raw) {
            if (attempt + 1 < kAmt21ReadRetries)
                ::usleep(kAmt21ReadRetryDelayUs);
            continue;
        }
        if (const auto dec = decodePosition12(*raw))
            return dec;
        if (attempt + 1 < kAmt21ReadRetries)
            ::usleep(kAmt21ReadRetryDelayUs);
    }
    return std::nullopt;
}

std::optional<int16_t> Amt21Driver::readTurns14()
{
    for (int attempt = 0; attempt < kAmt21ReadRetries; ++attempt) {
        const uint8_t cmd = cmdReadTurns(nodeBase_);
        port_->flushIo();
        if (!port_->writeBytes(&cmd, 1)) {
            if (attempt + 1 < kAmt21ReadRetries)
                ::usleep(kAmt21ReadRetryDelayUs);
            continue;
        }
        postTxDelay();
        uint8_t rx[2]{};
        if (!port_->readExact(rx, 2, kReadTimeoutMs)) {
            if (attempt + 1 < kAmt21ReadRetries)
                ::usleep(kAmt21ReadRetryDelayUs);
            continue;
        }
        const uint16_t raw = le16(rx);
        if (!verifyResponseChecksum(raw)) {
            if (attempt + 1 < kAmt21ReadRetries)
                ::usleep(kAmt21ReadRetryDelayUs);
            continue;
        }
        const uint16_t u14 = static_cast<uint16_t>(raw & 0x3FFFu);
        int16_t signed14;
        if (u14 & 0x2000u) {
            const uint16_t neg = static_cast<uint16_t>(u14 | 0xC000u);
            signed14 = static_cast<int16_t>(neg);
        } else {
            signed14 = static_cast<int16_t>(u14);
        }
        return signed14;
    }
    return std::nullopt;
}

bool Amt21Driver::sendExtended(uint8_t extByte)
{
    const uint8_t pkt[2] = {cmdExtendedPrefix(nodeBase_), extByte};
    port_->flushIo();
    return port_->writeBytes(pkt, 2);
}

} // namespace amt21
