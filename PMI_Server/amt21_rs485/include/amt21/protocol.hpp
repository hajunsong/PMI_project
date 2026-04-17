#ifndef AMT21_PROTOCOL_HPP
#define AMT21_PROTOCOL_HPP

#include "amt21/model.hpp"

#include <cstdint>
#include <optional>

// AMT21 계열 RS485 공통 (타깃: model::kPartNumber, 예 AMT212H-V)

namespace amt21 {
/// 기본 노드 주소(위치 읽기와 동일). 하위 2비트는 00이어야 함 (PDF: 4의 배수).
constexpr uint8_t kDefaultNodeAddress = 0x54;

/// 위치/턴 읽기: 타임아웃·체크섬 실패 시 재시도 (버스 잡음·RS485 반전 지연).
inline constexpr int kAmt21ReadRetries = 6;
inline constexpr int kAmt21ReadRetryDelayUs = 4000;

inline uint8_t cmdReadPosition(uint8_t nodeBase)
{
    return static_cast<uint8_t>(nodeBase & 0xFCu);
}

inline uint8_t cmdReadTurns(uint8_t nodeBase)
{
    return static_cast<uint8_t>((nodeBase & 0xFCu) | 0x01u);
}

inline uint8_t cmdExtendedPrefix(uint8_t nodeBase)
{
    return static_cast<uint8_t>((nodeBase & 0xFCu) | 0x02u);
}

constexpr uint8_t kExtSetZero = 0x5E;
constexpr uint8_t kExtReset = 0x75;

/// 16비트 응답 상위 2비트: K1(bit15), K0(bit14). 하위 14비트가 데이터(위치 등).
bool verifyResponseChecksum(uint16_t word);

/// 14비트 싱글턴: 하위 14비트가 위치(0..16383). 체크섬 실패 시 nullopt.
std::optional<uint16_t> decodePosition14(uint16_t rawLittleEndian);

/// 12비트: PDF — 응답 하위 2비트 버림 후 12비트 사용.
std::optional<uint16_t> decodePosition12(uint16_t rawLittleEndian);

} // namespace amt21

#endif
