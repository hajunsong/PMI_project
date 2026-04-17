#ifndef AMT21_DRIVER_HPP
#define AMT21_DRIVER_HPP

#include "amt21/model.hpp"
#include "amt21/protocol.hpp"
#include "amt21/rs485_port.hpp"

#include <cstdint>
#include <optional>

namespace amt21 {

class Amt21Driver {
public:
    explicit Amt21Driver(Rs485Port &port);

    void setNodeBase(uint8_t node) { nodeBase_ = static_cast<uint8_t>(node & 0xFCu); }

    /// 위치 읽기: 버스에 1바이트 전송 후 2바이트 수신.
    std::optional<uint16_t> readPosition14();
    std::optional<uint16_t> readPosition12();

    /// 2바이트 LE 응답만 수신(체크섬 미검증). 디버그·--verbose용.
    std::optional<uint16_t> readPositionRawFrame();

    /// 멀티턴만. 실패 시 nullopt.
    std::optional<int16_t> readTurns14();

    /// 확장 명령(응답 없음). 전송만 수행.
    bool sendExtended(uint8_t extByte);

private:
    Rs485Port *port_ = nullptr;
    uint8_t nodeBase_ = kDefaultNodeAddress;
};

} // namespace amt21

#endif
