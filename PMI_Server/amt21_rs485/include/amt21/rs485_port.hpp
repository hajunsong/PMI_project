#ifndef AMT21_RS485_PORT_HPP
#define AMT21_RS485_PORT_HPP

#include <cstddef>
#include <cstdint>

namespace amt21 {

/// 리눅스 시리얼 8N1 (RS485 변환기 뒤 AMT21과 연결).
class Rs485Port {
public:
    Rs485Port() = default;
    ~Rs485Port();

    Rs485Port(const Rs485Port &) = delete;
    Rs485Port &operator=(const Rs485Port &) = delete;

    bool open(const char *devicePath, int baudRate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    bool writeBytes(const uint8_t *data, size_t len);
    bool readExact(uint8_t *out, size_t len, int timeoutMs);

    void flushInput();
    /// 송·수신 버퍼 모두 비움 (이전 버스 잔여 바이트 제거).
    void flushIo();

private:
    int fd_ = -1;
};

} // namespace amt21

#endif
