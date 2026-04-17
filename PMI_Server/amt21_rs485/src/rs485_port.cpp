#include "amt21/rs485_port.hpp"

#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace amt21 {

namespace {

speed_t baudToFlag(int baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 4000000:
        return B4000000;
    default:
        return B0;
    }
}

} // namespace

Rs485Port::~Rs485Port()
{
    close();
}

void Rs485Port::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Rs485Port::open(const char *devicePath, int baudRate)
{
    close();
    const int fd = ::open(devicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return false;

    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        ::close(fd);
        return false;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS));
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    const speed_t spd = baudToFlag(baudRate);
    if (spd == B0 || ::cfsetspeed(&tio, spd) != 0) {
        ::close(fd);
        return false;
    }
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        ::close(fd);
        return false;
    }
    fd_ = fd;
    return true;
}

void Rs485Port::flushInput()
{
    if (fd_ >= 0)
        (void)::tcflush(fd_, TCIFLUSH);
}

void Rs485Port::flushIo()
{
    if (fd_ >= 0)
        (void)::tcflush(fd_, TCIOFLUSH);
}

bool Rs485Port::writeBytes(const uint8_t *data, size_t len)
{
    if (fd_ < 0)
        return false;
    size_t off = 0;
    while (off < len) {
        const ssize_t w = ::write(fd_, data + off, len - off);
        if (w < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        off += static_cast<size_t>(w);
    }
    (void)::tcdrain(fd_);
    return true;
}

bool Rs485Port::readExact(uint8_t *out, size_t len, int timeoutMs)
{
    if (fd_ < 0)
        return false;
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeoutMs < 0 ? 0 : timeoutMs);
    size_t got = 0;
    while (got < len) {
        const int waitMs = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count());
        if (waitMs <= 0)
            return false;
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, waitMs);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (pr == 0)
            return false;
        const ssize_t n = ::read(fd_, out + got, len - got);
        if (n > 0) {
            got += static_cast<size_t>(n);
        } else if (n == 0) {
            return false;
        } else if (errno != EAGAIN && errno != EINTR) {
            return false;
        }
    }
    return true;
}

} // namespace amt21
