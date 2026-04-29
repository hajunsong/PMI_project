#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>

static bool amt21_checksum_ok(uint16_t raw)
{
    uint8_t k1 = (raw >> 15) & 0x01;
    uint8_t k0 = (raw >> 14) & 0x01;

    uint8_t calc_k1 = !(
        ((raw >> 13) & 0x01) ^
        ((raw >> 11) & 0x01) ^
        ((raw >> 9)  & 0x01) ^
        ((raw >> 7)  & 0x01) ^
        ((raw >> 5)  & 0x01) ^
        ((raw >> 3)  & 0x01) ^
        ((raw >> 1)  & 0x01)
    );

    uint8_t calc_k0 = !(
        ((raw >> 12) & 0x01) ^
        ((raw >> 10) & 0x01) ^
        ((raw >> 8)  & 0x01) ^
        ((raw >> 6)  & 0x01) ^
        ((raw >> 4)  & 0x01) ^
        ((raw >> 2)  & 0x01) ^
        ((raw >> 0)  & 0x01)
    );

    return (k1 == calc_k1) && (k0 == calc_k0);
}

static uint16_t amt21_position(uint16_t raw, int resolution_bits)
{
    uint16_t data14 = raw & 0x3FFF;

    if (resolution_bits == 14)
    {
        return data14;
    }
    else if (resolution_bits == 12)
    {
        return data14 >> 2;
    }

    return 0;
}

static double amt21_angle_deg(uint16_t position, int resolution_bits)
{
    const double max_count = (resolution_bits == 14) ? 16384.0 : 4096.0;
    return static_cast<double>(position) / max_count * 360.0;
}

static speed_t baudrate_to_termios(int baudrate)
{
    switch (baudrate)
    {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
        default: return B115200;
    }
}

static int open_serial_port(const std::string& port_name, int baudrate)
{
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        std::perror("open");
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        std::perror("tcgetattr");
        close(fd);
        return -1;
    }

    speed_t speed = baudrate_to_termios(baudrate);

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 설정
    tty.c_cflag &= ~PARENB;          // no parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              // 8 data bits

    tty.c_cflag &= ~CRTSCTS;         // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;   // enable receiver, ignore modem control

    // raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;

    // read timeout
    // VMIN = 0, VTIME = 1 means read returns when data arrives or after 0.1 s.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::perror("tcsetattr");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    return fd;
}

static int read_exact_with_timeout(int fd, uint8_t* buffer, int length, int timeout_ms)
{
    int total = 0;
    auto start = std::chrono::steady_clock::now();

    while (total < length)
    {
        int n = read(fd, buffer + total, length - total);

        if (n > 0)
        {
            total += n;
        }

        auto now = std::chrono::steady_clock::now();
        int elapsed_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
        );

        if (elapsed_ms >= timeout_ms)
        {
            break;
        }

        if (n == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    return total;
}

static bool read_amt21_once(int fd, uint8_t node_address, int resolution_bits)
{
    // 이전 데이터 제거
    tcflush(fd, TCIOFLUSH);

    // Read position command: 1 byte only
    int written = write(fd, &node_address, 1);

    if (written != 1)
    {
        std::perror("write");
        return false;
    }

    // 출력 완료 대기
    tcdrain(fd);

    uint8_t rx[2] = {0, 0};

    // AMT21 response: low byte first, high byte second
    int received = read_exact_with_timeout(fd, rx, 2, 50);

    if (received != 2)
    {
        std::cout << "RX length error. Expected 2 bytes, got "
                  << received << " byte(s): ";

        for (int i = 0; i < received; ++i)
        {
            std::cout << std::hex << std::uppercase
                      << std::setw(2) << std::setfill('0')
                      << static_cast<int>(rx[i]) << " ";
        }

        std::cout << std::dec << "\n";
        return false;
    }

    uint8_t low = rx[0];
    uint8_t high = rx[1];

    uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;

    bool checksum_ok = amt21_checksum_ok(raw);
    uint16_t data14 = raw & 0x3FFF;

    std::cout << "----- AMT21 Read Result -----\n";

    std::cout << "TX command    : 0x"
              << std::hex << std::uppercase
              << std::setw(2) << std::setfill('0')
              << static_cast<int>(node_address) << "\n";

    std::cout << "RX bytes      : "
              << std::setw(2) << static_cast<int>(low) << " "
              << std::setw(2) << static_cast<int>(high)
              << "  low-high\n";

    std::cout << "Raw assembled : 0x"
              << std::setw(4) << raw << "\n";

    std::cout << std::dec;
    std::cout << "K1            : " << ((raw >> 15) & 1) << "\n";
    std::cout << "K0            : " << ((raw >> 14) & 1) << "\n";
    std::cout << "Data14        : " << data14 << "\n";
    std::cout << "Checksum OK   : " << (checksum_ok ? "true" : "false") << "\n";

    if (!checksum_ok)
    {
        std::cout << "Invalid checksum. Discard this sample and retry.\n";
        return false;
    }

    uint16_t position = amt21_position(raw, resolution_bits);
    double angle = amt21_angle_deg(position, resolution_bits);

    std::cout << "Resolution    : " << resolution_bits << "-bit\n";
    std::cout << "Position      : " << position << "\n";
    std::cout << "Angle         : " << std::fixed << std::setprecision(3)
              << angle << " deg\n";

    return true;
}

int main()
{
    std::string port_name = "/dev/ttyU2D2";

    // AMT21 adjustable data-rate 모델 기본값은 보통 115200 bps.
    int baudrate = 115200;

    // 사용자가 테스트한 node address
    uint8_t node_address = 0x84;

    // 모델에 따라 12 또는 14
    int resolution_bits = 14;

    int fd = open_serial_port(port_name, baudrate);

    if (fd < 0)
    {
        return 1;
    }

    while (true)
    {
        read_amt21_once(fd, node_address, resolution_bits);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(fd);
    return 0;
}

// g++ -std=c++17 -O2 -Wall amt21_linux_test.cpp -o amt21_linux_test