// PMI TCP server: accepts connections, parses client frames, streams telemetry from DYNAMIXEL.

#include "dxl_protocol2.h"
#include "tcp_server.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <unistd.h>

int main(int argc, char *argv[])
{
    uint16_t port = 9000;
    const char *serialDev = "/dev/CM904";
    int baud = 1000000;

    if (argc >= 2) {
        const long p = std::strtol(argv[1], nullptr, 10);
        if (p > 0 && p <= 65535)
            port = static_cast<uint16_t>(p);
    }
    if (argc >= 3)
        serialDev = argv[2];
    if (argc >= 4) {
        const long b = std::strtol(argv[3], nullptr, 10);
        if (b > 0)
            baud = static_cast<int>(b);
    }

    auto dxl = std::make_shared<DxlBus>();
    if (!dxl->open(serialDev, baud)) {
        std::cerr << "DxlBus::open failed: " << serialDev << " @ " << baud;
        if (errno != 0)
            std::cerr << " (" << std::strerror(errno) << ")\n";
        else
            std::cerr << " (port init or DYNAMIXEL setup failed; errno unchanged)\n";
        std::cerr << "Usage: PMI_Server [tcp_port] [serial_device] [baud]\n";
        return 1;
    }

    TcpServer server;
    server.setDxlBus(dxl);
    if (!server.start(port)) {
        std::cerr << "Failed to bind/listen on port " << port << "\n";
        return 1;
    }

    std::cout << "PMI_Server TCP port " << port << ", DYNAMIXEL " << serialDev << " @ " << baud
              << " (IDs 1–4, Ctrl+C to stop)\n";
    std::cout.flush();

    for (;;)
        pause();

    return 0;
}
