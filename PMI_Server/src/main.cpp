// PMI TCP server: accepts connections, parses client frames, streams telemetry from DYNAMIXEL.

#include "dxl_protocol2.h"
#include "tcp_server.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

int main(int argc, char *argv[])
{
    uint16_t telemetryPort = 9000;
    uint16_t commandPort = 9001;
    const char *serialDev = "/dev/ttyUSB1";
    int baud = 2000000;
    bool noDevice = false;
    int positionalIdx = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--no-device") {
            noDevice = true;
            continue;
        }

        // Backward-compatible positional args:
        // [telemetry_port] [serial_device] [baud]
        if (positionalIdx == 0) {
            const long p = std::strtol(arg.c_str(), nullptr, 10);
            if (p > 0 && p <= 65535) {
                telemetryPort = static_cast<uint16_t>(p);
                commandPort = static_cast<uint16_t>(p + 1);
                positionalIdx++;
                continue;
            }
        }
        if (positionalIdx == 1) {
            serialDev = argv[i];
            positionalIdx++;
            continue;
        }
        if (positionalIdx == 2) {
            const long b = std::strtol(arg.c_str(), nullptr, 10);
            if (b > 0) {
                baud = static_cast<int>(b);
                positionalIdx++;
                continue;
            }
        }

        std::cerr << "Unknown or invalid argument: " << arg << "\n";
        std::cerr << "Usage: PMI_Server [telemetry_port] [serial_device] [baud] [--no-device]\n";
        return 1;
    }

    std::shared_ptr<DxlBus> dxl;
    if (!noDevice) {
        dxl = std::make_shared<DxlBus>();
        if (!dxl->open(serialDev, baud)) {
            std::cerr << "DxlBus::open failed: " << serialDev << " @ " << baud;
            if (errno != 0)
                std::cerr << " (" << std::strerror(errno) << ")\n";
            else
                std::cerr << " (port init or DYNAMIXEL setup failed; errno unchanged)\n";
            std::cerr << "Usage: PMI_Server [telemetry_port] [serial_device] [baud] [--no-device]\n";
            return 1;
        }
    }

    TcpServer server;
    server.setDxlBus(dxl);
    if (!server.start(telemetryPort, commandPort)) {
        std::cerr << "Failed to bind/listen on ports " << telemetryPort << " (telemetry), " << commandPort << " (command)\n";
        return 1;
    }

    if (noDevice) {
        std::cout << "PMI_Server telemetry:" << telemetryPort << " command:" << commandPort
                  << ", no-device mode (dummy telemetry, Ctrl+C to stop)\n";
    } else {
        std::cout << "PMI_Server telemetry:" << telemetryPort << " command:" << commandPort
                  << ", DYNAMIXEL " << serialDev << " @ " << baud
                  << " (IDs 1–4, Ctrl+C to stop)\n";
    }
    std::cout.flush();

    for (;;)
        pause();

    return 0;
}
