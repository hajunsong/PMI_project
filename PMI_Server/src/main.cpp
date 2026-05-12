// PMI TCP server: accepts connections, parses client frames, streams telemetry from DYNAMIXEL.

#include "dxl_protocol2.h"
#include "tcp_server.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

namespace {

struct SerialRuntimeConfig {
    std::string dxlDevice = "/dev/ttyUSB0";
    int dxlBaud = 2000000;
    std::string amt21Device = "/dev/ttyUSB1";
    int amt21Baud = 115200;
};

std::string serialConfigFilePath()
{
#ifdef PMI_SERVER_DATA_DIR
    return std::string(PMI_SERVER_DATA_DIR) + "/pmi_serial_config.txt";
#else
    return "pmi_serial_config.txt";
#endif
}

std::string trimAscii(std::string s)
{
    auto isSpace = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!s.empty() && isSpace(static_cast<unsigned char>(s.front())))
        s.erase(s.begin());
    while (!s.empty() && isSpace(static_cast<unsigned char>(s.back())))
        s.pop_back();
    return s;
}

void loadSerialConfigFromFile(SerialRuntimeConfig &cfg)
{
    const std::string path = serialConfigFilePath();
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "[PMI] Serial config not found at " << path << " — using built-in defaults\n";
        return;
    }

    std::string line;
    int lineNo = 0;
    while (std::getline(ifs, line)) {
        ++lineNo;
        const auto hash = line.find('#');
        if (hash != std::string::npos)
            line.erase(hash);
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            if (!trimAscii(line).empty())
                std::cerr << "[PMI] Serial config: ignoring malformed line " << lineNo << "\n";
            continue;
        }
        std::string key = trimAscii(line.substr(0, eq));
        std::string val = trimAscii(line.substr(eq + 1));
        if (key.empty())
            continue;
        std::transform(key.begin(), key.end(), key.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (key == "dxl_serial_device") {
            cfg.dxlDevice = val;
            continue;
        }
        if (key == "amt21_serial_device") {
            cfg.amt21Device = val;
            continue;
        }

        char *endp = nullptr;
        if (key == "dxl_baud") {
            const long b = std::strtol(val.c_str(), &endp, 10);
            if (endp != val.c_str() && b > 0)
                cfg.dxlBaud = static_cast<int>(b);
            else
                std::cerr << "[PMI] Serial config: bad dxl_baud on line " << lineNo << "\n";
            continue;
        }
        if (key == "amt21_baud") {
            const long b = std::strtol(val.c_str(), &endp, 10);
            if (endp != val.c_str() && b > 0)
                cfg.amt21Baud = static_cast<int>(b);
            else
                std::cerr << "[PMI] Serial config: bad amt21_baud on line " << lineNo << "\n";
            continue;
        }

        std::cerr << "[PMI] Serial config: unknown key '" << key << "' (line " << lineNo << ")\n";
    }

    std::cerr << "[PMI] Serial config loaded from " << path << " (dxl=" << cfg.dxlDevice << " @ " << cfg.dxlBaud
              << ", amt21=" << (cfg.amt21Device.empty() ? std::string("(disabled)") : cfg.amt21Device) << " @ "
              << cfg.amt21Baud << ")\n";
}

} // namespace

int main(int argc, char *argv[])
{
    uint16_t telemetryPort = 9000;
    uint16_t commandPort = 9001;
    SerialRuntimeConfig serialCfg;
    loadSerialConfigFromFile(serialCfg);

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
            serialCfg.dxlDevice = argv[i];
            positionalIdx++;
            continue;
        }
        if (positionalIdx == 2) {
            const long b = std::strtol(arg.c_str(), nullptr, 10);
            if (b > 0) {
                serialCfg.dxlBaud = static_cast<int>(b);
                positionalIdx++;
                continue;
            }
        }

        std::cerr << "Unknown or invalid argument: " << arg << "\n";
        std::cerr << "Usage: PMI_Server [telemetry_port] [serial_device] [baud] [--no-device]\n";
        std::cerr << "       Serial defaults + AMT21: " << serialConfigFilePath() << "\n";
        return 1;
    }

    std::shared_ptr<DxlBus> dxl;
    if (!noDevice) {
        dxl = std::make_shared<DxlBus>();
        const char *amt21Path = serialCfg.amt21Device.empty() ? nullptr : serialCfg.amt21Device.c_str();
        if (!dxl->open(serialCfg.dxlDevice.c_str(), serialCfg.dxlBaud, amt21Path, serialCfg.amt21Baud)) {
            std::cerr << "DxlBus::open failed: " << serialCfg.dxlDevice << " @ " << serialCfg.dxlBaud;
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
        std::cout << "PMI_Server telemetry:" << telemetryPort << " command:" << commandPort << ", DYNAMIXEL "
                  << serialCfg.dxlDevice << " @ " << serialCfg.dxlBaud;
        if (!serialCfg.amt21Device.empty())
            std::cout << ", AMT21 " << serialCfg.amt21Device << " @ " << serialCfg.amt21Baud;
        else
            std::cout << ", AMT21 (disabled)";
        std::cout << " (IDs 1–4, Ctrl+C to stop)\n";
    }
    std::cout.flush();

    for (;;)
        pause();

    return 0;
}
