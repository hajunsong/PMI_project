#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <array>

#include "pmi_protocol.h"

class DxlBus;

class TcpServer {
public:
    TcpServer();
    ~TcpServer();

    bool start(uint16_t telemetryPort, uint16_t commandPort);
    void stop();

    void setDxlBus(std::shared_ptr<DxlBus> bus) { m_dxl = std::move(bus); }

    TcpServer(const TcpServer &) = delete;
    TcpServer &operator=(const TcpServer &) = delete;

private:
    void acceptLoop(int listenFd, bool telemetryOnly);
    void telemetrySession(int cfd);
    void commandSession(int cfd);

    std::atomic<bool> m_stop{true};
    int m_telemetryListenFd = -1;
    int m_commandListenFd = -1;
    std::thread m_telemetryAcceptThread;
    std::thread m_commandAcceptThread;
    std::shared_ptr<DxlBus> m_dxl;

    std::mutex m_latestMutex;
    pmi::ServoTelemetry m_latestAxes[pmi::kTelemetryAxisCount]{};
    bool m_haveLatestTelemetry = false;

    std::mutex m_clientMutex;
    int m_activeTelemetryClientFd = -1;
    int m_activeCommandClientFd = -1;
};

#endif
