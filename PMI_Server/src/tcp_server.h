#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

class DxlBus;

class TcpServer {
public:
    TcpServer();
    ~TcpServer();

    bool start(uint16_t port);
    void stop();

    void setDxlBus(std::shared_ptr<DxlBus> bus) { m_dxl = std::move(bus); }

    TcpServer(const TcpServer &) = delete;
    TcpServer &operator=(const TcpServer &) = delete;

private:
    void acceptLoop(uint16_t port);
    void clientSession(int cfd);

    std::atomic<bool> m_stop{true};
    int m_listenFd = -1;
    std::thread m_thread;
    std::shared_ptr<DxlBus> m_dxl;
};

#endif
