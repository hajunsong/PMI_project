#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

// TCP client on a worker thread (POSIX sockets, no Qt in this module).

#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class TcpClient {
public:
    using OnVoid = std::function<void()>;
    using OnBytes = std::function<void(std::vector<uint8_t>)>;
    using OnError = std::function<void(std::string)>;

    TcpClient();
    ~TcpClient();

    void setCallbacks(OnVoid onConnected, OnVoid onDisconnected, OnBytes onReceived, OnError onError);

    void start();
    void requestConnect(std::string host, uint16_t port);
    void requestDisconnect();
    void requestSend(std::vector<uint8_t> data);

    TcpClient(const TcpClient &) = delete;
    TcpClient &operator=(const TcpClient &) = delete;

private:
    enum class Op { Connect, Disconnect, Send, Shutdown };
    struct Task {
        Op op = Op::Shutdown;
        std::string host;
        uint16_t port = 0;
        std::vector<uint8_t> payload;
    };

    void threadMain();
    bool ioCycle(int sock);

    void invokeError(std::string message);
    void invokeConnected();
    void invokeDisconnected();
    void invokeReceived(std::vector<uint8_t> data);

    OnVoid m_onConnected;
    OnVoid m_onDisconnected;
    OnBytes m_onReceived;
    OnError m_onError;

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<Task> m_queue;
    std::atomic<bool> m_stop{false};
    std::thread m_thread;
};

#endif
