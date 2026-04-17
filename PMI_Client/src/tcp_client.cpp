// Implements TcpClient: connect/send/recv on a background std::thread.

#include "tcp_client.h"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <poll.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>

namespace {

std::string errnoString(const char *what)
{
    return std::string(what) + ": " + std::strerror(errno);
}

int tcpConnect(const std::string &host, uint16_t port, std::string *errOut)
{
    const std::string portStr = std::to_string(port);

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo *res = nullptr;
    const int gai = getaddrinfo(host.c_str(), portStr.c_str(), &hints, &res);
    if (gai != 0) {
        if (errOut)
            *errOut = std::string("getaddrinfo: ") + gai_strerror(gai);
        return -1;
    }

    int fd = -1;
    for (addrinfo *p = res; p != nullptr; p = p->ai_next) {
        fd = static_cast<int>(::socket(p->ai_family, p->ai_socktype, p->ai_protocol));
        if (fd < 0)
            continue;
        if (::connect(fd, p->ai_addr, p->ai_addrlen) == 0)
            break;
        ::close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0 && errOut)
        *errOut = errnoString("connect");

    return fd;
}

} // namespace

TcpClient::TcpClient() = default;

TcpClient::~TcpClient()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop.store(true);
        m_queue.push_back(Task{Op::Shutdown});
    }
    m_cv.notify_all();
    if (m_thread.joinable())
        m_thread.join();
}

void TcpClient::setCallbacks(OnVoid onConnected, OnVoid onDisconnected, OnBytes onReceived, OnError onError)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_onConnected = std::move(onConnected);
    m_onDisconnected = std::move(onDisconnected);
    m_onReceived = std::move(onReceived);
    m_onError = std::move(onError);
}

void TcpClient::start()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_thread.joinable())
            return;
    }
    m_stop.store(false);
    m_thread = std::thread([this]() { threadMain(); });
}

void TcpClient::requestConnect(std::string host, uint16_t port)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        Task t;
        t.op = Op::Connect;
        t.host = std::move(host);
        t.port = port;
        m_queue.push_back(std::move(t));
    }
    m_cv.notify_all();
}

void TcpClient::requestDisconnect()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push_back(Task{Op::Disconnect});
    }
    m_cv.notify_all();
}

void TcpClient::requestSend(std::vector<uint8_t> data)
{
    if (data.empty())
        return;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        Task t;
        t.op = Op::Send;
        t.payload = std::move(data);
        m_queue.push_back(std::move(t));
    }
    m_cv.notify_all();
}

void TcpClient::invokeError(std::string message)
{
    OnError cb;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cb = m_onError;
    }
    if (cb)
        cb(std::move(message));
}

void TcpClient::invokeConnected()
{
    OnVoid cb;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cb = m_onConnected;
    }
    if (cb)
        cb();
}

void TcpClient::invokeDisconnected()
{
    OnVoid cb;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cb = m_onDisconnected;
    }
    if (cb)
        cb();
}

void TcpClient::invokeReceived(std::vector<uint8_t> data)
{
    OnBytes cb;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cb = m_onReceived;
    }
    if (cb)
        cb(std::move(data));
}

bool TcpClient::ioCycle(int sock)
{
    std::deque<Task> batch;
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait_for(lock, std::chrono::milliseconds(50));
        if (m_stop.load()) {
            m_queue.clear();
            return false;
        }
        while (!m_queue.empty()) {
            batch.push_back(std::move(m_queue.front()));
            m_queue.pop_front();
        }
    }

    for (const Task &t : batch) {
        if (t.op == Op::Shutdown || t.op == Op::Disconnect)
            return false;
        if (t.op == Op::Send) {
            const uint8_t *p = t.payload.data();
            size_t remaining = t.payload.size();
            while (remaining > 0) {
                const ssize_t n = ::send(sock, p, remaining, MSG_NOSIGNAL);
                if (n < 0) {
                    if (errno == EINTR)
                        continue;
                    invokeError(errnoString("send"));
                    return false;
                }
                p += static_cast<size_t>(n);
                remaining -= static_cast<size_t>(n);
            }
        }
    }

    pollfd pfd{};
    pfd.fd = sock;
    pfd.events = POLLIN;
    const int pr = ::poll(&pfd, 1, 0);
    if (pr < 0) {
        if (errno != EINTR) {
            invokeError(errnoString("poll"));
            return false;
        }
        return true;
    }

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return false;

    if (pfd.revents & POLLIN) {
        std::vector<uint8_t> chunk;
        chunk.reserve(4096);
        char buf[4096];
        while (true) {
            const ssize_t n = ::recv(sock, buf, sizeof buf, MSG_DONTWAIT);
            if (n > 0) {
                chunk.insert(chunk.end(), buf, buf + n);
            } else if (n == 0) {
                return false;
            } else {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                if (errno == EINTR)
                    continue;
                invokeError(errnoString("recv"));
                return false;
            }
        }
        if (!chunk.empty())
            invokeReceived(std::move(chunk));
    }

    return true;
}

void TcpClient::threadMain()
{
    int sock = -1;

    while (true) {
        Task t;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            while (!m_stop.load() && m_queue.empty())
                m_cv.wait(lock);
            if (m_stop.load()) {
                m_queue.clear();
                break;
            }
            t = std::move(m_queue.front());
            m_queue.pop_front();
        }

        if (t.op == Op::Shutdown)
            break;
        if (t.op == Op::Disconnect)
            continue;
        if (t.op == Op::Send)
            continue;
        if (t.op != Op::Connect)
            continue;

        if (sock >= 0) {
            ::shutdown(sock, SHUT_RDWR);
            ::close(sock);
            sock = -1;
        }

        std::string err;
        const int fd = tcpConnect(t.host, t.port, &err);
        if (fd < 0) {
            invokeError(std::move(err));
            continue;
        }
        int one = 1;
        (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        sock = fd;
        invokeConnected();

        while (sock >= 0 && !m_stop.load()) {
            if (!ioCycle(sock)) {
                ::shutdown(sock, SHUT_RDWR);
                ::close(sock);
                sock = -1;
                invokeDisconnected();
                break;
            }
        }
    }

    if (sock >= 0) {
        ::shutdown(sock, SHUT_RDWR);
        ::close(sock);
    }
}
