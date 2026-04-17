#include "tcp_server.h"

#include "dxl_protocol2.h"
#include "pmi_protocol.h"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

void setNoDelay(int fd)
{
    int one = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

bool setNonBlock(int fd)
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0)
        return false;
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

void fillDummyTelemetry(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount], uint64_t tick)
{
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        axes[i].id_op_mode = pmi::packTelemetryIdOp(static_cast<uint8_t>(i + 1), 1);
        axes[i].servo_state = 1;
        axes[i].present_position = static_cast<double>(i) * 0.01 + 1e-6 * static_cast<double>(tick % 1000000);
        axes[i].present_velocity = static_cast<double>(i) * 0.001;
        axes[i].present_current = 0.1 * static_cast<double>(i);
        axes[i].goal_position = static_cast<double>(i);
        axes[i].goal_velocity = 0.5;
        axes[i].goal_current = 0.2;
        axes[i].error_state = 0;
    }
}

std::string formatPeer(int fd)
{
    sockaddr_storage ss{};
    socklen_t len = sizeof(ss);
    if (::getpeername(fd, reinterpret_cast<sockaddr *>(&ss), &len) != 0)
        return std::string("(unknown peer)");

    if (ss.ss_family == AF_INET) {
        const auto *in = reinterpret_cast<const sockaddr_in *>(&ss);
        char buf[INET_ADDRSTRLEN]{};
        if (!::inet_ntop(AF_INET, &in->sin_addr, buf, sizeof(buf)))
            return std::string("(invalid IPv4)");
        return std::string(buf) + ':' + std::to_string(ntohs(in->sin_port));
    }
    if (ss.ss_family == AF_INET6) {
        const auto *in6 = reinterpret_cast<const sockaddr_in6 *>(&ss);
        char buf[INET6_ADDRSTRLEN]{};
        if (!::inet_ntop(AF_INET6, &in6->sin6_addr, buf, sizeof(buf)))
            return std::string("(invalid IPv6)");
        return std::string("[") + buf + "]:" + std::to_string(ntohs(in6->sin6_port));
    }
    return std::string("(unknown family)");
}

void printDxlTelemetryLines(std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point &lastLog,
    const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    constexpr auto kMinInterval = std::chrono::milliseconds(200);
    if (now - lastLog < kMinInterval)
        return;
    lastLog = now;

    std::cerr << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        const pmi::ServoTelemetry &t = axes[i];
        const int id = static_cast<int>(pmi::telemetryIdFromIdOp(t.id_op_mode));
        const int op = static_cast<int>(pmi::telemetryOpModeFromIdOp(t.id_op_mode));
        std::cerr << "[DXL] ID" << id << " op=" << op << " tq=" << static_cast<int>(t.servo_state) << " pos_deg=" << t.present_position
                  << " vel_dps=" << t.present_velocity << " I_A=" << t.present_current << " gPos_deg=" << t.goal_position
                  << " gVel_dps=" << t.goal_velocity << " gI_A=" << t.goal_current << " err=0x" << std::hex
                  << static_cast<int>(t.error_state) << std::dec << std::endl;
    }
    std::cerr << std::defaultfloat;
}

bool sendTelemetryFrameNonBlock(int cfd, const std::vector<uint8_t> &frame)
{
    const uint8_t *p = frame.data();
    size_t rem = frame.size();
    while (rem > 0) {
        const ssize_t w = ::send(cfd, p, rem, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return true;
            if (errno == EINTR)
                continue;
            return false;
        }
        p += static_cast<size_t>(w);
        rem -= static_cast<size_t>(w);
    }
    return true;
}

} // namespace

void TcpServer::clientSession(int cfd)
{
    setNoDelay(cfd);
    (void)setNonBlock(cfd);

    const std::string peer = formatPeer(cfd);
    std::cerr << "[PMI] client connected: " << peer << std::endl;

    std::vector<uint8_t> rx;
    rx.reserve(4096);

    using clock = std::chrono::steady_clock;
    auto nextTx = clock::now();
    constexpr auto kTxPeriod = std::chrono::milliseconds(10);
    uint64_t txTick = 0;
    auto lastDxlTelemetryLog = clock::time_point{};
    auto lastDxlFailLog = clock::time_point{};

    while (true) {
        const auto now = clock::now();
        if (now >= nextTx) {
            nextTx = now + kTxPeriod;
            ++txTick;
            pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]{};
            if (m_dxl && m_dxl->isOpen()) {
                if (!m_dxl->syncReadTelemetry(axes)) {
                    for (auto &a : axes)
                        a = {};
                    constexpr auto kFailLogInterval = std::chrono::seconds(1);
                    if (now - lastDxlFailLog >= kFailLogInterval) {
                        lastDxlFailLog = now;
                        std::cerr << "[DXL] sync read failed (check bus, baud, IDs 1–4)\n";
                    }
                } else {
                    printDxlTelemetryLines(now, lastDxlTelemetryLog, axes);
                }
            } else {
                fillDummyTelemetry(axes, txTick);
            }
            const std::vector<uint8_t> frame = pmi::buildServerFrame(axes);
            if (!frame.empty() && !sendTelemetryFrameNonBlock(cfd, frame))
                goto client_done;
        }

        int timeoutMs = 50;
        const auto msToNext = std::chrono::duration_cast<std::chrono::milliseconds>(nextTx - clock::now()).count();
        if (msToNext > 0 && msToNext < timeoutMs)
            timeoutMs = static_cast<int>(msToNext);
        if (timeoutMs < 0)
            timeoutMs = 0;

        pollfd pfd{};
        pfd.fd = cfd;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, timeoutMs);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            goto client_done;
        }

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            goto client_done;

        if (pfd.revents & POLLIN) {
            char buf[4096];
            while (true) {
                const ssize_t n = ::recv(cfd, buf, sizeof buf, 0);
                if (n > 0) {
                    rx.insert(rx.end(), buf, buf + n);
                } else if (n == 0) {
                    goto client_done;
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                        break;
                    if (errno == EINTR)
                        continue;
                    goto client_done;
                }
            }

            pmi::feedClientRxStream(rx, [this](uint8_t cmd, const std::vector<uint8_t> & /*payload*/) {
                if (m_dxl)
                    m_dxl->handlePmiClientCommand(cmd);
            });
        }
    }

client_done:
    std::cerr << "[PMI] client disconnected: " << peer << std::endl;
    ::shutdown(cfd, SHUT_RDWR);
    ::close(cfd);
}

TcpServer::TcpServer() = default;

TcpServer::~TcpServer()
{
    stop();
}

bool TcpServer::start(uint16_t port)
{
    stop();

    const int fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0)
        return false;

    int reuse = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return false;
    }
    if (::listen(fd, 8) != 0) {
        ::close(fd);
        return false;
    }

    m_listenFd = fd;
    m_stop.store(false);
    m_thread = std::thread([this, port]() { acceptLoop(port); });
    (void)port;
    return true;
}

void TcpServer::stop()
{
    m_stop.store(true);
    if (m_listenFd >= 0) {
        ::shutdown(m_listenFd, SHUT_RDWR);
        ::close(m_listenFd);
        m_listenFd = -1;
    }
    if (m_thread.joinable())
        m_thread.join();
}

void TcpServer::acceptLoop(uint16_t port)
{
    (void)port;
    while (!m_stop.load()) {
        const int cfd = ::accept(m_listenFd, nullptr, nullptr);
        if (cfd < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        std::thread([this, cfd]() { clientSession(cfd); }).detach();
    }
}
