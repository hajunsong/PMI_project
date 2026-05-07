#include "tcp_server.h"

#include "dxl_protocol2.h"
#include "path_planner.h"
#include "pmi_protocol.h"
#include "server_logger.h"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
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
        axes[i].encoder_position = axes[i].present_position;
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
                  << " enc_deg=" << t.encoder_position
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

double readF64LE(const uint8_t *src)
{
    std::uint64_t u = 0;
    for (int b = 0; b < 8; ++b)
        u |= static_cast<std::uint64_t>(src[b]) << (8 * b);
    double v = 0.0;
    std::memcpy(&v, &u, 8);
    return v;
}

bool parseLogStartPayload(const std::vector<uint8_t> &payload, double &durationSecOut)
{
    if (payload.size() != 8)
        return false;
    durationSecOut = readF64LE(payload.data());
    return std::isfinite(durationSecOut) && durationSecOut > 0.0;
}

bool parseWaypointPayload(const std::vector<uint8_t> &payload, std::vector<PathPlanner::Waypoint> &out)
{
    out.clear();
    if (payload.empty())
        return false;
    const std::size_t count = payload[0];
    if (count < 2)
        return false;
    const std::size_t need = 1 + count * 32;
    if (payload.size() != need)
        return false;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t off = 1 + i * 32;
        PathPlanner::Waypoint wp;
        wp.t = readF64LE(payload.data() + off);
        wp.x = readF64LE(payload.data() + off + 8);
        wp.y = readF64LE(payload.data() + off + 16);
        wp.z = readF64LE(payload.data() + off + 24);
        if (!std::isfinite(wp.t) || !std::isfinite(wp.x) || !std::isfinite(wp.y) || !std::isfinite(wp.z))
            return false;
        out.push_back(wp);
    }
    for (std::size_t i = 1; i < out.size(); ++i) {
        if (out[i].t <= out[i - 1].t)
            return false;
    }
    return true;
}

bool parseInitialJointPosePayload(const std::vector<uint8_t> &payload, std::array<double, 4> &jointRadOut)
{
    if (payload.size() != 32)
        return false;
    for (int i = 0; i < 4; ++i) {
        const double v = readF64LE(payload.data() + i * 8);
        if (!std::isfinite(v))
            return false;
        jointRadOut[static_cast<size_t>(i)] = v;
    }
    return true;
}

struct InitPoseMotion {
    bool active = false;
    std::chrono::steady_clock::time_point startTime{};
    std::array<double, 4> qStartRad{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> qTargetRad{{0.0, 0.0, 0.0, 0.0}};
};

double quinticBlend(double r)
{
    if (r <= 0.0)
        return 0.0;
    if (r >= 1.0)
        return 1.0;
    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r3 * r;
    const double r5 = r4 * r;
    return 10.0 * r3 - 15.0 * r4 + 6.0 * r5;
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
    auto nextPoll = clock::now();
    auto nextTx = clock::now();
    constexpr auto kPollPeriod = std::chrono::milliseconds(5);
    constexpr auto kTxPeriod = std::chrono::milliseconds(100);
    uint64_t txTick = 0;
    auto lastDxlTelemetryLog = clock::time_point{};
    auto lastDxlFailLog = clock::time_point{};
    pmi::ServoTelemetry latestAxes[pmi::kTelemetryAxisCount]{};
    bool haveLatestTelemetry = false;
    PathPlanner planner;
    std::vector<PathPlanner::Waypoint> waypoints;
    ServerLogger logger;
    constexpr double kPlannerDt = 0.005;
    constexpr double kGear[4] = {32.0 / 60.0, 360.0 / 54.0, 360.0 / 108.0, 360.0 / 108.0};
    constexpr double kInitPoseMoveSec = 5.0;
    InitPoseMotion initPoseMotion;
    auto lastInitProgressAck = clock::time_point{};

    while (true) {
        const auto now = clock::now();
        if (now >= nextPoll) {
            nextPoll = now + kPollPeriod;
            ++txTick;
            pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]{};
            if (m_dxl && m_dxl->isOpen()) {
                if (!m_dxl->syncReadTelemetry(axes)) {
                    constexpr auto kFailLogInterval = std::chrono::seconds(1);
                    if (now - lastDxlFailLog >= kFailLogInterval) {
                        lastDxlFailLog = now;
                        std::cerr << "[DXL] sync read failed (check bus, baud, IDs 1–4)\n";
                    }
                } else {
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
                        latestAxes[i] = axes[i];
                    haveLatestTelemetry = true;
                    printDxlTelemetryLines(now, lastDxlTelemetryLog, latestAxes);
                }
            } else {
                fillDummyTelemetry(latestAxes, txTick);
                haveLatestTelemetry = true;
            }
            ServerLogger::PathDesiredPose pathDesired{};
            PathPlanner::DesiredPose plannerDesired{};
            if (planner.currentDesiredPose(plannerDesired)) {
                pathDesired.valid = true;
                pathDesired.x = plannerDesired.x;
                pathDesired.y = plannerDesired.y;
                pathDesired.z = plannerDesired.z;
                pathDesired.roll = plannerDesired.roll;
                pathDesired.pitch = plannerDesired.pitch;
                pathDesired.yaw = plannerDesired.yaw;
            }
            logger.updateLatest(latestAxes, haveLatestTelemetry, pathDesired);

            if (planner.isRunning()) {
                std::array<double, 4> qJointRad{};
                if (planner.step(qJointRad)) {
                    std::array<double, pmi::kTelemetryAxisCount> motorDeg{};
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                        const double qJointDeg = qJointRad[i] * 180.0 / M_PI;
                        motorDeg[i] = qJointDeg / kGear[i];
                        latestAxes[i].goal_position = qJointDeg;
                    }
                    if (m_dxl && m_dxl->isOpen())
                        (void)m_dxl->writeGoalPositionDeg(motorDeg);
                } else {
                    planner.stop();
                }
            }

            if (initPoseMotion.active) {
                const double elapsedSec =
                    std::chrono::duration_cast<std::chrono::duration<double>>(now - initPoseMotion.startTime).count();
                const double r = elapsedSec / kInitPoseMoveSec;
                const double s = quinticBlend(r);
                const double progressPercent = std::min(100.0, std::max(0.0, r * 100.0));
                std::array<double, 4> qJointRad{};
                std::array<double, pmi::kTelemetryAxisCount> motorDeg{};
                for (size_t i = 0; i < 4; ++i) {
                    qJointRad[i] =
                        initPoseMotion.qStartRad[i] + (initPoseMotion.qTargetRad[i] - initPoseMotion.qStartRad[i]) * s;
                    const double qJointDeg = qJointRad[i] * 180.0 / M_PI;
                    motorDeg[i] = qJointDeg / kGear[i];
                    latestAxes[i].goal_position = qJointDeg;
                }
                if (m_dxl && m_dxl->isOpen())
                    (void)m_dxl->writeGoalPositionDeg(motorDeg);
                constexpr auto kAckInterval = std::chrono::milliseconds(200);
                if (lastInitProgressAck == clock::time_point{} || now - lastInitProgressAck >= kAckInterval) {
                    lastInitProgressAck = now;
                    const std::string msg = "INIT_POSE_PROGRESS:" + std::to_string(progressPercent);
                    (void)sendTelemetryFrameNonBlock(
                        cfd, pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                }

                if (r >= 1.0) {
                    initPoseMotion.active = false;
                    planner.setInitialJointRad(initPoseMotion.qTargetRad);
                    const std::string msg = "INIT_POSE_DONE";
                    (void)sendTelemetryFrameNonBlock(
                        cfd, pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    std::cerr << "[PMI] initial joint pose motion completed (5s quintic)\n";
                }
            }
        }

        if (now >= nextTx) {
            nextTx = now + kTxPeriod;
            if (haveLatestTelemetry) {
                const std::vector<uint8_t> frame = pmi::buildServerFrame(latestAxes);
                if (!frame.empty() && !sendTelemetryFrameNonBlock(cfd, frame))
                    goto client_done;
            }
        }

        int timeoutMs = 50;
        const auto soonestDeadline = (nextPoll < nextTx) ? nextPoll : nextTx;
        const auto msToNext = std::chrono::duration_cast<std::chrono::milliseconds>(soonestDeadline - clock::now()).count();
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

            pmi::feedClientRxStream(
                rx, [this, &planner, &waypoints, &latestAxes, &kGear, &haveLatestTelemetry, &initPoseMotion, &logger, &cfd, now](
                        uint8_t cmd, const std::vector<uint8_t> &payload) {
                if (cmd == pmi::kCmdSetWaypointBatch) {
                    if (parseWaypointPayload(payload, waypoints)) {
                        planner.setWaypoints(waypoints);
                        std::cerr << "[PMI] received " << waypoints.size() << " waypoint(s)\n";
                    } else {
                        std::cerr << "[PMI] invalid waypoint payload\n";
                    }
                    return;
                }
                if (cmd == pmi::kCmdPlanPath) {
                    if (planner.plan(kPlannerDt))
                        std::cerr << "[PMI] trajectory planned\n";
                    else
                        std::cerr << "[PMI] trajectory plan failed\n";
                    return;
                }
                if (cmd == pmi::kCmdStartTrajectoryIk) {
                    planner.start();
                    std::cerr << "[PMI] trajectory IK start\n";
                    return;
                }
                if (cmd == pmi::kCmdStopTrajectoryIk) {
                    planner.stop();
                    std::cerr << "[PMI] trajectory IK stop\n";
                    return;
                }
                if (cmd == pmi::kCmdSetInitialJointPose) {
                    std::array<double, 4> qJointRad{};
                    if (!parseInitialJointPosePayload(payload, qJointRad)) {
                        std::cerr << "[PMI] invalid initial joint pose payload\n";
                        return;
                    }

                    std::array<double, 4> qCurrentRad{};
                    if (haveLatestTelemetry) {
                        for (size_t i = 0; i < 4; ++i) {
                            const double qCurrentJointDeg = latestAxes[i].present_position * kGear[i];
                            qCurrentRad[i] = qCurrentJointDeg * M_PI / 180.0;
                        }
                    } else {
                        // Fallback when telemetry has not been received yet.
                        qCurrentRad = qJointRad;
                    }

                    initPoseMotion.active = true;
                    initPoseMotion.startTime = now;
                    initPoseMotion.qStartRad = qCurrentRad;
                    initPoseMotion.qTargetRad = qJointRad;
                    planner.stop();

                    for (size_t i = 0; i < 4; ++i) {
                        const double qJointDeg = qJointRad[i] * 180.0 / M_PI;
                        latestAxes[i].goal_position = qJointDeg;
                    }
                    std::cerr << "[PMI] initial joint pose motion started (5s quintic)\n";
                    return;
                }
                if (cmd == pmi::kCmdLogStart) {
                    double durationSec = 0.0;
                    if (!parseLogStartPayload(payload, durationSec)) {
                        std::cerr << "[PMI] invalid log start payload\n";
                        const std::string msg = "LOG_START_FAIL:invalid payload";
                        (void)sendTelemetryFrameNonBlock(cfd, pmi::buildServerAckFrame(
                                                              pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                        return;
                    }
                    if (!logger.start(durationSec)) {
                        std::cerr << "[PMI] failed to start logger\n";
                        const std::string msg = "LOG_START_FAIL:start failed";
                        (void)sendTelemetryFrameNonBlock(cfd, pmi::buildServerAckFrame(
                                                              pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    } else {
                        const std::string msg = std::string("LOG_START_OK:") + logger.currentLogPath();
                        (void)sendTelemetryFrameNonBlock(cfd, pmi::buildServerAckFrame(
                                                              pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    }
                    return;
                }
                if (cmd == pmi::kCmdLogStop) {
                    logger.stop();
                    const std::string msg = std::string("LOG_STOP_OK:") + logger.currentLogPath();
                    (void)sendTelemetryFrameNonBlock(
                        cfd, pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    return;
                }
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
