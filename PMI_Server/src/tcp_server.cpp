#include "tcp_server.h"

#include "dxl_protocol2.h"
#include "path_planner.h"
#include "pmi_protocol.h"
#include "server_logger.h"

#include <pmi_kinematics/pmi_kinematics.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
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
    constexpr bool kEnableDxlTelemetryLog = false;
    if (!kEnableDxlTelemetryLog)
        return;

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

bool parseJogVelocityPayload(const std::vector<uint8_t> &payload, uint8_t &axisOut, double &jointVelDegPerSecOut)
{
    if (payload.size() != 9)
        return false;
    axisOut = payload[0];
    if (axisOut >= pmi::kTelemetryAxisCount)
        return false;
    jointVelDegPerSecOut = readF64LE(payload.data() + 1);
    return std::isfinite(jointVelDegPerSecOut);
}

struct InitPoseMotion {
    bool active = false;
    std::chrono::steady_clock::time_point startTime{};
    std::array<double, 4> qStartRad{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> qTargetRad{{0.0, 0.0, 0.0, 0.0}};
    /// After quintic time is done, hold final setpoint until joint feedback is within tolerance (or timeout).
    bool waitingReach = false;
    std::chrono::steady_clock::time_point reachWaitStart{};
    /// Operating modes before `switchAllAxesToVelocityOperatingMode` (restored when init pose ends).
    std::array<uint8_t, 4> savedMotorModes{};
    bool savedMotorModesValid = false;
};

struct ExternalEncoderVelocityControl {
    bool active = false;
    std::array<double, 4> targetJointDeg{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> integralErr{{0.0, 0.0, 0.0, 0.0}};
};

struct JogState {
    bool active = false;
    size_t axis = 0;
    double jointVelDegPerSec = 0.0;
    /// Modes for all axes before temporary switch to velocity control during jog.
    std::array<uint8_t, 4> savedMotorModes{};
    bool savedMotorModesValid = false;
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

double clampValue(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

double wrapToPi(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    if (wrapped < 0.0)
        wrapped += 2.0 * kPi;
    return wrapped - kPi;
}

/// VSD task-space PD + gravity comp parameters (loaded from `pmi_vsd_config.txt` each session).
/// Built-in defaults match analysis/cpp `ControlMain::run_vsd` — keep these as a safety fallback if the
/// config file is missing or malformed; in normal operation the file contents take precedence.
struct VsdConfig {
    bool enable = true;
    std::array<double, 5> Ks{{218279.026146, 230800.488552, 159168.050363, 2433.229196, 16124.352342}};
    std::array<double, 5> Kd{{1109.348611, 1707.380149, 5502.23545, 16.731615, 7.095061}};
    double ktNmPerA = 2.41;
    double maxAmp = 1.6;
    double maxJointTauNm = 25.0;
    /// Linear gain ramp-in window applied at trajectory start. The Ks/Kd terms are scaled by
    /// `t / rampInSec` (clipped to [0,1]) for the first `rampInSec` seconds after the planner is
    /// started, so any residual joint mismatch between planner-q[0] and actual encoder doesn't
    /// produce an instantaneous saturating kick. Set to 0 to disable.
    double rampInSec = 0.5;
};

std::string vsdConfigFilePath()
{
#ifdef PMI_SERVER_DATA_DIR
    return std::string(PMI_SERVER_DATA_DIR) + "/pmi_vsd_config.txt";
#else
    return "pmi_vsd_config.txt";
#endif
}

std::string trimAscii(std::string s)
{
    auto isSpace = [](unsigned char c) { return std::isspace(c); };
    while (!s.empty() && isSpace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && isSpace(static_cast<unsigned char>(s.back()))) s.pop_back();
    return s;
}

/// Read VSD config from `pmi_vsd_config.txt` (KEY = VALUE; '#' starts a comment, keys case-insensitive).
/// Missing keys keep the in-struct defaults; missing/unreadable file logs once and uses all defaults.
VsdConfig loadVsdConfigFromFile()
{
    VsdConfig cfg;
    const std::string path = vsdConfigFilePath();
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "[PMI] VSD config not found at " << path << " — using built-in defaults\n";
        return cfg;
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
                std::cerr << "[PMI] VSD config: ignoring malformed line " << lineNo << "\n";
            continue;
        }
        std::string key = trimAscii(line.substr(0, eq));
        std::string val = trimAscii(line.substr(eq + 1));
        if (key.empty() || val.empty())
            continue;
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        char *endp = nullptr;
        const double parsed = std::strtod(val.c_str(), &endp);
        if (endp == val.c_str()) {
            std::cerr << "[PMI] VSD config: bad number on line " << lineNo << " (" << key << ")\n";
            continue;
        }

        if (key == "enable")             cfg.enable = (parsed >= 0.5);
        else if (key == "ks_x")          cfg.Ks[0] = parsed;
        else if (key == "ks_y")          cfg.Ks[1] = parsed;
        else if (key == "ks_z")          cfg.Ks[2] = parsed;
        else if (key == "ks_roll")       cfg.Ks[3] = parsed;
        else if (key == "ks_pitch")      cfg.Ks[4] = parsed;
        else if (key == "kd_x")          cfg.Kd[0] = parsed;
        else if (key == "kd_y")          cfg.Kd[1] = parsed;
        else if (key == "kd_z")          cfg.Kd[2] = parsed;
        else if (key == "kd_roll")       cfg.Kd[3] = parsed;
        else if (key == "kd_pitch")      cfg.Kd[4] = parsed;
        else if (key == "kt_nm_per_a")   cfg.ktNmPerA = parsed;
        else if (key == "max_amp")       cfg.maxAmp = parsed;
        else if (key == "max_joint_tau_nm") cfg.maxJointTauNm = parsed;
        else if (key == "ramp_in_sec")   cfg.rampInSec = std::max(0.0, parsed);
        else
            std::cerr << "[PMI] VSD config: unknown key '" << key << "' (line " << lineNo << ")\n";
    }

    if (cfg.ktNmPerA < 1e-6) {
        std::cerr << "[PMI] VSD config: kt_nm_per_a too small — reverting to 2.41\n";
        cfg.ktNmPerA = 2.41;
    }

    std::cerr << "[PMI] VSD config loaded from " << path
              << " (enable=" << (cfg.enable ? 1 : 0)
              << ", Ks=[" << cfg.Ks[0] << ", " << cfg.Ks[1] << ", " << cfg.Ks[2] << ", "
              << cfg.Ks[3] << ", " << cfg.Ks[4] << "]"
              << ", Kd=[" << cfg.Kd[0] << ", " << cfg.Kd[1] << ", " << cfg.Kd[2] << ", "
              << cfg.Kd[3] << ", " << cfg.Kd[4] << "]"
              << ", Kt=" << cfg.ktNmPerA << " Nm/A"
              << ", max_amp=" << cfg.maxAmp << " A"
              << ", max_tau=" << cfg.maxJointTauNm << " Nm)\n";
    return cfg;
}

/// Zero Goal Velocity and Goal Current so stopping jog / switching cmds does not leave torque in Current Mode.
void zeroMotorGoalVelocityAndCurrent(DxlBus *dxl)
{
    if (!dxl || !dxl->isOpen())
        return;
    std::array<double, pmi::kTelemetryAxisCount> z{};
    z.fill(0.0);
    (void)dxl->writeGoalVelocityDegPerSec(z);
    (void)dxl->writeGoalCurrentAmp(z);
}

void restoreSavedOperatingModes(DxlBus *dxl, const std::array<uint8_t, 4> &modes, bool valid)
{
    if (!dxl || !dxl->isOpen() || !valid)
        return;
    constexpr int16_t kCurrentBasedPosGoalCurrentRaw = 350;
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        (void)dxl->setAxisTorque(i, false);
        (void)dxl->setAxisOperatingMode(i, modes[i]);
        if (modes[i] == 5)
            (void)dxl->setAxisGoalCurrentRaw(i, kCurrentBasedPosGoalCurrentRaw);
        (void)dxl->setAxisTorque(i, true);
    }
}

void switchAllAxesToVelocityOperatingMode(DxlBus *dxl)
{
    if (!dxl || !dxl->isOpen())
        return;
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
        (void)dxl->setAxisTorque(i, false);
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
        (void)dxl->setAxisOperatingMode(i, 1);
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
        (void)dxl->setAxisTorque(i, true);
}

bool refreshLatestAxesFromDxl(DxlBus *dxl, pmi::ServoTelemetry *latestAxes, bool *haveLatest,
                              const double encoderSign[4])
{
    if (!dxl || !dxl->isOpen())
        return false;
    pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]{};
    if (!dxl->syncReadTelemetry(axes))
        return false;
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        latestAxes[i] = axes[i];
        if ((i > 0) && std::isfinite(latestAxes[i].encoder_position))
            latestAxes[i].encoder_position *= encoderSign[i];
    }
    *haveLatest = true;
    return true;
}

void endInitPoseTemporaryVelocity(DxlBus *dxl, InitPoseMotion &m)
{
    if (m.savedMotorModesValid && dxl && dxl->isOpen()) {
        restoreSavedOperatingModes(dxl, m.savedMotorModes, true);
        zeroMotorGoalVelocityAndCurrent(dxl);
    }
    m.active = false;
    m.waitingReach = false;
    m.savedMotorModesValid = false;
}

void stopJogRestoreHardware(JogState &j, DxlBus *dxl)
{
    if (!j.active)
        return;
    if (dxl && dxl->isOpen()) {
        zeroMotorGoalVelocityAndCurrent(dxl);
        if (j.savedMotorModesValid) {
            restoreSavedOperatingModes(dxl, j.savedMotorModes, true);
            j.savedMotorModesValid = false;
        }
    } else {
        j.savedMotorModesValid = false;
    }
    j.active = false;
}

} // namespace

void TcpServer::commandSession(int cfd)
{
    setNoDelay(cfd);
    (void)setNonBlock(cfd);

    const std::string peer = formatPeer(cfd);
    std::cerr << "[PMI] client connected: " << peer << std::endl;

    std::vector<uint8_t> rx;
    rx.reserve(4096);
    std::mutex rxMutex;
    std::vector<uint8_t> rxShared;
    rxShared.reserve(4096);
    std::mutex ackMutex;
    std::atomic<bool> running{true};

    using clock = std::chrono::steady_clock;
    auto nextPoll = clock::now();
    constexpr int kPollPeriodMs = 2;
    constexpr auto kPollPeriod = std::chrono::milliseconds(kPollPeriodMs);
    constexpr auto kCommandCommPeriod = std::chrono::milliseconds(100);
    std::deque<std::vector<uint8_t>> pendingCommandFrames;
    uint64_t txTick = 0;
    auto lastDxlTelemetryLog = clock::time_point{};
    auto lastDxlFailLog = clock::time_point{};
    auto lastGoalWriteFailLog = clock::time_point{};
    pmi::ServoTelemetry latestAxes[pmi::kTelemetryAxisCount]{};
    bool haveLatestTelemetry = false;
    PathPlanner planner;
    std::vector<PathPlanner::Waypoint> waypoints;
    ServerLogger logger;
    // 2 ms poll: `kPlannerDt` == control period == quintic path step (`plan(dt)`) == PI integral dt.
    constexpr double kPlannerDt = static_cast<double>(kPollPeriodMs) / 1000.0;
    constexpr double kGear[4] = {32.0 / 60.0, 54.0 / 360.0, 108.0 / 360.0, 108.0 / 360.0};
    // External encoder direction compensation per axis (axis4 reversed).
    constexpr double kEncoderSign[4] = {1.0, 1.0, 1.0, -1.0};
    const double *const encSignForRx = kEncoderSign;
    constexpr double kInitPoseMoveSec = 5.0;
    // Init-pose "done" uses same feedback convention as the outer PI (encoder joint deg vs motor×gear).
    // Axis 0: motor-only fb → slightly looser; axes 1–3: encoder → tight reach band.
    constexpr std::array<double, 4> kInitPoseReachTolDeg{{2.0, 0.5, 0.5, 0.5}};
    constexpr double kInitPoseReachMaxWaitSec = 30.0;
    uint8_t requestedControlOpMode = 0xFF; // 0xFF: follow telemetry op mode
    InitPoseMotion initPoseMotion;
    ExternalEncoderVelocityControl extVelControl;
    JogState jogState;
    auto lastInitProgressAck = clock::time_point{};

    // VSD task-space PD + gravity comp: matches `analysis/cpp` ControlMain::run_vsd.
    // Reloaded each new client session so editing pmi_vsd_config.txt + reconnecting takes effect.
    const VsdConfig vsdConfig = loadVsdConfigFromFile();
    // Joint torque [Nm] → motor current [A] gain. motor_torque = joint_torque * gear; current = motor_torque / Kt.
    const double vsdAmpPerNm = 1.0 / vsdConfig.ktNmPerA;
    const double vsdMaxAmp = vsdConfig.maxAmp;
    const double vsdMaxJointTorqueNm = vsdConfig.maxJointTauNm;
    const bool vsdEnable = vsdConfig.enable;
    // Tracked across loop iterations: time at which the planner most recently transitioned from
    // "not running" to "running". Used by the gain-ramp scaling at the top of each VSD step.
    bool vsdPlannerWasRunning = false;
    auto vsdRampStart = clock::time_point{};
    auto enqueueCommandFrame = [&](std::vector<uint8_t> frame) {
        if (!frame.empty())
            std::lock_guard<std::mutex> lock(ackMutex);
            pendingCommandFrames.emplace_back(std::move(frame));
    };

    std::thread netThread([&]() {
        using net_clock = std::chrono::steady_clock;
        auto nextCommandTx = net_clock::now();
        pollfd pfd{};
        pfd.fd = cfd;
        pfd.events = POLLIN;
        while (running.load()) {
            int timeoutMs = 20;
            const auto msToNextTx = std::chrono::duration_cast<std::chrono::milliseconds>(nextCommandTx - net_clock::now()).count();
            if (msToNextTx > 0 && msToNextTx < timeoutMs)
                timeoutMs = static_cast<int>(msToNextTx);
            if (timeoutMs < 0)
                timeoutMs = 0;

            const int pr = ::poll(&pfd, 1, timeoutMs);
            if (pr < 0) {
                if (errno == EINTR)
                    continue;
                running.store(false);
                break;
            }
            if (pr > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
                running.store(false);
                break;
            }
            if (pr > 0 && (pfd.revents & POLLIN)) {
                char buf[4096];
                while (running.load()) {
                    const ssize_t n = ::recv(cfd, buf, sizeof buf, MSG_DONTWAIT);
                    if (n > 0) {
                        std::lock_guard<std::mutex> lock(rxMutex);
                        rxShared.insert(rxShared.end(), buf, buf + n);
                        continue;
                    }
                    if (n == 0) {
                        running.store(false);
                        break;
                    }
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                        break;
                    if (errno == EINTR)
                        continue;
                    running.store(false);
                    break;
                }
            }

            const auto now = net_clock::now();
            if (now >= nextCommandTx) {
                nextCommandTx = now + kCommandCommPeriod;
                std::deque<std::vector<uint8_t>> frames;
                {
                    std::lock_guard<std::mutex> lock(ackMutex);
                    frames.swap(pendingCommandFrames);
                }
                while (!frames.empty()) {
                    if (!sendTelemetryFrameNonBlock(cfd, frames.front())) {
                        running.store(false);
                        break;
                    }
                    frames.pop_front();
                }
            }
        }
    });

    while (running.load()) {
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
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                        latestAxes[i] = axes[i];
                        if ((i > 0) && std::isfinite(latestAxes[i].encoder_position))
                            latestAxes[i].encoder_position *= kEncoderSign[i];
                    }
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

            if (planner.isRunning()) {
                std::array<double, 4> qJointRad{};
                if (planner.step(qJointRad)) {
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                        extVelControl.targetJointDeg[i] = qJointRad[i] * 180.0 / M_PI;
                        latestAxes[i].goal_position = extVelControl.targetJointDeg[i];
                    }
                    extVelControl.active = true;
                } else {
                    std::cerr << "[PMI] trajectory step returned false — stopping planner (check IK failure log above)\n";
                    planner.stop();
                }
            }

            if (initPoseMotion.active) {
                auto jointFeedbackDeg = [&](size_t i) -> double {
                    const bool useExternalEncoder = (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                    return useExternalEncoder ? latestAxes[i].encoder_position
                                              : (latestAxes[i].present_position * kGear[i]);
                };

                if (!initPoseMotion.waitingReach) {
                    const double elapsedSec =
                        std::chrono::duration_cast<std::chrono::duration<double>>(now - initPoseMotion.startTime).count();
                    const double r = elapsedSec / kInitPoseMoveSec;
                    const double s = quinticBlend(r);
                    const double progressPercent = std::min(100.0, std::max(0.0, r * 100.0));
                    std::array<double, 4> qJointRad{};
                    for (size_t i = 0; i < 4; ++i) {
                        qJointRad[i] =
                            initPoseMotion.qStartRad[i] + (initPoseMotion.qTargetRad[i] - initPoseMotion.qStartRad[i]) * s;
                        extVelControl.targetJointDeg[i] = qJointRad[i] * 180.0 / M_PI;
                        latestAxes[i].goal_position = extVelControl.targetJointDeg[i];
                    }
                    extVelControl.active = true;
                    constexpr auto kAckInterval = std::chrono::milliseconds(200);
                    if (lastInitProgressAck == clock::time_point{} || now - lastInitProgressAck >= kAckInterval) {
                        lastInitProgressAck = now;
                        const std::string msg = "INIT_POSE_PROGRESS:" + std::to_string(progressPercent);
                        enqueueCommandFrame(
                            pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    }

                    if (r >= 1.0) {
                        initPoseMotion.waitingReach = true;
                        initPoseMotion.reachWaitStart = now;
                        for (size_t i = 0; i < 4; ++i) {
                            extVelControl.targetJointDeg[i] = initPoseMotion.qTargetRad[i] * 180.0 / M_PI;
                            latestAxes[i].goal_position = extVelControl.targetJointDeg[i];
                        }
                        if (!haveLatestTelemetry) {
                            planner.setInitialJointRad(initPoseMotion.qTargetRad);
                            extVelControl.active = true;
                            const std::string msg = "INIT_POSE_DONE";
                            enqueueCommandFrame(
                                pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                            endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                            std::cerr << "[PMI] initial joint pose: quintic done (no telemetry — skipping reach check)\n";
                        }
                    }
                } else {
                    for (size_t i = 0; i < 4; ++i) {
                        extVelControl.targetJointDeg[i] = initPoseMotion.qTargetRad[i] * 180.0 / M_PI;
                        latestAxes[i].goal_position = extVelControl.targetJointDeg[i];
                    }
                    extVelControl.active = true;

                    bool withinTol = true;
                    for (size_t i = 0; i < 4; ++i) {
                        const double targetDeg = initPoseMotion.qTargetRad[i] * 180.0 / M_PI;
                        const double fbDeg = jointFeedbackDeg(i);
                        const double errDeg = targetDeg - fbDeg;
                        if (!std::isfinite(fbDeg) || !std::isfinite(errDeg)
                            || std::fabs(errDeg) > kInitPoseReachTolDeg[i])
                            withinTol = false;
                    }

                    const double waitSec =
                        std::chrono::duration_cast<std::chrono::duration<double>>(now - initPoseMotion.reachWaitStart).count();

                    constexpr auto kAckInterval = std::chrono::milliseconds(200);
                    if (lastInitProgressAck == clock::time_point{} || now - lastInitProgressAck >= kAckInterval) {
                        lastInitProgressAck = now;
                        const std::string msg = "INIT_POSE_PROGRESS:100.0";
                        enqueueCommandFrame(
                            pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    }

                    if (withinTol || waitSec >= kInitPoseReachMaxWaitSec) {
                        if (!withinTol)
                            std::cerr << "[PMI] init pose: reach tolerance not met within " << kInitPoseReachMaxWaitSec
                                      << " s — sending INIT_POSE_DONE anyway\n";
                        planner.setInitialJointRad(initPoseMotion.qTargetRad);
                        for (size_t i = 0; i < 4; ++i)
                            extVelControl.targetJointDeg[i] = initPoseMotion.qTargetRad[i] * 180.0 / M_PI;
                        extVelControl.active = true;
                        const std::string msg = "INIT_POSE_DONE";
                        enqueueCommandFrame(
                            pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                        endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                        std::cerr << "[PMI] initial joint pose completed ("
                                    << (withinTol ? "within reach tolerance" : "timeout") << ")\n";
                    }
                }
            }

            logger.updateLatest(latestAxes, haveLatestTelemetry, pathDesired, extVelControl.active, extVelControl.targetJointDeg);
            if (haveLatestTelemetry) {
                std::lock_guard<std::mutex> lock(m_latestMutex);
                for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
                    m_latestAxes[i] = latestAxes[i];
                m_haveLatestTelemetry = true;
            }

            if (m_dxl && m_dxl->isOpen() && jogState.active) {
                std::array<double, pmi::kTelemetryAxisCount> motorVelCmdDegPerSec{};
                motorVelCmdDegPerSec.fill(0.0);
                motorVelCmdDegPerSec[jogState.axis] = jogState.jointVelDegPerSec / kGear[jogState.axis];
                if (!m_dxl->writeGoalVelocityDegPerSec(motorVelCmdDegPerSec)) {
                    constexpr auto kFailLogInterval = std::chrono::seconds(1);
                    if (now - lastGoalWriteFailLog >= kFailLogInterval) {
                        lastGoalWriteFailLog = now;
                        std::cerr << "[PMI] write jog velocity failed\n";
                    }
                } else {
                    latestAxes[jogState.axis].goal_velocity = jogState.jointVelDegPerSec;
                }
            }

            if (m_dxl && m_dxl->isOpen() && haveLatestTelemetry && extVelControl.active && !jogState.active) {
                uint8_t op0 = (requestedControlOpMode != 0xFF) ? requestedControlOpMode
                                                                       : pmi::telemetryOpModeFromIdOp(latestAxes[0].id_op_mode);
                // Init / zero pose temporarily switches all motors to velocity mode; outer loop must match hardware.
                if (initPoseMotion.active)
                    op0 = 1;
                const bool useVelocityControl = (op0 == 1);
                const bool useCurrentControl = (op0 == 0); // XM540-W270 Operating Mode 0 (e-Manual)
                const bool usePositionControl = (op0 == 3 || op0 == 4 || op0 == 5);

                if (usePositionControl) {
                    std::array<double, pmi::kTelemetryAxisCount> motorPosCmdDeg{};
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
                        motorPosCmdDeg[i] = extVelControl.targetJointDeg[i] / kGear[i];

                    if (!m_dxl->writeGoalPositionDeg(motorPosCmdDeg)) {
                        constexpr auto kFailLogInterval = std::chrono::seconds(1);
                        if (now - lastGoalWriteFailLog >= kFailLogInterval) {
                            lastGoalWriteFailLog = now;
                            std::cerr << "[PMI] write goal position failed (mode-based control)\n";
                        }
                    }
                } else if (useVelocityControl) {
                    // Outer-loop PI: joint angle error [deg] -> commanded joint velocity [deg/s] -> motor velocity.
                    // Tuned for 1 ms loop + log review (pmi_server_log_*): lower gain / wider deadband → less limit-cycle hunting.
                    // Axis 0: motor-side angle only (no external encoder) → softer gains / wider deadband vs axes 1–3.
                    constexpr std::array<double, 4> kKp{{2.4, 4.8, 4.8, 4.5}}; // [deg/s per deg]
                    constexpr std::array<double, 4> kKi{{0.16, 0.39, 0.39, 0.32}}; // mult. integral state (see below)
                    constexpr std::array<double, 4> kMaxJointVelDegPerSec{{44.0, 50.0, 50.0, 47.0}};
                    constexpr std::array<double, 4> kIntegralLimitDegSec{{48.0, 48.0, 48.0, 48.0}};
                    constexpr std::array<double, 4> kHoldDeadbandDeg{{0.18, 0.10, 0.10, 0.10}};

                    std::array<double, pmi::kTelemetryAxisCount> motorVelCmdDegPerSec{};
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                        const bool useExternalEncoder = (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                        const double feedbackJointDeg =
                            useExternalEncoder ? latestAxes[i].encoder_position : (latestAxes[i].present_position * kGear[i]);
                        const double errDeg = extVelControl.targetJointDeg[i] - feedbackJointDeg;

                        if (std::fabs(errDeg) < kHoldDeadbandDeg[i]) {
                            extVelControl.integralErr[i] *= 0.95;
                        } else {
                            extVelControl.integralErr[i] += errDeg * kPlannerDt;
                            extVelControl.integralErr[i] = clampValue(
                                extVelControl.integralErr[i], -kIntegralLimitDegSec[i], kIntegralLimitDegSec[i]);
                        }

                        double jointVelCmd = kKp[i] * errDeg + kKi[i] * extVelControl.integralErr[i];
                        jointVelCmd = clampValue(jointVelCmd, -kMaxJointVelDegPerSec[i], kMaxJointVelDegPerSec[i]);
                        motorVelCmdDegPerSec[i] = jointVelCmd / kGear[i];
                        latestAxes[i].goal_velocity = jointVelCmd;
                    }

                    if (!m_dxl->writeGoalVelocityDegPerSec(motorVelCmdDegPerSec)) {
                        constexpr auto kFailLogInterval = std::chrono::seconds(1);
                        if (now - lastGoalWriteFailLog >= kFailLogInterval) {
                            lastGoalWriteFailLog = now;
                            std::cerr << "[PMI] write goal velocity failed (external encoder feedback)\n";
                        }
                    }
                } else if (useCurrentControl) {
                    // Current Control Mode: when a planned trajectory is running, use the VSD task-space
                    // PD + gravity-comp controller from analysis/cpp `run_vsd`. Otherwise (stationary
                    // hold or init pose hold) keep the joint-space PI fallback.
                    double vsdDesPos[3]{};
                    double vsdDesRoll = 0.0;
                    double vsdDesPitch = 0.0;
                    double vsdDesVel[5]{};
                    const bool useVsd = vsdEnable && planner.isRunning()
                        && planner.currentDesiredPoseAndVelocity(vsdDesPos, vsdDesRoll, vsdDesPitch, vsdDesVel);

                    // Track planner running edge so the gain ramp restarts on each new trajectory.
                    if (planner.isRunning() && !vsdPlannerWasRunning)
                        vsdRampStart = now;
                    vsdPlannerWasRunning = planner.isRunning();

                    if (useVsd) {
                        // 1) Joint state in [rad] / [rad/s] from telemetry.
                        Eigen::Vector4d q;
                        Eigen::Vector4d dq;
                        for (int i = 0; i < 4; ++i) {
                            const bool useExternalEncoder =
                                (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                            const double qDeg = useExternalEncoder
                                ? latestAxes[i].encoder_position
                                : (latestAxes[i].present_position * kGear[i]);
                            q(i) = qDeg * (M_PI / 180.0);
                            const double dqJointDegPerSec = latestAxes[i].present_velocity * kGear[i];
                            dq(i) = std::isfinite(dqJointDegPerSec) ? (dqJointDegPerSec * (M_PI / 180.0)) : 0.0;
                        }

                        // 2) FK & Jacobian.
                        Eigen::Vector3d re;
                        Eigen::Vector3d rpy;
                        pmi::fk_ee_pose_joint_rad(q, re, rpy);
                        Eigen::Matrix<double, 5, 4> J;
                        pmi::jacobian_5x4_joint_rad(q, J);

                        // 3) Workspace (5D) error and velocity error (run_vsd uses ω_x, ω_y for orientation rates;
                        //    here we use the analytic roll/pitch Jacobian rows × dq, which matches J^T mapping).
                        Eigen::Matrix<double, 5, 1> err;
                        err(0) = vsdDesPos[0] - re(0);
                        err(1) = vsdDesPos[1] - re(1);
                        err(2) = vsdDesPos[2] - re(2);
                        err(3) = wrapToPi(vsdDesRoll - rpy(0));
                        err(4) = wrapToPi(vsdDesPitch - rpy(1));

                        const Eigen::Matrix<double, 5, 1> curVel = J * dq;
                        Eigen::Matrix<double, 5, 1> errVel;
                        for (int k = 0; k < 5; ++k)
                            errVel(k) = vsdDesVel[k] - curVel(k);

                        // 4) Joint-space VSD torque + gravity comp (same form as ControlMain::run_vsd).
                        // Ramp Ks/Kd from 0 → nominal across vsdConfig.rampInSec to absorb any
                        // residual joint mismatch between planner-q[0] and actual encoder pose.
                        // gravity comp is NOT ramped — we want to hold against gravity from t=0.
                        double rampScale = 1.0;
                        if (vsdConfig.rampInSec > 0.0) {
                            const double elapsed =
                                std::chrono::duration_cast<std::chrono::duration<double>>(now - vsdRampStart).count();
                            rampScale = clampValue(elapsed / vsdConfig.rampInSec, 0.0, 1.0);
                        }
                        Eigen::Matrix<double, 5, 1> KePlusKv;
                        for (int k = 0; k < 5; ++k)
                            KePlusKv(k) = rampScale * (vsdConfig.Ks[k] * err(k) + vsdConfig.Kd[k] * errVel(k));
                        const Eigen::Vector4d tauVsd = J.transpose() * KePlusKv;
                        const Eigen::Vector4d tauG = pmi::joint_gravity_torque(q);

                        // 5) Joint torque [Nm] → motor current [A]: motor_tau = joint_tau * gear; I = motor_tau / Kt.
                        std::array<double, pmi::kTelemetryAxisCount> motorAmpCmd{};
                        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                            double tauJointNm = tauVsd(static_cast<int>(i)) + tauG(static_cast<int>(i));
                            tauJointNm = clampValue(tauJointNm, -vsdMaxJointTorqueNm, vsdMaxJointTorqueNm);
                            const double motorTauNm = tauJointNm * kGear[i];
                            double ampCmd = motorTauNm * vsdAmpPerNm;
                            ampCmd = clampValue(ampCmd, -vsdMaxAmp, vsdMaxAmp);
                            motorAmpCmd[i] = ampCmd;
                            latestAxes[i].goal_current = ampCmd;
                        }

                        if (!m_dxl->writeGoalCurrentAmp(motorAmpCmd)) {
                            constexpr auto kFailLogInterval = std::chrono::seconds(1);
                            if (now - lastGoalWriteFailLog >= kFailLogInterval) {
                                lastGoalWriteFailLog = now;
                                std::cerr << "[PMI] write goal current failed (VSD path tracking)\n";
                            }
                        }
                    } else {
                        // Joint-space PI -> Goal Current [A] at motor (addr 102, 2.69 mA/LSB) — used while holding,
                        // or when VSD is disabled via VSD_ENABLE=0.
                        constexpr std::array<double, 4> kKpCur{{0.018, 0.051, 0.051, 0.042}};
                        constexpr std::array<double, 4> kKiCur{{0.0014, 0.0042, 0.0042, 0.0034}};
                        constexpr std::array<double, 4> kMaxMotorAmp{{1.4, 1.8, 1.8, 1.4}};
                        constexpr std::array<double, 4> kIntegralLimitCur{{48.0, 48.0, 48.0, 48.0}};
                        constexpr std::array<double, 4> kHoldDeadbandDegCur{{0.18, 0.10, 0.10, 0.10}};

                        std::array<double, pmi::kTelemetryAxisCount> motorAmpCmd{};
                        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                            const bool useExternalEncoder =
                                (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                            const double feedbackJointDeg = useExternalEncoder
                                ? latestAxes[i].encoder_position
                                : (latestAxes[i].present_position * kGear[i]);
                            const double errDeg = extVelControl.targetJointDeg[i] - feedbackJointDeg;

                            if (std::fabs(errDeg) < kHoldDeadbandDegCur[i]) {
                                extVelControl.integralErr[i] *= 0.95;
                            } else {
                                extVelControl.integralErr[i] += errDeg * kPlannerDt;
                                extVelControl.integralErr[i] = clampValue(
                                    extVelControl.integralErr[i], -kIntegralLimitCur[i], kIntegralLimitCur[i]);
                            }

                            double ampCmd = kKpCur[i] * errDeg + kKiCur[i] * extVelControl.integralErr[i];
                            ampCmd = clampValue(ampCmd, -kMaxMotorAmp[i], kMaxMotorAmp[i]);
                            motorAmpCmd[i] = ampCmd;
                            latestAxes[i].goal_current = ampCmd;
                        }

                        if (!m_dxl->writeGoalCurrentAmp(motorAmpCmd)) {
                            constexpr auto kFailLogInterval = std::chrono::seconds(1);
                            if (now - lastGoalWriteFailLog >= kFailLogInterval) {
                                lastGoalWriteFailLog = now;
                                std::cerr << "[PMI] write goal current failed (current control mode)\n";
                            }
                        }
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(rxMutex);
            if (!rxShared.empty()) {
                rx.insert(rx.end(), rxShared.begin(), rxShared.end());
                rxShared.clear();
            }
        }
        if (!rx.empty()) {
            pmi::feedClientRxStream(
                rx, [this, &planner, &waypoints, &latestAxes, &kGear, &haveLatestTelemetry, &initPoseMotion, &extVelControl, &jogState,
                        &requestedControlOpMode, &logger, &enqueueCommandFrame, now, encSignForRx](
                        uint8_t cmd, const std::vector<uint8_t> &payload) {
                if (cmd == pmi::kCmdSetWaypointBatch) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    if (parseWaypointPayload(payload, waypoints)) {
                        planner.setWaypoints(waypoints);
                        std::cerr << "[PMI] received " << waypoints.size() << " waypoint(s)\n";
                    } else {
                        std::cerr << "[PMI] invalid waypoint payload\n";
                    }
                    return;
                }
                if (cmd == pmi::kCmdPlanPath) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());

                    // Re-anchor the planner's initial joint configuration to the *actual* current
                    // joint state right before IK pre-pass. The path's first sample then exits IK
                    // exactly at the current pose, instead of at whatever pose `setInitialJointPose`
                    // commanded earlier (which the robot rarely reaches within ±0.1°). Without this,
                    // the VSD sees a 5–10° "instant" joint error the moment the trajectory is
                    // started → task-space error of ~5–15 cm × Ks=15000 N/m → joint-torque clip →
                    // bang-bang current saturation.
                    if (m_dxl && m_dxl->isOpen())
                        (void)refreshLatestAxesFromDxl(m_dxl.get(), latestAxes, &haveLatestTelemetry, encSignForRx);
                    if (haveLatestTelemetry) {
                        std::array<double, 4> qCurrentRad{};
                        for (size_t i = 0; i < 4; ++i) {
                            const bool useExternalEncoder =
                                (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                            const double qCurrentJointDeg = useExternalEncoder
                                ? latestAxes[i].encoder_position
                                : (latestAxes[i].present_position * kGear[i]);
                            qCurrentRad[i] = qCurrentJointDeg * M_PI / 180.0;
                        }
                        planner.setInitialJointRad(qCurrentRad);
                        std::cerr << "[PMI] plan-path anchored at current joints (rad): ["
                                  << qCurrentRad[0] << ", " << qCurrentRad[1] << ", "
                                  << qCurrentRad[2] << ", " << qCurrentRad[3] << "]\n";
                    } else {
                        std::cerr << "[PMI] plan-path: no telemetry — using last setInitialJointRad value\n";
                    }

                    std::string ackMsg;
                    if (planner.plan(kPlannerDt)) {
                        std::cerr << "[PMI] trajectory planned: samples=" << planner.pathSampleCount() << "\n";
                        if (planner.lastPlanWasClipped()) {
                            ackMsg = "PLAN_OK_CLIPPED:" + std::to_string(planner.pathSampleCount())
                                + "@" + std::to_string(planner.lastPlanFirstClippedIndex());
                        } else {
                            ackMsg = "PLAN_OK:" + std::to_string(planner.pathSampleCount());
                        }
                    } else {
                        std::cerr << "[PMI] trajectory plan failed\n";
                        ackMsg = "PLAN_FAIL";
                    }
                    enqueueCommandFrame(
                        pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(ackMsg.begin(), ackMsg.end())));
                    return;
                }
                if (cmd == pmi::kCmdStartTrajectoryIk) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    planner.start();
                    extVelControl.active = true;
                    double ee0[3]{};
                    planner.currentEeFromInternalFk(ee0);
                    std::cerr << "[PMI] trajectory IK start: samples=" << planner.pathSampleCount()
                              << " running=" << (planner.isRunning() ? "yes" : "no") << " current_ee_xyz=(" << ee0[0] << ", "
                              << ee0[1] << ", " << ee0[2] << ") m\n";
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

                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    if (m_dxl && m_dxl->isOpen())
                        (void)refreshLatestAxesFromDxl(m_dxl.get(), latestAxes, &haveLatestTelemetry, encSignForRx);

                    std::array<double, 4> qCurrentRad{};
                    if (haveLatestTelemetry) {
                        for (size_t i = 0; i < 4; ++i) {
                            const bool useExternalEncoder = (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                            const double qCurrentJointDeg =
                                useExternalEncoder ? latestAxes[i].encoder_position : (latestAxes[i].present_position * kGear[i]);
                            qCurrentRad[i] = qCurrentJointDeg * M_PI / 180.0;
                        }
                    } else {
                        // Fallback when telemetry has not been received yet.
                        qCurrentRad = qJointRad;
                    }

                    for (size_t i = 0; i < 4; ++i)
                        initPoseMotion.savedMotorModes[i] = pmi::telemetryOpModeFromIdOp(latestAxes[i].id_op_mode);
                    if (!haveLatestTelemetry && requestedControlOpMode != 0xFF)
                        initPoseMotion.savedMotorModes.fill(static_cast<uint8_t>(requestedControlOpMode));
                    initPoseMotion.savedMotorModesValid = true;

                    switchAllAxesToVelocityOperatingMode(m_dxl.get());
                    zeroMotorGoalVelocityAndCurrent(m_dxl.get());

                    // Always override previous command/motion with the newest init-pose command.
                    planner.stop();
                    initPoseMotion.waitingReach = false;
                    extVelControl.active = false;
                    initPoseMotion.active = true;
                    initPoseMotion.startTime = now;
                    initPoseMotion.qStartRad = qCurrentRad;
                    initPoseMotion.qTargetRad = qJointRad;
                    extVelControl.active = true;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};

                    for (size_t i = 0; i < 4; ++i) {
                        const double qJointDeg = qJointRad[i] * 180.0 / M_PI;
                        latestAxes[i].goal_position = qJointDeg;
                        extVelControl.targetJointDeg[i] = qJointDeg;
                    }
                    std::cerr << "[PMI] initial joint pose motion started (5s quintic, motors temporary velocity mode)\n";
                    return;
                }
                if (cmd == pmi::kCmdJogVelocity) {
                    uint8_t axis = 0;
                    double jointVelDegPerSec = 0.0;
                    if (!parseJogVelocityPayload(payload, axis, jointVelDegPerSec)) {
                        std::cerr << "[PMI] invalid jog velocity payload\n";
                        return;
                    }

                    planner.stop();
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    extVelControl.active = false;

                    auto stopJogAndRestoreMode = [&]() { stopJogRestoreHardware(jogState, m_dxl.get()); };

                    if (std::fabs(jointVelDegPerSec) <= 1e-9) {
                        stopJogAndRestoreMode();
                        std::cerr << "[PMI] jog stop\n";
                        return;
                    }

                    if (jogState.active && jogState.axis != axis)
                        stopJogAndRestoreMode();

                    if (!jogState.active) {
                        if (m_dxl && m_dxl->isOpen())
                            (void)refreshLatestAxesFromDxl(m_dxl.get(), latestAxes, &haveLatestTelemetry, encSignForRx);
                        for (size_t i = 0; i < 4; ++i)
                            jogState.savedMotorModes[i] = pmi::telemetryOpModeFromIdOp(latestAxes[i].id_op_mode);
                        if (!haveLatestTelemetry && requestedControlOpMode != 0xFF)
                            jogState.savedMotorModes.fill(static_cast<uint8_t>(requestedControlOpMode));
                        jogState.savedMotorModesValid = true;
                        jogState.axis = axis;
                        switchAllAxesToVelocityOperatingMode(m_dxl.get());
                        zeroMotorGoalVelocityAndCurrent(m_dxl.get());
                    }

                    jogState.active = true;
                    jogState.jointVelDegPerSec = jointVelDegPerSec;
                    latestAxes[axis].goal_velocity = jointVelDegPerSec;
                    std::cerr << "[PMI] jog axis=" << static_cast<int>(axis) << " vel=" << jointVelDegPerSec
                              << " deg/s (all axes temporary velocity mode)\n";
                    return;
                }
                if (cmd == pmi::kCmdLogStart) {
                    double durationSec = 0.0;
                    if (!parseLogStartPayload(payload, durationSec)) {
                        std::cerr << "[PMI] invalid log start payload\n";
                        const std::string msg = "LOG_START_FAIL:invalid payload";
                        enqueueCommandFrame(pmi::buildServerAckFrame(
                            pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                        return;
                    }
                    if (!logger.start(durationSec)) {
                        std::cerr << "[PMI] failed to start logger\n";
                        const std::string msg = "LOG_START_FAIL:start failed";
                        enqueueCommandFrame(pmi::buildServerAckFrame(
                            pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    } else {
                        const std::string msg = std::string("LOG_START_OK:") + logger.currentLogPath();
                        enqueueCommandFrame(pmi::buildServerAckFrame(
                            pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    }
                    return;
                }
                if (cmd == pmi::kCmdLogStop) {
                    logger.stop();
                    const std::string msg = std::string("LOG_STOP_OK:") + logger.currentLogPath();
                    enqueueCommandFrame(
                        pmi::buildServerAckFrame(pmi::kSrvAck, std::vector<uint8_t>(msg.begin(), msg.end())));
                    return;
                }
                if (cmd == pmi::kCmdModeVelocity) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    requestedControlOpMode = 1;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};
                } else if (cmd == pmi::kCmdModeExtendedPos) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    requestedControlOpMode = 4;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};
                } else if (cmd == pmi::kCmdModeCurrentBasedPos) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    requestedControlOpMode = 5;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};
                } else if (cmd == pmi::kCmdModeCurrent) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    requestedControlOpMode = 0;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};
                } else if (cmd == pmi::kCmdStop) {
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                } else if (cmd == pmi::kCmdHoldStop) {
                    // Stop button: freeze the manipulator at the present joint state and discard any planned path.
                    // Torque stays ON (outer loop holds), waypoints & planned trajectory are cleared.
                    planner.stop();
                    planner.setWaypoints({});
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    if (haveLatestTelemetry) {
                        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                            const bool useExternalEncoder =
                                (i > 0) && std::isfinite(latestAxes[i].encoder_position);
                            const double qJointDeg = useExternalEncoder
                                ? latestAxes[i].encoder_position
                                : (latestAxes[i].present_position * kGear[i]);
                            extVelControl.targetJointDeg[i] = qJointDeg;
                            latestAxes[i].goal_position = qJointDeg;
                        }
                        // Sync planner internal joint state so a subsequent IK plan starts from the held pose.
                        std::array<double, 4> qHoldRad{};
                        for (size_t i = 0; i < 4; ++i)
                            qHoldRad[i] = extVelControl.targetJointDeg[i] * (M_PI / 180.0);
                        planner.setInitialJointRad(qHoldRad);
                    }
                    extVelControl.active = true;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};
                    if (m_dxl && m_dxl->isOpen())
                        zeroMotorGoalVelocityAndCurrent(m_dxl.get());
                    std::cerr << "[PMI] hold-stop: holding current joint pose, planned path cleared\n";
                    return; // Skip handlePmiClientCommand below — that path runs hardware-side servo logic.
                } else if (cmd == pmi::kCmdResetError) {
                    // Reboot recovery: cancel all in-flight motion before the bus stalls for ~800 ms.
                    planner.stop();
                    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
                    stopJogRestoreHardware(jogState, m_dxl.get());
                    extVelControl.active = false;
                    extVelControl.integralErr = {0.0, 0.0, 0.0, 0.0};
                }
                if (m_dxl)
                    m_dxl->handlePmiClientCommand(cmd);
            });
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

client_done:
    running.store(false);
    if (netThread.joinable())
        netThread.join();
    endInitPoseTemporaryVelocity(m_dxl.get(), initPoseMotion);
    stopJogRestoreHardware(jogState, m_dxl.get());
    std::cerr << "[PMI] client disconnected: " << peer << std::endl;
    {
        std::lock_guard<std::mutex> lock(m_clientMutex);
        if (m_activeCommandClientFd == cfd)
            m_activeCommandClientFd = -1;
    }
    ::shutdown(cfd, SHUT_RDWR);
    ::close(cfd);
}

void TcpServer::telemetrySession(int cfd)
{
    setNoDelay(cfd);
    (void)setNonBlock(cfd);

    const std::string peer = formatPeer(cfd);
    std::cerr << "[PMI] telemetry client connected: " << peer << std::endl;

    using clock = std::chrono::steady_clock;
    constexpr auto kTelemetryPeriod = std::chrono::milliseconds(100);
    auto nextTx = clock::now();

    while (!m_stop.load()) {
        const auto now = clock::now();
        if (now >= nextTx) {
            nextTx = now + kTelemetryPeriod;

            pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]{};
            bool haveLatest = false;
            {
                std::lock_guard<std::mutex> lock(m_latestMutex);
                haveLatest = m_haveLatestTelemetry;
                if (haveLatest) {
                    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
                        axes[i] = m_latestAxes[i];
                }
            }
            if (haveLatest) {
                const std::vector<uint8_t> frame = pmi::buildServerFrame(axes);
                if (!frame.empty() && !sendTelemetryFrameNonBlock(cfd, frame))
                    break;
            }
        }

        pollfd pfd{};
        pfd.fd = cfd;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, 5);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (pr > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
            break;

        // Port 9000 is telemetry-only. Drain and ignore unexpected inbound bytes.
        if (pr > 0 && (pfd.revents & POLLIN)) {
            char dropBuf[1024];
            while (true) {
                const ssize_t n = ::recv(cfd, dropBuf, sizeof(dropBuf), MSG_DONTWAIT);
                if (n > 0)
                    continue;
                if (n == 0)
                    goto telemetry_done;
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                if (errno == EINTR)
                    continue;
                goto telemetry_done;
            }
        }
    }

telemetry_done:
    std::cerr << "[PMI] telemetry client disconnected: " << peer << std::endl;
    {
        std::lock_guard<std::mutex> lock(m_clientMutex);
        if (m_activeTelemetryClientFd == cfd)
            m_activeTelemetryClientFd = -1;
    }
    ::shutdown(cfd, SHUT_RDWR);
    ::close(cfd);
}

TcpServer::TcpServer() = default;

TcpServer::~TcpServer()
{
    stop();
}

bool TcpServer::start(uint16_t telemetryPort, uint16_t commandPort)
{
    stop();

    const int telemetryFd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (telemetryFd < 0)
        return false;
    const int commandFd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (commandFd < 0) {
        ::close(telemetryFd);
        return false;
    }

    int reuse = 1;
    (void)::setsockopt(telemetryFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    (void)::setsockopt(commandFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in telemetryAddr{};
    telemetryAddr.sin_family = AF_INET;
    telemetryAddr.sin_port = htons(telemetryPort);
    telemetryAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr_in commandAddr{};
    commandAddr.sin_family = AF_INET;
    commandAddr.sin_port = htons(commandPort);
    commandAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(telemetryFd, reinterpret_cast<sockaddr *>(&telemetryAddr), sizeof(telemetryAddr)) != 0) {
        ::close(telemetryFd);
        ::close(commandFd);
        return false;
    }
    if (::bind(commandFd, reinterpret_cast<sockaddr *>(&commandAddr), sizeof(commandAddr)) != 0) {
        ::close(telemetryFd);
        ::close(commandFd);
        return false;
    }
    if (::listen(telemetryFd, 8) != 0 || ::listen(commandFd, 8) != 0) {
        ::close(telemetryFd);
        ::close(commandFd);
        return false;
    }

    m_telemetryListenFd = telemetryFd;
    m_commandListenFd = commandFd;
    m_stop.store(false);
    m_telemetryAcceptThread = std::thread([this]() { acceptLoop(m_telemetryListenFd, true); });
    m_commandAcceptThread = std::thread([this]() { acceptLoop(m_commandListenFd, false); });
    return true;
}

void TcpServer::stop()
{
    m_stop.store(true);
    int activeTelemetryFd = -1;
    int activeCommandFd = -1;
    {
        std::lock_guard<std::mutex> lock(m_clientMutex);
        activeTelemetryFd = m_activeTelemetryClientFd;
        activeCommandFd = m_activeCommandClientFd;
        m_activeTelemetryClientFd = -1;
        m_activeCommandClientFd = -1;
    }
    if (activeTelemetryFd >= 0) {
        ::shutdown(activeTelemetryFd, SHUT_RDWR);
        ::close(activeTelemetryFd);
    }
    if (activeCommandFd >= 0) {
        ::shutdown(activeCommandFd, SHUT_RDWR);
        ::close(activeCommandFd);
    }
    if (m_telemetryListenFd >= 0) {
        ::shutdown(m_telemetryListenFd, SHUT_RDWR);
        ::close(m_telemetryListenFd);
        m_telemetryListenFd = -1;
    }
    if (m_commandListenFd >= 0) {
        ::shutdown(m_commandListenFd, SHUT_RDWR);
        ::close(m_commandListenFd);
        m_commandListenFd = -1;
    }
    if (m_telemetryAcceptThread.joinable())
        m_telemetryAcceptThread.join();
    if (m_commandAcceptThread.joinable())
        m_commandAcceptThread.join();
}

void TcpServer::acceptLoop(int listenFd, bool telemetryOnly)
{
    while (!m_stop.load()) {
        const int cfd = ::accept(listenFd, nullptr, nullptr);
        if (cfd < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        int oldFd = -1;
        {
            std::lock_guard<std::mutex> lock(m_clientMutex);
            if (telemetryOnly) {
                oldFd = m_activeTelemetryClientFd;
                m_activeTelemetryClientFd = cfd;
            } else {
                oldFd = m_activeCommandClientFd;
                m_activeCommandClientFd = cfd;
            }
        }
        if (oldFd >= 0 && oldFd != cfd) {
            ::shutdown(oldFd, SHUT_RDWR);
            ::close(oldFd);
        }
        if (telemetryOnly)
            std::thread([this, cfd]() { telemetrySession(cfd); }).detach();
        else
            std::thread([this, cfd]() { commandSession(cfd); }).detach();
    }
}
