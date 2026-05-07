#include "server_logger.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear[4] = {32.0 / 60.0, 360.0 / 54.0, 360.0 / 108.0, 360.0 / 108.0};

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

struct EePose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

Mat3 matIdentity()
{
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 matMul(const Mat3 &a, const Mat3 &b)
{
    Mat3 c{};
    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
            c[r][col] = 0.0;
            for (int k = 0; k < 3; ++k)
                c[r][col] += a[r][k] * b[k][col];
        }
    }
    return c;
}

Vec3 matVecMul(const Mat3 &a, const Vec3 &v)
{
    Vec3 out{};
    for (int r = 0; r < 3; ++r)
        out[r] = a[r][0] * v[0] + a[r][1] * v[1] + a[r][2] * v[2];
    return out;
}

Vec3 vecAdd(const Vec3 &a, const Vec3 &b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Mat3 rotZ(double q)
{
    const double c = std::cos(q);
    const double s = std::sin(q);
    return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 eulerZXZ(double phi, double theta, double psi)
{
    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double cth = std::cos(theta);
    const double sth = std::sin(theta);
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);

    const Mat3 rotPhi = {{{cphi, -sphi, 0.0}, {sphi, cphi, 0.0}, {0.0, 0.0, 1.0}}};
    const Mat3 rotTheta = {{{1.0, 0.0, 0.0}, {0.0, cth, -sth}, {0.0, sth, cth}}};
    const Mat3 rotPsi = {{{cpsi, -spsi, 0.0}, {spsi, cpsi, 0.0}, {0.0, 0.0, 1.0}}};
    return matMul(matMul(rotPhi, rotTheta), rotPsi);
}

Vec3 mat2rpy(const Mat3 &a)
{
    const double roll = std::atan2(a[2][1], a[2][2]);
    const double pitch = std::atan2(-a[2][0], std::sqrt(a[0][0] * a[0][0] + a[1][0] * a[1][0]));
    const double yaw = std::atan2(a[1][0], a[0][0]);
    return {roll, pitch, yaw};
}

EePose eePoseFromMotorDeg(const std::array<pmi::ServoTelemetry, pmi::kTelemetryAxisCount> &axes, bool useGoal)
{
    std::array<double, 4> q{};
    for (int i = 0; i < 4; ++i) {
        const double motorDeg = useGoal ? axes[static_cast<size_t>(i)].goal_position : axes[static_cast<size_t>(i)].present_position;
        const double jointDeg = motorDeg * kGear[i];
        q[static_cast<size_t>(i)] = jointDeg * kPi / 180.0;
    }

    // Same kinematic constants as analysis/cpp ControlMain::read_data().
    const Vec3 sijp[4] = {{0.0, 0.0, -0.22}, {0.0, -0.23, 0.0}, {0.23, 0.0, 0.0}, {0.23, 0.0, 0.0}};
    const Mat3 Cij[4] = {
        eulerZXZ(0.0, kPi, 0.0),
        eulerZXZ(0.0, kPi / 2.0, 0.0),
        eulerZXZ(-kPi / 2.0, 0.0, 0.0),
        eulerZXZ(0.0, kPi, 0.0),
    };
    const Vec3 sep = {0.18, 0.0, 0.0};
    const Mat3 Ce = eulerZXZ(-kPi / 2.0, 0.0, 0.0);

    Mat3 prevAi = matIdentity();
    Vec3 prevRi = {0.0, 0.0, 0.0};
    Mat3 ai[4]{};
    Vec3 ri[4]{};
    for (int i = 0; i < 4; ++i) {
        const Mat3 aijpp = rotZ(q[static_cast<size_t>(i)]);
        ai[i] = matMul(matMul(prevAi, Cij[i]), aijpp);
        const Vec3 sij = matVecMul(prevAi, sijp[i]);
        ri[i] = vecAdd(prevRi, sij);
        prevAi = ai[i];
        prevRi = ri[i];
    }

    const Vec3 se = matVecMul(ai[3], sep);
    const Vec3 re = vecAdd(ri[3], se);
    const Mat3 ae = matMul(ai[3], Ce);
    const Vec3 rpy = mat2rpy(ae);

    EePose pose;
    pose.x = re[0];
    pose.y = re[1];
    pose.z = re[2];
    pose.roll = rpy[0];
    pose.pitch = rpy[1];
    pose.yaw = rpy[2];
    return pose;
}

std::string makeLogPath()
{
    namespace fs = std::filesystem;
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << "pmi_server_log_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
#ifdef PMI_SERVER_DATA_DIR
    return (fs::path(PMI_SERVER_DATA_DIR) / oss.str()).string();
#else
    return oss.str();
#endif
}
} // namespace

ServerLogger::ServerLogger()
{
    m_writerThread = std::thread([this]() { writerLoop(); });
}

ServerLogger::~ServerLogger()
{
    stop();
    m_shutdown.store(true);
    m_queueCv.notify_all();
    if (m_writerThread.joinable())
        m_writerThread.join();
    closeFile();
}

void ServerLogger::updateLatest(
    const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount], bool hasLatest, const PathDesiredPose &pathDesired)
{
    std::lock_guard<std::mutex> lock(m_latestMutex);
    m_hasLatest = hasLatest;
    m_latestPathDesired = pathDesired;
    if (!hasLatest)
        return;
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
        m_latestAxes[i] = axes[i];
}

bool ServerLogger::start(double durationSec)
{
    if (durationSec <= 0.0)
        return false;
    stop();
    openFile();
    if (!m_file.is_open())
        return false;

    m_samplerStop.store(false);
    m_logging.store(true);
    m_samplerThread = std::thread([this, durationSec]() { samplerLoop(durationSec); });
    std::cerr << "[PMI] logging start (" << durationSec << " s): " << m_filePath << "\n";
    return true;
}

void ServerLogger::stop()
{
    m_samplerStop.store(true);
    m_logging.store(false);
    if (m_samplerThread.joinable())
        m_samplerThread.join();
}

std::string ServerLogger::currentLogPath() const
{
    std::lock_guard<std::mutex> lock(m_fileMutex);
    return m_filePath;
}

void ServerLogger::samplerLoop(double durationSec)
{
    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    auto nextTick = start;
    while (!m_samplerStop.load()) {
        const auto now = clock::now();
        if (std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count() >= durationSec)
            break;

        LogSample sample;
        bool have = false;
        {
            std::lock_guard<std::mutex> lock(m_latestMutex);
            have = m_hasLatest;
            if (have) {
                sample.t_us = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());
                sample.axes = m_latestAxes;
                sample.pathDesired = m_latestPathDesired;
            }
        }
        if (have) {
            {
                std::lock_guard<std::mutex> lock(m_queueMutex);
                m_queue.push_back(std::move(sample));
            }
            m_queueCv.notify_one();
        }
        nextTick += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(nextTick);
    }
    m_logging.store(false);
    m_queueCv.notify_one();
    std::cerr << "[PMI] logging stopped\n";
}

void ServerLogger::writerLoop()
{
    while (!m_shutdown.load()) {
        std::deque<LogSample> batch;
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCv.wait(lock, [this]() { return m_shutdown.load() || !m_queue.empty(); });
            if (m_shutdown.load() && m_queue.empty())
                break;
            batch.swap(m_queue);
        }
        if (batch.empty())
            continue;

        std::lock_guard<std::mutex> lock(m_fileMutex);
        if (!m_file.is_open())
            continue;
        for (const auto &s : batch) {
            const EePose eeCur = eePoseFromMotorDeg(s.axes, false);
            const EePose eeDes = eePoseFromMotorDeg(s.axes, true);
            m_file << s.t_us;
            m_file << "," << eeCur.x << "," << eeCur.y << "," << eeCur.z
                   << "," << eeCur.roll << "," << eeCur.pitch << "," << eeCur.yaw;
            m_file << "," << eeDes.x << "," << eeDes.y << "," << eeDes.z
                   << "," << eeDes.roll << "," << eeDes.pitch << "," << eeDes.yaw;
            m_file << "," << eeDes.x << "," << eeDes.y << "," << eeDes.z
                   << "," << eeDes.roll << "," << eeDes.pitch << "," << eeDes.yaw;
            if (s.pathDesired.valid) {
                m_file << "," << s.pathDesired.x << "," << s.pathDesired.y << "," << s.pathDesired.z
                       << "," << s.pathDesired.roll << "," << s.pathDesired.pitch << "," << s.pathDesired.yaw;
            } else {
                m_file << ",nan,nan,nan,nan,nan,nan";
            }
            for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
                const auto &a = s.axes[i];
                m_file << "," << static_cast<int>(a.id_op_mode)
                       << "," << static_cast<int>(a.servo_state)
                       << "," << a.present_position
                       << "," << a.encoder_position
                       << "," << a.present_velocity
                       << "," << a.present_current
                       << "," << a.goal_position
                       << "," << a.goal_velocity
                       << "," << a.goal_current
                       << "," << static_cast<int>(a.error_state);
            }
            m_file << "\n";
        }
        m_file.flush();
    }
}

void ServerLogger::openFile()
{
    std::lock_guard<std::mutex> lock(m_fileMutex);
    closeFile();
    m_filePath = makeLogPath();
    m_file.open(m_filePath, std::ios::out | std::ios::trunc);
    if (!m_file.is_open())
        return;
    m_file << "t_us";
    m_file << ",ee_cur_x,ee_cur_y,ee_cur_z,ee_cur_roll,ee_cur_pitch,ee_cur_yaw";
    m_file << ",ee_des_x,ee_des_y,ee_des_z,ee_des_roll,ee_des_pitch,ee_des_yaw";
    m_file << ",ee_des_goal_x,ee_des_goal_y,ee_des_goal_z,ee_des_goal_roll,ee_des_goal_pitch,ee_des_goal_yaw";
    m_file << ",ee_des_path_x,ee_des_path_y,ee_des_path_z,ee_des_path_roll,ee_des_path_pitch,ee_des_path_yaw";
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        m_file << ",ax" << i << "_id_op"
               << ",ax" << i << "_servo_state"
               << ",ax" << i << "_present_pos"
               << ",ax" << i << "_encoder_pos"
               << ",ax" << i << "_present_vel"
               << ",ax" << i << "_present_cur"
               << ",ax" << i << "_goal_pos"
               << ",ax" << i << "_goal_vel"
               << ",ax" << i << "_goal_cur"
               << ",ax" << i << "_err";
    }
    m_file << "\n";
}

void ServerLogger::closeFile()
{
    if (m_file.is_open())
        m_file.close();
}
