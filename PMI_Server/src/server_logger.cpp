#include "server_logger.h"

#include <pmi_kinematics/pmi_kinematics.hpp>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace {

struct EePose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

/// Compute EE pose using the same joint source as the VSD/PI controllers in `tcp_server.cpp`:
///   - axis 0   : motor `present_position` × gear ratio (no external encoder available)
///   - axis 1-3 : external AMT21 encoder when finite, otherwise fall back to motor × gear.
/// Mixing motor*gear with encoder readings here used to make `ee_cur` look like it was at the
/// origin (off by ~38° on axis 2 due to gear/zero-offset calibration drift), which made VSD logs
/// completely unusable for tuning.
EePose eePoseFromTelemetry(const std::array<pmi::ServoTelemetry, pmi::kTelemetryAxisCount> &axes, bool useGoal)
{
    double motor_deg[4]{};
    for (int i = 0; i < 4; ++i)
        motor_deg[i] = useGoal ? axes[static_cast<size_t>(i)].goal_position
                                : axes[static_cast<size_t>(i)].present_position;
    double q_rad[4]{};
    pmi::joint_rad_from_motor_deg(motor_deg, q_rad);

    if (!useGoal) {
        for (int i = 1; i < 4; ++i) {
            const double enc_deg = axes[static_cast<size_t>(i)].encoder_position;
            if (std::isfinite(enc_deg))
                q_rad[i] = enc_deg * (M_PI / 180.0);
        }
    }

    const Eigen::Vector4d q(q_rad[0], q_rad[1], q_rad[2], q_rad[3]);
    Eigen::Vector3d pos;
    Eigen::Vector3d rpy;
    pmi::fk_ee_pose_joint_rad(q, pos, rpy);

    EePose pose;
    pose.x = pos[0];
    pose.y = pos[1];
    pose.z = pos[2];
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
    const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount], bool hasLatest, const PathDesiredPose &pathDesired,
    bool targetJointDegValid, const std::array<double, pmi::kTelemetryAxisCount> &targetJointDeg)
{
    std::lock_guard<std::mutex> lock(m_latestMutex);
    m_hasLatest = hasLatest;
    m_latestPathDesired = pathDesired;
    m_latestTargetJointDegValid = targetJointDegValid;
    m_latestTargetJointDeg = targetJointDeg;
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
                sample.targetJointDegValid = m_latestTargetJointDegValid;
                sample.targetJointDeg = m_latestTargetJointDeg;
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
            const EePose eeCur = eePoseFromTelemetry(s.axes, false);
            const EePose eeDes = eePoseFromTelemetry(s.axes, true);
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
            constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
            if (s.targetJointDegValid) {
                for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
                    m_file << "," << s.targetJointDeg[i];
            } else {
                for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
                    m_file << "," << kNan;
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
    m_file << ",target_joint_deg_0,target_joint_deg_1,target_joint_deg_2,target_joint_deg_3";
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
