#ifndef SERVER_LOGGER_H
#define SERVER_LOGGER_H

#include "pmi_protocol.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

class ServerLogger {
public:
    struct PathDesiredPose {
        bool valid = false;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    ServerLogger();
    ~ServerLogger();

    void updateLatest(const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount], bool hasLatest, const PathDesiredPose &pathDesired);
    bool start(double durationSec);
    void stop();
    bool isLogging() const { return m_logging.load(); }
    std::string currentLogPath() const;

private:
    struct LogSample {
        std::uint64_t t_us = 0;
        std::array<pmi::ServoTelemetry, pmi::kTelemetryAxisCount> axes{};
        PathDesiredPose pathDesired{};
    };

    void samplerLoop(double durationSec);
    void writerLoop();
    void openFile();
    void closeFile();

    std::atomic<bool> m_shutdown{false};
    std::atomic<bool> m_logging{false};
    std::atomic<bool> m_samplerStop{false};
    std::thread m_samplerThread;
    std::thread m_writerThread;

    mutable std::mutex m_latestMutex;
    std::array<pmi::ServoTelemetry, pmi::kTelemetryAxisCount> m_latestAxes{};
    PathDesiredPose m_latestPathDesired{};
    bool m_hasLatest = false;

    std::mutex m_queueMutex;
    std::condition_variable m_queueCv;
    std::deque<LogSample> m_queue;

    mutable std::mutex m_fileMutex;
    std::ofstream m_file;
    std::string m_filePath;
};

#endif
