// DYNAMIXEL SDK: PortHandler, PacketHandler, readTxRx / read1ByteTxRx for telemetry; GroupSyncWrite for commands.

#include "dxl_protocol2.h"

#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdlib>
#include <fstream>

namespace {

constexpr uint8_t kMotorIds[pmi::kTelemetryAxisCount] = {1, 2, 3, 4};
constexpr uint8_t kAmt21AddressForMotor2 = 0x84;
constexpr uint8_t kAmt21AddressForMotor3 = 0x74;
constexpr uint8_t kAmt21AddressForMotor4 = 0x64;
constexpr int kAmt21ResolutionBits = 14;
constexpr int kAmt21BaudRate = 115200;
constexpr const char *kAmt21DevicePath = "/dev/ttyU2D2";

constexpr uint16_t kAddrOperatingMode = 11;
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrHardwareError = 70;
constexpr uint16_t kAddrGoalCurrent = 102;
constexpr uint16_t kAddrPresentCurrent = 126;

// XM540-W270 (Protocol 2.0) raw → SI-style for client: deg, deg/s, A. See e-Manual control table.
constexpr double kPulseToDeg = 360.0 / 4096.0; // Present / Goal Position resolution
constexpr double kVelRawToRpm = 0.229;       // Present / Goal Velocity
constexpr double kRpmToDegPerSec = 6.0;      // rpm → deg/s (×360/60)
constexpr double kCurRawToMa = 2.69;         // Present / Goal Current (when used as current)
constexpr double kMaToA = 0.001;

int32_t readI32Le(const uint8_t *p)
{
    uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16)
        | (static_cast<uint32_t>(p[3]) << 24);
    int32_t v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

int16_t readI16Le(const uint8_t *p)
{
    uint16_t u = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    int16_t v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

speed_t baudrateToTermios(int baudrate)
{
    switch (baudrate) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
#ifdef B230400
    case 230400:
        return B230400;
#endif
#ifdef B460800
    case 460800:
        return B460800;
#endif
#ifdef B921600
    case 921600:
        return B921600;
#endif
#ifdef B1000000
    case 1000000:
        return B1000000;
#endif
#ifdef B2000000
    case 2000000:
        return B2000000;
#endif
    default:
        return B115200;
    }
}

bool amt21ChecksumOk(uint16_t raw)
{
    const uint8_t k1 = (raw >> 15) & 0x01;
    const uint8_t k0 = (raw >> 14) & 0x01;

    const uint8_t calcK1 = !(((raw >> 13) & 0x01) ^ ((raw >> 11) & 0x01) ^ ((raw >> 9) & 0x01) ^ ((raw >> 7) & 0x01)
        ^ ((raw >> 5) & 0x01) ^ ((raw >> 3) & 0x01) ^ ((raw >> 1) & 0x01));
    const uint8_t calcK0 = !(((raw >> 12) & 0x01) ^ ((raw >> 10) & 0x01) ^ ((raw >> 8) & 0x01) ^ ((raw >> 6) & 0x01)
        ^ ((raw >> 4) & 0x01) ^ ((raw >> 2) & 0x01) ^ ((raw >> 0) & 0x01));

    return (k1 == calcK1) && (k0 == calcK0);
}

int readExactWithTimeout(int fd, uint8_t *buffer, int length, int timeoutMs)
{
    int total = 0;
    const auto start = std::chrono::steady_clock::now();

    while (total < length) {
        const int n = static_cast<int>(::read(fd, buffer + total, static_cast<size_t>(length - total)));
        if (n > 0)
            total += n;
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            break;

        const auto now = std::chrono::steady_clock::now();
        const int elapsedMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
        if (elapsedMs >= timeoutMs)
            break;
    }

    return total;
}

} // namespace

DxlBus::~DxlBus()
{
    close();
}

bool DxlBus::isOpen() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return port_ != nullptr;
}

void DxlBus::closeUnlocked()
{
    if (port_) {
        port_->closePort();
        delete port_;
        port_ = nullptr;
    }
    if (packet_) {
        delete packet_;
        packet_ = nullptr;
    }
    if (amt21_fd_ >= 0) {
        ::close(amt21_fd_);
        amt21_fd_ = -1;
    }
}

bool DxlBus::open(const char *devicePath, int baudRate)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeUnlocked();
    }

    port_ = dynamixel::PortHandler::getPortHandler(devicePath);
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_ || !packet_) {
        closeUnlocked();
        return false;
    }
    if (!port_->openPort()) {
        closeUnlocked();
        return false;
    }
    if (!port_->setBaudRate(baudRate)) {
        closeUnlocked();
        return false;
    }
    (void)openAmt21PortUnlocked(kAmt21DevicePath, kAmt21BaudRate);
    loadZeroOffsetFromFileUnlocked();
    return true;
}

bool DxlBus::openAmt21PortUnlocked(const std::string &devicePath, int baudRate)
{
    if (amt21_fd_ >= 0) {
        ::close(amt21_fd_);
        amt21_fd_ = -1;
    }

    const int fd = ::open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
        return false;

    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return false;
    }

    const speed_t speed = baudrateToTermios(baudRate);
    ::cfsetospeed(&tty, speed);
    ::cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }
    ::tcflush(fd, TCIOFLUSH);

    amt21_fd_ = fd;
    return true;
}

bool DxlBus::readAmt21AngleDegUnlocked(uint8_t nodeAddress, double &angleDegOut)
{
    if (amt21_fd_ < 0)
        return false;

    ::tcflush(amt21_fd_, TCIOFLUSH);
    if (::write(amt21_fd_, &nodeAddress, 1) != 1)
        return false;
    ::tcdrain(amt21_fd_);

    uint8_t rx[2]{0, 0};
    if (readExactWithTimeout(amt21_fd_, rx, 2, 50) != 2)
        return false;

    const uint16_t raw = (static_cast<uint16_t>(rx[1]) << 8) | rx[0];
    if (!amt21ChecksumOk(raw))
        return false;

    uint16_t position = raw & 0x3FFF;
    if (kAmt21ResolutionBits == 12)
        position >>= 2;
    const double maxCount = (kAmt21ResolutionBits == 14) ? 16384.0 : 4096.0;
    angleDegOut = static_cast<double>(position) / maxCount * 360.0;
    return true;
}

void DxlBus::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    closeUnlocked();
}

bool DxlBus::readMotorTelemetryUnlocked(uint8_t id, pmi::ServoTelemetry &out)
{
    uint8_t dxl_error = 0;
    uint8_t b1 = 0;
    if (packet_->read1ByteTxRx(port_, id, kAddrOperatingMode, &b1, &dxl_error) != COMM_SUCCESS)
        return false;
    const uint8_t op = b1;

    uint8_t tq = 0;
    if (packet_->read1ByteTxRx(port_, id, kAddrTorqueEnable, &tq, &dxl_error) != COMM_SUCCESS)
        return false;

    uint8_t hw = 0;
    if (packet_->read1ByteTxRx(port_, id, kAddrHardwareError, &hw, &dxl_error) != COMM_SUCCESS)
        return false;

    uint8_t goalBlock[18]{};
    if (packet_->readTxRx(port_, id, kAddrGoalCurrent, 18, goalBlock, &dxl_error) != COMM_SUCCESS)
        return false;
    const int16_t goalCurRaw = readI16Le(goalBlock);
    const int32_t goalVelRaw = readI32Le(goalBlock + 2);
    const int32_t goalPosRaw = readI32Le(goalBlock + 14);

    uint8_t presBlock[10]{};
    if (packet_->readTxRx(port_, id, kAddrPresentCurrent, 10, presBlock, &dxl_error) != COMM_SUCCESS)
        return false;
    const int16_t presCurRaw = readI16Le(presBlock);
    const int32_t presVelRaw = readI32Le(presBlock + 2);
    const int32_t presPosRaw = readI32Le(presBlock + 6);

    out.id_op_mode = pmi::packTelemetryIdOp(id, op);
    out.servo_state = tq ? 1 : 0;
    out.present_position = static_cast<double>(presPosRaw) * kPulseToDeg;
    out.encoder_position = std::nan("");
    out.present_velocity = static_cast<double>(presVelRaw) * kVelRawToRpm * kRpmToDegPerSec;
    out.present_current = static_cast<double>(presCurRaw) * kCurRawToMa * kMaToA;
    out.goal_position = static_cast<double>(goalPosRaw) * kPulseToDeg;
    out.goal_velocity = static_cast<double>(goalVelRaw) * kVelRawToRpm * kRpmToDegPerSec;
    out.goal_current = static_cast<double>(goalCurRaw) * kCurRawToMa * kMaToA;
    out.error_state = hw;
    return true;
}

bool DxlBus::syncReadTelemetry(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_)
        return false;

    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        if (!readMotorTelemetryUnlocked(kMotorIds[i], axes[i]))
            return false;
    }

    struct AmtMap {
        uint8_t motorId;
        uint8_t nodeAddress;
        size_t axisIndex;
    };
    constexpr AmtMap kAmtMap[3] = {
        {2, kAmt21AddressForMotor2, 1},
        {3, kAmt21AddressForMotor3, 2},
        {4, kAmt21AddressForMotor4, 3},
    };

    constexpr auto kFailLogInterval = std::chrono::seconds(1);
    const auto now = std::chrono::steady_clock::now();

    for (size_t i = 0; i < 3; ++i) {
        const auto &m = kAmtMap[i];
        double amtAngleDeg = 0.0;
        const bool ok = readAmt21AngleDegUnlocked(m.nodeAddress, amtAngleDeg);
        if (ok) {
            axes[m.axisIndex].encoder_position = amtAngleDeg;
            if (amt21_in_fail_state_[i]) {
                std::cerr << "[AMT21] recovered: DXL ID " << static_cast<int>(m.motorId) << " addr=0x" << std::hex
                          << static_cast<int>(m.nodeAddress) << std::dec << " after " << amt21_fail_count_[i]
                          << " failed read(s)\n";
                amt21_in_fail_state_[i] = false;
                amt21_fail_count_[i] = 0;
                amt21_last_fail_log_[i] = {};
            }
            continue;
        }

        ++amt21_fail_count_[i];
        if (!amt21_in_fail_state_[i] || (now - amt21_last_fail_log_[i]) >= kFailLogInterval) {
            std::cerr << "[AMT21] read failed: DXL ID " << static_cast<int>(m.motorId) << " addr=0x" << std::hex
                      << static_cast<int>(m.nodeAddress) << std::dec << " -> encoder_position unavailable (fail count="
                      << amt21_fail_count_[i] << ")\n";
            amt21_last_fail_log_[i] = now;
        }
        amt21_in_fail_state_[i] = true;
    }

    applyZeroOffsetUnlocked(axes);
    return true;
}

bool DxlBus::captureZeroOffsetUnlocked()
{
    pmi::ServoTelemetry snapshot[pmi::kTelemetryAxisCount]{};
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        if (!readMotorTelemetryUnlocked(kMotorIds[i], snapshot[i]))
            return false;
    }

    struct AmtMap {
        uint8_t nodeAddress;
        size_t axisIndex;
    };
    constexpr AmtMap kAmtMap[3] = {
        {kAmt21AddressForMotor2, 1},
        {kAmt21AddressForMotor3, 2},
        {kAmt21AddressForMotor4, 3},
    };
    for (const auto &m : kAmtMap) {
        double amtAngleDeg = 0.0;
        if (readAmt21AngleDegUnlocked(m.nodeAddress, amtAngleDeg))
            snapshot[m.axisIndex].encoder_position = amtAngleDeg;
    }

    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        motor_offset_deg_[i] = snapshot[i].present_position;
        goal_offset_deg_[i] = snapshot[i].goal_position;
        encoder_offset_deg_[i] = snapshot[i].encoder_position;
    }
    zero_offset_valid_ = true;
    if (!saveZeroOffsetToFileUnlocked())
        std::cerr << "[PMI] warning: failed to persist zero offsets to file\n";
    return true;
}

void DxlBus::applyZeroOffsetUnlocked(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]) const
{
    if (!zero_offset_valid_)
        return;

    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        axes[i].present_position -= motor_offset_deg_[i];
        axes[i].goal_position -= goal_offset_deg_[i];
        if (std::isfinite(axes[i].encoder_position) && std::isfinite(encoder_offset_deg_[i]))
            axes[i].encoder_position -= encoder_offset_deg_[i];
    }
}

void DxlBus::handlePmiClientCommand(uint8_t cmd)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_)
        return;

    const auto torqueAll = [&](uint8_t v) {
        std::array<uint8_t, pmi::kTelemetryAxisCount> bytes{};
        bytes.fill(v);
        dynamixel::GroupSyncWrite gsw(port_, packet_, kAddrTorqueEnable, 1);
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            (void)gsw.addParam(kMotorIds[i], &bytes[i]);
        }
        (void)gsw.txPacket();
    };
    const auto modeAll = [&](uint8_t mode) {
        std::array<uint8_t, pmi::kTelemetryAxisCount> bytes{};
        bytes.fill(mode);
        dynamixel::GroupSyncWrite gsw(port_, packet_, kAddrOperatingMode, 1);
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            (void)gsw.addParam(kMotorIds[i], &bytes[i]);
        }
        (void)gsw.txPacket();
    };

    switch (cmd) {
    case pmi::kCmdPing:
        break;
    case pmi::kCmdServoOn:
        torqueAll(1);
        break;
    case pmi::kCmdStop:
        torqueAll(0);
        break;
    case pmi::kCmdSetZero:
        (void)captureZeroOffsetUnlocked();
        break;
    case pmi::kCmdModeCurrent:
        torqueAll(0);
        modeAll(0);
        break;
    case pmi::kCmdModeVelocity:
        torqueAll(0);
        modeAll(1);
        break;
    case pmi::kCmdModeExtendedPos:
        torqueAll(0);
        modeAll(4);
        break;
    default:
        break;
    }
}

std::string DxlBus::zeroOffsetFilePath() const
{
    if (!zero_offset_path_.empty())
        return zero_offset_path_;
#ifdef PMI_SERVER_DATA_DIR
    return std::string(PMI_SERVER_DATA_DIR) + "/pmi_zero_offsets.txt";
#else
    return "pmi_zero_offsets.txt";
#endif
}

void DxlBus::loadZeroOffsetFromFileUnlocked()
{
    zero_offset_path_ = zeroOffsetFilePath();
    std::ifstream ifs(zero_offset_path_);
    if (!ifs.is_open()) {
        std::cerr << "[PMI] no zero-offset file at " << zero_offset_path_ << " (using identity offsets)\n";
        return;
    }

    const auto parseDouble = [](const std::string &tok, double &out) -> bool {
        if (tok.empty())
            return false;
        try {
            size_t pos = 0;
            out = std::stod(tok, &pos);
            return pos == tok.size();
        } catch (...) {
            return false;
        }
    };

    std::array<double, pmi::kTelemetryAxisCount> motor{};
    std::array<double, pmi::kTelemetryAxisCount> goal{};
    std::array<double, pmi::kTelemetryAxisCount> enc{};
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        std::string m, g, e;
        if (!(ifs >> m >> g >> e)) {
            std::cerr << "[PMI] zero-offset file truncated at axis " << i << " (ignored)\n";
            return;
        }
        if (!parseDouble(m, motor[i]) || !parseDouble(g, goal[i]) || !parseDouble(e, enc[i])) {
            std::cerr << "[PMI] zero-offset parse failed at axis " << i << " (\"" << m << "\" \"" << g << "\" \"" << e
                      << "\") — file ignored\n";
            return;
        }
    }

    motor_offset_deg_ = motor;
    goal_offset_deg_ = goal;
    encoder_offset_deg_ = enc;
    zero_offset_valid_ = true;
    std::cerr << "[PMI] loaded zero offsets from " << zero_offset_path_ << ":\n";
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        std::cerr << "  axis " << i << ": motor=" << motor_offset_deg_[i] << " goal=" << goal_offset_deg_[i]
                  << " enc=" << encoder_offset_deg_[i] << "\n";
    }
}

bool DxlBus::saveZeroOffsetToFileUnlocked() const
{
    if (!zero_offset_valid_)
        return false;

    std::ofstream ofs(zeroOffsetFilePath(), std::ios::trunc);
    if (!ofs.is_open())
        return false;

    ofs.setf(std::ios::fixed);
    ofs.precision(10);
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
        ofs << motor_offset_deg_[i] << ' ' << goal_offset_deg_[i] << ' ' << encoder_offset_deg_[i] << '\n';
    return static_cast<bool>(ofs);
}
