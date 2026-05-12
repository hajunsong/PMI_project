// DYNAMIXEL SDK: PortHandler, PacketHandler.
//   Telemetry: persistent GroupSyncRead of a 23-byte Indirect Data block (1 broadcast TxRx for all
//              4 motors per loop). Requires U2D2 / /dev/ttyUSB*. USB-CDC bridges (/dev/ttyACM*) bunch
//              multiple status packets into a single USB IN transfer causing -3002 framing errors.
//   Goals:     persistent GroupSyncWrite per goal-type, routed through the write-side Indirect Data
//              block. Writes have no per-ID status response, so broadcast is safe here.
//   Reboot resets the indirect-address mappings, so they are re-applied on connect & after Reset.
// Operating Mode (addr 11) lives in EEPROM and cannot be Indirect-mapped, so it is cached
// locally and updated on every mode-set path.

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
#include <thread>
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

constexpr uint16_t kAddrReturnDelayTime = 9;   // EEPROM — unit: 2 µs; 0 = minimum latency
constexpr uint16_t kAddrOperatingMode = 11;   // EEPROM — cached, NOT in indirect block
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrHardwareError = 70;
constexpr uint16_t kAddrGoalCurrent = 102;
constexpr uint16_t kAddrGoalVelocity = 104;
constexpr uint16_t kAddrGoalPosition = 116;
constexpr uint16_t kAddrMoving = 122;
constexpr uint16_t kAddrPresentCurrent = 126;
constexpr uint16_t kAddrPresentVelocity = 128;
constexpr uint16_t kAddrPresentPosition = 132;
constexpr int16_t kCurrentBasedPosGoalCurrentRaw = 350; // ~0.94A (350 * 2.69mA)

// X-series Indirect Addressing (e-Manual: Indirect Address 1 = addr 168, Indirect Data 1 = addr 224;
// Indirect Address 29 = addr 578, Indirect Data 29 = addr 634). Source addresses must be in RAM (≥64).
constexpr uint16_t kAddrIndirectAddrRead = 168;   // Indirect Address 1 .. 22 (we use 22)
constexpr uint16_t kAddrIndirectDataRead = 224;   // Indirect Data 1 .. 22
constexpr uint16_t kAddrIndirectAddrWrite = 578;  // Indirect Address 29 .. 38 (10 bytes for goals)
constexpr uint16_t kAddrIndirectDataWrite = 634;  // Indirect Data 29 .. 38

// Read block layout (23 bytes, contiguous in indirect data) - aligned with dxl_test:
//   offset  0  : Torque Enable        (src 64,  1 B)
//   offset  1  : Present Position     (src 132, 4 B)
//   offset  5  : Present Velocity     (src 128, 4 B)
//   offset  9  : Present Current      (src 126, 2 B)
//   offset 11  : Hardware Error       (src 70,  1 B)
//   offset 12  : Goal Position        (src 116, 4 B)
//   offset 16  : Goal Velocity        (src 104, 4 B)
//   offset 20  : Goal Current         (src 102, 2 B)
//   offset 22  : Moving               (src 122, 1 B)
constexpr uint16_t kReadBlockLength = 23;
constexpr uint16_t kReadOffTorque = 0;
constexpr uint16_t kReadOffPresPos = 1;
constexpr uint16_t kReadOffPresVel = 5;
constexpr uint16_t kReadOffPresCur = 9;
constexpr uint16_t kReadOffHwErr = 11;
constexpr uint16_t kReadOffGoalPos = 12;
constexpr uint16_t kReadOffGoalVel = 16;
constexpr uint16_t kReadOffGoalCur = 20;

// Write block layout (10 bytes): Goal Current (2 B) | Goal Velocity (4 B) | Goal Position (4 B).
constexpr uint16_t kWriteOffGoalCur = 0;
constexpr uint16_t kWriteOffGoalVel = 2;
constexpr uint16_t kWriteOffGoalPos = 6;

// XM540-W270 (Protocol 2.0) raw → SI-style for client: deg, deg/s, A. See e-Manual control table.
constexpr double kPulseToDeg = 360.0 / 4096.0; // Present / Goal Position resolution
constexpr double kVelRawToRpm = 0.229;       // Present / Goal Velocity
constexpr double kRpmToDegPerSec = 6.0;      // rpm → deg/s (×360/60)
constexpr double kCurRawToMa = 2.69;         // Present / Goal Current (when used as current)
constexpr double kMaToA = 0.001;

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

void DxlBus::destroySyncGroupsUnlocked()
{
    if (sync_read_telemetry_) {
        delete sync_read_telemetry_;
        sync_read_telemetry_ = nullptr;
    }
    if (sync_write_goal_current_) {
        delete sync_write_goal_current_;
        sync_write_goal_current_ = nullptr;
    }
    if (sync_write_goal_velocity_) {
        delete sync_write_goal_velocity_;
        sync_write_goal_velocity_ = nullptr;
    }
    if (sync_write_goal_position_) {
        delete sync_write_goal_position_;
        sync_write_goal_position_ = nullptr;
    }
}

void DxlBus::closeUnlocked()
{
    destroySyncGroupsUnlocked();
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
    indirect_setup_ok_ = false;
}

bool DxlBus::open(const char *devicePath, int baudRate, const char *amt21DevicePath, int amt21BaudRate)
{
    std::lock_guard<std::mutex> lock(mutex_);
    closeUnlocked();

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

    // Apply indirect-address mappings (motors come up with torque OFF, so writes to addresses 168+/578+
    // are accepted). Then build persistent GroupSyncWrite instances against the write-side indirect
    // data block. This is the path that turns a per-loop 8×4 = 32 read-transactions into 4 (one
    // 23-byte readTxRx per axis through the indirect-data block).
    if (!setupIndirectMappingsUnlocked())
        std::cerr << "[DXL] indirect-address setup encountered errors (telemetry may be partially zero)\n";
    rebuildSyncGroupsUnlocked();
    refreshOpModeCacheUnlocked();

    if (amt21DevicePath && amt21DevicePath[0] != '\0') {
        if (!openAmt21PortUnlocked(std::string(amt21DevicePath), amt21BaudRate))
            std::cerr << "[AMT21] open failed: " << amt21DevicePath << " @ " << amt21BaudRate << "\n";
    } else {
        std::cerr << "[PMI] AMT21 UART disabled (no amt21_serial_device)\n";
    }
    loadZeroOffsetFromFileUnlocked();
    return true;
}

bool DxlBus::setupIndirectMappingsUnlocked()
{
    if (!port_ || !packet_)
        return false;

    // Order of source addresses for the read block — must mirror the offsets at the top of this file.
    static constexpr uint16_t kReadSrc[kReadBlockLength] = {
        kAddrTorqueEnable,                                                          // 1
        kAddrPresentPosition, static_cast<uint16_t>(kAddrPresentPosition + 1),
        static_cast<uint16_t>(kAddrPresentPosition + 2), static_cast<uint16_t>(kAddrPresentPosition + 3), // 4
        kAddrPresentVelocity, static_cast<uint16_t>(kAddrPresentVelocity + 1),
        static_cast<uint16_t>(kAddrPresentVelocity + 2), static_cast<uint16_t>(kAddrPresentVelocity + 3), // 4
        kAddrPresentCurrent, static_cast<uint16_t>(kAddrPresentCurrent + 1),                        // 2
        kAddrHardwareError,                                                         // 1
        kAddrGoalPosition, static_cast<uint16_t>(kAddrGoalPosition + 1),
        static_cast<uint16_t>(kAddrGoalPosition + 2), static_cast<uint16_t>(kAddrGoalPosition + 3), // 4
        kAddrGoalVelocity, static_cast<uint16_t>(kAddrGoalVelocity + 1),
        static_cast<uint16_t>(kAddrGoalVelocity + 2), static_cast<uint16_t>(kAddrGoalVelocity + 3), // 4
        kAddrGoalCurrent, static_cast<uint16_t>(kAddrGoalCurrent + 1),              // 2
        kAddrMoving,                                                                 // 1
    };
    static constexpr uint16_t kWriteSrc[10] = {
        kAddrGoalCurrent, static_cast<uint16_t>(kAddrGoalCurrent + 1),
        kAddrGoalVelocity, static_cast<uint16_t>(kAddrGoalVelocity + 1),
        static_cast<uint16_t>(kAddrGoalVelocity + 2), static_cast<uint16_t>(kAddrGoalVelocity + 3),
        kAddrGoalPosition, static_cast<uint16_t>(kAddrGoalPosition + 1),
        static_cast<uint16_t>(kAddrGoalPosition + 2), static_cast<uint16_t>(kAddrGoalPosition + 3),
    };

    // DYNAMIXEL Protocol 2.0 Status Packet error byte:
    //   bit 7 (0x80) = Alert (motor has a Hardware-Error stored in addr 70 — instruction still OK)
    //   bits 0–6     = Instruction-error code (0x01..0x07): real rejection
    // We must ignore bit 7 here; only the lower 7 bits indicate a write that didn't take.
    constexpr uint8_t kInstrErrMask = 0x7F;

    // Each Indirect Address slot is a single 16-bit register. Use per-slot `write2ByteTxRx` —
    // matches the ROBOTIS reference examples and is what we read back via per-axis readTxRx on
    // the indirect-data block (kAddrIndirectDataRead).
    bool allOk = true;
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        const uint8_t id = kMotorIds[i];

        // Indirect Address writes require Torque Enable = 0; force OFF (no-op if already off).
        uint8_t err = 0;
        const int rTorque = packet_->write1ByteTxRx(port_, id, kAddrTorqueEnable, 0, &err);
        const bool torqueAccepted = (rTorque == COMM_SUCCESS) && ((err & kInstrErrMask) == 0);
        if (!torqueAccepted) {
            std::cerr << "[DXL] disable-torque before indirect setup failed (id=" << static_cast<int>(id)
                      << ") rc=" << packet_->getTxRxResult(rTorque) << " dxl_err=0x"
                      << std::hex << static_cast<int>(err) << std::dec << "\n";
            allOk = false;
            continue;
        }
        if (err & 0x80u) {
            // Hardware-error alert is informational — note it but keep going. User will need to
            // run kCmdResetError (Reset button) eventually to clear addr 70.
            std::cerr << "[DXL] note: id=" << static_cast<int>(id)
                      << " has stored hardware-error (alert bit set) — indirect setup continuing\n";
        }

        // Set Return Delay Time (addr 9, EEPROM) to 0 (minimum = 0 µs) while torque is already
        // OFF. Default is 250 (= 500 µs); setting 0 removes this per-packet latency from every
        // GroupSyncRead response, cutting the read cycle by up to 4 × 500 µs = 2 ms at 1 Mbaud.
        err = 0;
        const int rRdt = packet_->write1ByteTxRx(port_, id, kAddrReturnDelayTime, 0, &err);
        if (rRdt != COMM_SUCCESS || (err & kInstrErrMask) != 0) {
            std::cerr << "[DXL] Return Delay Time set failed (id=" << static_cast<int>(id)
                      << ") rc=" << packet_->getTxRxResult(rRdt) << " dxl_err=0x"
                      << std::hex << static_cast<int>(err) << std::dec << "\n";
        } else {
            std::cerr << "[DXL] Return Delay Time = 0 set for id=" << static_cast<int>(id) << "\n";
        }

        for (size_t k = 0; k < kReadBlockLength; ++k) {
            err = 0;
            const uint16_t addr = static_cast<uint16_t>(kAddrIndirectAddrRead + 2 * k);
            const int rc = packet_->write2ByteTxRx(port_, id, addr, kReadSrc[k], &err);
            if (rc != COMM_SUCCESS || (err & kInstrErrMask) != 0) {
                std::cerr << "[DXL] indirect-read mapping failed (id=" << static_cast<int>(id)
                          << " slot=" << k << " addr=" << addr << "): "
                          << packet_->getTxRxResult(rc) << " dxl_err=0x"
                          << std::hex << static_cast<int>(err) << std::dec << "\n";
                allOk = false;
            }
        }

        for (size_t k = 0; k < 10; ++k) {
            err = 0;
            const uint16_t addr = static_cast<uint16_t>(kAddrIndirectAddrWrite + 2 * k);
            const int rc = packet_->write2ByteTxRx(port_, id, addr, kWriteSrc[k], &err);
            if (rc != COMM_SUCCESS || (err & kInstrErrMask) != 0) {
                std::cerr << "[DXL] indirect-write mapping failed (id=" << static_cast<int>(id)
                          << " slot=" << k << " addr=" << addr << "): "
                          << packet_->getTxRxResult(rc) << " dxl_err=0x"
                          << std::hex << static_cast<int>(err) << std::dec << "\n";
                allOk = false;
            }
        }

        // Sanity check: read back Indirect Address 1 and confirm it points at Torque Enable.
        // If the bus or firmware silently dropped the writes, this catches it before the first
        // telemetry read returns garbage and the control loop reacts to it.
        uint16_t readback = 0;
        err = 0;
        const int rcVerify = packet_->read2ByteTxRx(port_, id, kAddrIndirectAddrRead, &readback, &err);
        const bool verifyOk = (rcVerify == COMM_SUCCESS) && ((err & kInstrErrMask) == 0)
            && (readback == kAddrTorqueEnable);
        if (!verifyOk) {
            std::cerr << "[DXL] indirect-address verification failed (id=" << static_cast<int>(id)
                      << ") rc=" << packet_->getTxRxResult(rcVerify) << " dxl_err=0x"
                      << std::hex << static_cast<int>(err) << std::dec
                      << " readback=" << readback << " expected=" << kAddrTorqueEnable << "\n";
            allOk = false;
        } else {
            std::cerr << "[DXL] indirect-address mapping confirmed for id=" << static_cast<int>(id) << "\n";
        }
    }

    // After ~32 small writes per axis the bridge RX line may still have residue from the last
    // status packet — flushing the SDK port avoids feeding leftover bytes into the first telemetry
    // read after open()/reset.
    port_->clearPort();
    indirect_setup_ok_ = allOk;
    return allOk;
}

void DxlBus::rebuildSyncGroupsUnlocked()
{
    destroySyncGroupsUnlocked();
    if (!port_ || !packet_)
        return;

    sync_read_telemetry_ = new dynamixel::GroupSyncRead(
        port_, packet_, kAddrIndirectDataRead, kReadBlockLength);
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i)
        (void)sync_read_telemetry_->addParam(kMotorIds[i]);

    sync_write_goal_current_ = new dynamixel::GroupSyncWrite(
        port_, packet_, static_cast<uint16_t>(kAddrIndirectDataWrite + kWriteOffGoalCur), 2);
    sync_write_goal_velocity_ = new dynamixel::GroupSyncWrite(
        port_, packet_, static_cast<uint16_t>(kAddrIndirectDataWrite + kWriteOffGoalVel), 4);
    sync_write_goal_position_ = new dynamixel::GroupSyncWrite(
        port_, packet_, static_cast<uint16_t>(kAddrIndirectDataWrite + kWriteOffGoalPos), 4);

    uint8_t zero4[4] = {0, 0, 0, 0};
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        (void)sync_write_goal_current_->addParam(kMotorIds[i], zero4);
        (void)sync_write_goal_velocity_->addParam(kMotorIds[i], zero4);
        (void)sync_write_goal_position_->addParam(kMotorIds[i], zero4);
    }
}

void DxlBus::refreshOpModeCacheUnlocked()
{
    if (!port_ || !packet_)
        return;
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        uint8_t opMode = 3; // factory default = Position Control
        uint8_t err = 0;
        if (packet_->read1ByteTxRx(port_, kMotorIds[i], kAddrOperatingMode, &opMode, &err) == COMM_SUCCESS)
            op_mode_cache_[i] = opMode;
    }
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

    // 115200 baud: wire time ~260 µs + USB round-trip ~2 ms → 5 ms gives 2× margin.
    uint8_t rx[2]{0, 0};
    if (readExactWithTimeout(amt21_fd_, rx, 2, 5) != 2)
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

bool DxlBus::syncReadRawDxlOnlyUnlocked(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    if (!port_ || !packet_ || !sync_read_telemetry_)
        return false;

    // Broadcast GroupSyncRead: one instruction packet → 4 status packets collected in one call.
    const int rc = sync_read_telemetry_->txRxPacket();
    if (rc != COMM_SUCCESS) {
        std::cerr << "[DXL] GroupSyncRead failed: " << packet_->getTxRxResult(rc)
                  << " (rc=" << rc << ")\n";
        port_->clearPort();
        return false;
    }

    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        const uint8_t id = kMotorIds[i];
        if (!sync_read_telemetry_->isAvailable(id, kAddrIndirectDataRead, kReadBlockLength)) {
            std::cerr << "[DXL] GroupSyncRead: no data for id=" << static_cast<int>(id) << "\n";
            return false;
        }

        const uint8_t  torqueOn   = static_cast<uint8_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffTorque,  1));
        const uint8_t  hwErr      = static_cast<uint8_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffHwErr,   1));
        const int16_t  goalCurRaw = static_cast<int16_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffGoalCur, 2));
        const int32_t  goalVelRaw = static_cast<int32_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffGoalVel, 4));
        const int32_t  goalPosRaw = static_cast<int32_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffGoalPos, 4));
        const int16_t  presCurRaw = static_cast<int16_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffPresCur, 2));
        const int32_t  presVelRaw = static_cast<int32_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffPresVel, 4));
        const int32_t  presPosRaw = static_cast<int32_t>(
            sync_read_telemetry_->getData(id, kAddrIndirectDataRead + kReadOffPresPos, 4));

        axes[i].id_op_mode       = pmi::packTelemetryIdOp(id, op_mode_cache_[i]);
        axes[i].servo_state      = torqueOn ? 1 : 0;
        axes[i].present_position = static_cast<double>(presPosRaw) * kPulseToDeg;
        axes[i].encoder_position = std::nan("");   // AMT21 not read in this path
        axes[i].present_velocity = static_cast<double>(presVelRaw) * kVelRawToRpm * kRpmToDegPerSec;
        axes[i].present_current  = static_cast<double>(presCurRaw) * kCurRawToMa * kMaToA;
        axes[i].goal_position    = static_cast<double>(goalPosRaw) * kPulseToDeg;
        axes[i].goal_velocity    = static_cast<double>(goalVelRaw) * kVelRawToRpm * kRpmToDegPerSec;
        axes[i].goal_current     = static_cast<double>(goalCurRaw) * kCurRawToMa * kMaToA;
        axes[i].error_state      = hwErr;
    }
    return true;
}

bool DxlBus::syncReadRawTelemetryUnlocked(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    if (!syncReadRawDxlOnlyUnlocked(axes))
        return false;

    // AMT21 absolute encoders on the secondary UART (axes 1–3 only).
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
    return true;
}

bool DxlBus::syncReadTelemetry(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!syncReadRawTelemetryUnlocked(axes))
        return false;
    applyZeroOffsetUnlocked(axes);
    return true;
}

bool DxlBus::syncReadDxlOnly(pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!syncReadRawDxlOnlyUnlocked(axes))
        return false;
    applyZeroOffsetUnlocked(axes);
    return true;
}

bool DxlBus::readAllAmt21Angles(double encDeg[3])
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (amt21_fd_ < 0)
        return false;

    constexpr uint8_t kNodeAddr[3] = {
        kAmt21AddressForMotor2, kAmt21AddressForMotor3, kAmt21AddressForMotor4
    };
    constexpr size_t kAxisIdx[3] = {1, 2, 3};

    bool anyOk = false;
    const auto now = std::chrono::steady_clock::now();
    constexpr auto kFailLogInterval = std::chrono::seconds(1);

    for (size_t i = 0; i < 3; ++i) {
        encDeg[i] = std::nan("");
        double angleDeg = 0.0;
        const bool ok = readAmt21AngleDegUnlocked(kNodeAddr[i], angleDeg);
        if (ok) {
            // Apply encoder zero-offset (same logic as applyZeroOffsetUnlocked).
            if (zero_offset_valid_ && std::isfinite(encoder_offset_deg_[kAxisIdx[i]]))
                angleDeg -= encoder_offset_deg_[kAxisIdx[i]];
            encDeg[i] = angleDeg;
            anyOk = true;
            if (amt21_in_fail_state_[i]) {
                std::cerr << "[AMT21] recovered: axis " << kAxisIdx[i]
                          << " after " << amt21_fail_count_[i] << " failed read(s)\n";
                amt21_in_fail_state_[i] = false;
                amt21_fail_count_[i] = 0;
                amt21_last_fail_log_[i] = {};
            }
        } else {
            ++amt21_fail_count_[i];
            if (!amt21_in_fail_state_[i] || (now - amt21_last_fail_log_[i]) >= kFailLogInterval) {
                std::cerr << "[AMT21] read failed: axis " << kAxisIdx[i]
                          << " addr=0x" << std::hex << static_cast<int>(kNodeAddr[i]) << std::dec
                          << " (fail count=" << amt21_fail_count_[i] << ")\n";
                amt21_last_fail_log_[i] = now;
            }
            amt21_in_fail_state_[i] = true;
        }
    }
    return anyOk;
}

bool DxlBus::captureZeroOffsetUnlocked()
{
    // Snapshot raw (pre-offset) telemetry — these become the new offsets, so we must NOT apply the
    // currently-stored offsets here.
    pmi::ServoTelemetry snapshot[pmi::kTelemetryAxisCount]{};
    if (!syncReadRawTelemetryUnlocked(snapshot))
        return false;

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
        // Keep our cache aligned with the bus write so the next syncReadTelemetry reports the new mode.
        op_mode_cache_.fill(mode);
    };
    const auto goalCurrentAll = [&](int16_t goalCurrentRaw) {
        if (!sync_write_goal_current_)
            return;
        std::array<std::array<uint8_t, 2>, pmi::kTelemetryAxisCount> params{};
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            const uint16_t raw = static_cast<uint16_t>(goalCurrentRaw);
            params[i][0] = static_cast<uint8_t>(raw & 0xFFu);
            params[i][1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
            (void)sync_write_goal_current_->changeParam(kMotorIds[i], params[i].data());
        }
        (void)sync_write_goal_current_->txPacket();
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
    case pmi::kCmdResetError: {
        // Use the cached op modes (kept in sync via every kCmdMode* / setAxisOperatingMode call) — avoids
        // an extra bus read just for save/restore.
        const std::array<uint8_t, pmi::kTelemetryAxisCount> savedOpModes = op_mode_cache_;
        // Send Reboot instruction (DYNAMIXEL Protocol 2.0 instruction 0x08) to each axis.
        // This clears Hardware Error Status (addr 70), turns torque off, and wipes RAM —
        // including our Indirect Address mappings, which we must re-apply below.
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            uint8_t err = 0;
            const int rc = packet_->reboot(port_, kMotorIds[i], &err);
            if (rc != COMM_SUCCESS)
                std::cerr << "[DXL] reboot id=" << static_cast<int>(kMotorIds[i])
                          << " failed: " << packet_->getTxRxResult(rc) << "\n";
        }
        // Servos take roughly 300–500 ms to enumerate again; wait conservatively so subsequent writes succeed.
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        // Re-apply indirect mappings while torque is still OFF (post-reboot default).
        if (!setupIndirectMappingsUnlocked())
            std::cerr << "[DXL] indirect-address re-setup after reboot encountered errors\n";
        // Restore operating mode but leave torque off — user must explicitly Servo On after recovery.
        for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
            uint8_t err = 0;
            (void)packet_->write1ByteTxRx(port_, kMotorIds[i], kAddrTorqueEnable, 0, &err);
            (void)packet_->write1ByteTxRx(port_, kMotorIds[i], kAddrOperatingMode, savedOpModes[i], &err);
            op_mode_cache_[i] = savedOpModes[i];
        }
        std::cerr << "[PMI] hardware error reset (rebooted IDs 1–4, torque left OFF)\n";
        break;
    }
    case pmi::kCmdModeCurrent:
        torqueAll(0);
        modeAll(0);
        torqueAll(1);
        break;
    case pmi::kCmdModeVelocity:
        torqueAll(0);
        modeAll(1);
        torqueAll(1);
        break;
    case pmi::kCmdModeExtendedPos:
        torqueAll(0);
        modeAll(4);
        torqueAll(1);
        break;
    case pmi::kCmdModeCurrentBasedPos:
        torqueAll(0);
        modeAll(5);
        // Ensure enough allowable torque in current-based position mode.
        goalCurrentAll(kCurrentBasedPosGoalCurrentRaw);
        torqueAll(1);
        break;
    default:
        break;
    }
}

bool DxlBus::writeGoalPositionDeg(const std::array<double, pmi::kTelemetryAxisCount> &motorDeg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_ || !sync_write_goal_position_)
        return false;

    std::array<std::array<uint8_t, 4>, pmi::kTelemetryAxisCount> params{};
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        // Commands are given in zero-referenced motor deg on the server side.
        // Convert back to absolute device space using saved goal offsets.
        const double absMotorDeg = motorDeg[i] + (zero_offset_valid_ ? goal_offset_deg_[i] : 0.0);
        const int32_t pulse = static_cast<int32_t>(std::llround(absMotorDeg / kPulseToDeg));
        const uint32_t raw = static_cast<uint32_t>(pulse);
        params[i][0] = static_cast<uint8_t>(raw & 0xFFu);
        params[i][1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
        params[i][2] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
        params[i][3] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
        // Persistent GroupSyncWrite — changeParam updates the per-ID buffer in-place; one tx for all 4 IDs.
        if (!sync_write_goal_position_->changeParam(kMotorIds[i], params[i].data())) {
            std::cerr << "[DXL] writeGoalPositionDeg changeParam failed (id=" << static_cast<int>(kMotorIds[i]) << ")\n";
            return false;
        }
    }

    const int tx = sync_write_goal_position_->txPacket();
    if (tx != COMM_SUCCESS) {
        std::cerr << "[DXL] writeGoalPositionDeg tx failed: " << packet_->getTxRxResult(tx)
                  << " (code=" << tx << ")\n";
        return false;
    }
    return true;
}

bool DxlBus::writeGoalVelocityDegPerSec(const std::array<double, pmi::kTelemetryAxisCount> &motorDegPerSec)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_ || !sync_write_goal_velocity_)
        return false;

    std::array<std::array<uint8_t, 4>, pmi::kTelemetryAxisCount> params{};
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        const double rawVel = motorDegPerSec[i] / (kVelRawToRpm * kRpmToDegPerSec);
        const int32_t velRaw = static_cast<int32_t>(std::llround(rawVel));
        const uint32_t raw = static_cast<uint32_t>(velRaw);
        params[i][0] = static_cast<uint8_t>(raw & 0xFFu);
        params[i][1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
        params[i][2] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
        params[i][3] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
        if (!sync_write_goal_velocity_->changeParam(kMotorIds[i], params[i].data())) {
            std::cerr << "[DXL] writeGoalVelocityDegPerSec changeParam failed (id=" << static_cast<int>(kMotorIds[i]) << ")\n";
            return false;
        }
    }

    const int tx = sync_write_goal_velocity_->txPacket();
    if (tx != COMM_SUCCESS) {
        std::cerr << "[DXL] writeGoalVelocityDegPerSec tx failed: " << packet_->getTxRxResult(tx)
                  << " (code=" << tx << ")\n";
        return false;
    }
    return true;
}

bool DxlBus::writeGoalCurrentAmp(const std::array<double, pmi::kTelemetryAxisCount> &motorAmp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_ || !sync_write_goal_current_)
        return false;

    // e-Manual XM540-W270 Goal Current(102): unit 2.69 [mA]; value clamped by Current Limit(38).
    constexpr double kAmpPerRawUnit = kCurRawToMa * kMaToA;
    constexpr int32_t kGoalCurrentRawAbsMax = 1193; // ~3.21 A; stay within factory Current Limit unless raised

    std::array<std::array<uint8_t, 2>, pmi::kTelemetryAxisCount> params{};
    for (size_t i = 0; i < pmi::kTelemetryAxisCount; ++i) {
        int32_t raw = static_cast<int32_t>(std::llround(motorAmp[i] / kAmpPerRawUnit));
        if (raw > kGoalCurrentRawAbsMax)
            raw = kGoalCurrentRawAbsMax;
        if (raw < -kGoalCurrentRawAbsMax)
            raw = -kGoalCurrentRawAbsMax;
        const uint16_t uraw = static_cast<uint16_t>(static_cast<int16_t>(raw));
        params[i][0] = static_cast<uint8_t>(uraw & 0xFFu);
        params[i][1] = static_cast<uint8_t>((uraw >> 8) & 0xFFu);
        if (!sync_write_goal_current_->changeParam(kMotorIds[i], params[i].data())) {
            std::cerr << "[DXL] writeGoalCurrentAmp changeParam failed (id=" << static_cast<int>(kMotorIds[i]) << ")\n";
            return false;
        }
    }

    const int tx = sync_write_goal_current_->txPacket();
    if (tx != COMM_SUCCESS) {
        std::cerr << "[DXL] writeGoalCurrentAmp tx failed: " << packet_->getTxRxResult(tx) << " (code=" << tx << ")\n";
        return false;
    }
    return true;
}

bool DxlBus::setAxisTorque(size_t axisIndex, bool enable)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_ || axisIndex >= pmi::kTelemetryAxisCount)
        return false;
    uint8_t dxl_error = 0;
    const int rc = packet_->write1ByteTxRx(port_, kMotorIds[axisIndex], kAddrTorqueEnable, enable ? 1 : 0, &dxl_error);
    return rc == COMM_SUCCESS;
}

bool DxlBus::setAxisOperatingMode(size_t axisIndex, uint8_t mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_ || axisIndex >= pmi::kTelemetryAxisCount)
        return false;
    uint8_t dxl_error = 0;
    const int rc = packet_->write1ByteTxRx(port_, kMotorIds[axisIndex], kAddrOperatingMode, mode, &dxl_error);
    if (rc == COMM_SUCCESS)
        op_mode_cache_[axisIndex] = mode;
    return rc == COMM_SUCCESS;
}

bool DxlBus::setAxisGoalCurrentRaw(size_t axisIndex, int16_t goalCurrentRaw)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_ || !packet_ || axisIndex >= pmi::kTelemetryAxisCount)
        return false;
    uint8_t dxl_error = 0;
    const uint16_t raw = static_cast<uint16_t>(goalCurrentRaw);
    const int rc = packet_->write2ByteTxRx(port_, kMotorIds[axisIndex], kAddrGoalCurrent, raw, &dxl_error);
    return rc == COMM_SUCCESS;
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
