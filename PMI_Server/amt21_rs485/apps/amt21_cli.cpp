// AMT212H-V RS485 독립 테스트 CLI (Same Sky, AMT21 시리즈 데이터시트 프로토콜).
// amt21_cli <시리얼장치> [보드레이트] [노드주소] [12|14] [--turns] [--once] [--verbose]
//       amt21_cli <시리얼장치> --scan-baud [--scan-trials N] [0xNN] [12|14]
// 예: amt21_cli /dev/ttyUSB0 115200 0x54 14 --turns

#include "amt21/amt21_driver.hpp"
#include "amt21/model.hpp"
#include "amt21/protocol.hpp"
#include "amt21/rs485_port.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

int parseBitsToken(const char *s)
{
    if (!s)
        return -1;
    if (std::strcmp(s, "12") == 0)
        return 12;
    if (std::strcmp(s, "14") == 0)
        return 14;
    return -1;
}

void usage(const char *argv0)
{
    std::cerr << "Target: " << amt21::model::kPartNumber << " (14-bit multi-turn, adjustable baud)\n"
              << "Usage: " << argv0
              << " <serial_device> [baud] [node_0xNN] [12|14] [--turns] [--once] [--verbose]\n"
              << "       " << argv0
              << " <serial_device> --scan-baud [--scan-trials N] [node_0xNN] [12|14]\n"
              << "  defaults: baud=115200 node=0x54 bits=14\n"
              << "  baud must match AMT Viewpoint / label (H option: 115200/38400/...; not 2 Mbps).\n"
              << "  --verbose: 체크섬·디코드 성공할 때까지 무한 재시도(시도마다 로그, Ctrl+C 종료). "
                 "비-verbose는 드라이버 최대 " << amt21::kAmt21ReadRetries << "회 재시도.\n"
              << "  --scan-baud: Rs485Port가 지원하는 보드레이트마다 위치 읽기 "
                 "N회(기본 20) 시도·체크섬 성공 비율 표시.\n"
              << "  --scan-trials N: 스캔 시 보드레이트당 시도 횟수(기본 20, 최소 1).\n";
}

/// `rs485_port.cpp`의 baudToFlag와 동일 목록(열기 실패하는 값은 스캔에서 건너뜀).
constexpr int kScanBaudRates[] = {
    9600,   19200,  38400,   57600,   115200,  230400,  460800,  500000, 576000,
    921600, 1000000, 1500000, 2000000, 2500000, 3000000, 4000000,
};

struct BaudScanRow {
    int baud = 0;
    bool openOk = false;
    int trials = 0;
    int timeouts = 0;
    int rxBytes = 0;
    int checksumOk = 0;
    int decodeOk = 0;
};

static bool argLooksLikeNode(const char *s)
{
    return s && (std::strncmp(s, "0x", 2) == 0 || std::strncmp(s, "0X", 2) == 0);
}

static int runBaudScan(const char *dev, uint8_t node, int bits, int trialsPerBaud)
{
    std::vector<BaudScanRow> rows;
    rows.reserve(sizeof(kScanBaudRates) / sizeof(kScanBaudRates[0]));

    for (const int b : kScanBaudRates) {
        BaudScanRow row;
        row.baud = b;
        row.trials = trialsPerBaud;

        amt21::Rs485Port port;
        if (!port.open(dev, b)) {
            row.openOk = false;
            rows.push_back(row);
            continue;
        }
        row.openOk = true;
        amt21::Amt21Driver enc(port);
        enc.setNodeBase(node);

        for (int t = 0; t < trialsPerBaud; ++t) {
            const auto raw = enc.readPositionRawFrame();
            if (!raw) {
                row.timeouts++;
                continue;
            }
            row.rxBytes++;
            if (amt21::verifyResponseChecksum(*raw))
                row.checksumOk++;
            if (bits == 14) {
                if (amt21::decodePosition14(*raw).has_value())
                    row.decodeOk++;
            } else {
                if (amt21::decodePosition12(*raw).has_value())
                    row.decodeOk++;
            }
            ::usleep(amt21::kAmt21ReadRetryDelayUs);
        }
        port.close();
        rows.push_back(row);
        ::usleep(15000);
    }

    std::sort(rows.begin(), rows.end(), [](const BaudScanRow &a, const BaudScanRow &b) {
        if (a.openOk != b.openOk)
            return a.openOk > b.openOk;
        if (a.checksumOk != b.checksumOk)
            return a.checksumOk > b.checksumOk;
        return a.baud < b.baud;
    });

    std::cerr << "\n[scan-baud] device=" << dev << " node=0x" << std::hex << static_cast<int>(node)
              << std::dec << " bits=" << bits << " trials/baud=" << trialsPerBaud << "\n";
    std::cerr << std::right << std::setw(10) << "baud" << std::setw(7) << "open" << std::setw(8)
              << "timeout" << std::setw(8) << "rx_ok" << std::setw(8) << "cs_ok" << std::setw(8)
              << "decode" << std::setw(10) << "cs_ok%\n";

    int bestBaud = -1;
    int bestScore = -1;
    for (const auto &r : rows) {
        if (!r.openOk) {
            std::cerr << std::setw(10) << r.baud << std::setw(7) << "no" << std::setw(8) << "-"
                      << std::setw(8) << "-" << std::setw(8) << "-" << std::setw(8) << "-" << std::setw(10)
                      << "-\n";
            continue;
        }
        const int pct = (r.trials > 0) ? (100 * r.checksumOk / r.trials) : 0;
        std::cerr << std::setw(10) << r.baud << std::setw(7) << "yes" << std::setw(8) << r.timeouts
                  << std::setw(8) << r.rxBytes << std::setw(8) << r.checksumOk << std::setw(8)
                  << r.decodeOk << std::setw(9) << pct << "%\n";
        if (r.checksumOk > bestScore) {
            bestScore = r.checksumOk;
            bestBaud = r.baud;
        }
    }

    if (bestScore > 0)
        std::cerr << "\n[scan-baud] 체크섬 성공이 가장 많은 보드레이트: " << bestBaud << " (" << bestScore
                  << "/" << trialsPerBaud << ")\n";
    else
        std::cerr << "\n[scan-baud] 모든 보드레이트에서 체크섬 성공 0회. 배선·노드·종단을 확인하세요.\n";
    return (bestScore > 0) ? 0 : 2;
}
bool parseHexByte(const char *s, uint8_t &out)
{
    if (!s || !*s)
        return false;
    char *end = nullptr;
    long v = std::strtol(s, &end, 0);
    if (end == s || v < 0 || v > 255)
        return false;
    out = static_cast<uint8_t>(v);
    return true;
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const char *dev = argv[1];
    int baud = 115200;
    uint8_t node = amt21::kDefaultNodeAddress;
    int bits = 14;
    bool once = false;
    bool withTurns = false;
    bool verbose = false;
    bool scanBaud = false;
    int scanTrials = 20;

    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--once") == 0) {
            once = true;
            continue;
        }
        if (std::strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
            continue;
        }
        if (std::strcmp(argv[i], "--turns") == 0) {
            withTurns = true;
            continue;
        }
        if (std::strcmp(argv[i], "--scan-baud") == 0) {
            scanBaud = true;
            continue;
        }
        if (std::strcmp(argv[i], "--scan-trials") == 0) {
            if (i + 1 < argc) {
                const int n = std::atoi(argv[++i]);
                scanTrials = (n > 0) ? n : 1;
            }
            continue;
        }
        const int pb = parseBitsToken(argv[i]);
        if (pb >= 0) {
            bits = pb;
            continue;
        }
    }
    if (!scanBaud && argc >= 3 && std::strcmp(argv[2], "--once") != 0
        && std::strcmp(argv[2], "--verbose") != 0 && std::strcmp(argv[2], "--turns") != 0
        && std::strcmp(argv[2], "--scan-baud") != 0 && std::strcmp(argv[2], "--scan-trials") != 0) {
        const long b = std::strtol(argv[2], nullptr, 10);
        if (b > 0)
            baud = static_cast<int>(b);
    }
    if (scanBaud) {
        for (int i = 2; i < argc; ++i) {
            if (!argv[i] || argv[i][0] == '-')
                continue;
            uint8_t tmp = 0;
            if (argLooksLikeNode(argv[i]) && parseHexByte(argv[i], tmp))
                node = tmp;
            const int pb = parseBitsToken(argv[i]);
            if (pb >= 0)
                bits = pb;
        }
    } else {
        if (argc >= 4 && std::strcmp(argv[3], "--once") != 0 && std::strcmp(argv[3], "12") != 0
            && std::strcmp(argv[3], "14") != 0 && std::strcmp(argv[3], "--verbose") != 0
            && std::strcmp(argv[3], "--scan-baud") != 0 && std::strcmp(argv[3], "--scan-trials") != 0) {
            (void)parseHexByte(argv[3], node);
        }
        if (argc >= 5 && std::strcmp(argv[4], "--once") != 0 && std::strcmp(argv[4], "--turns") != 0
            && std::strcmp(argv[4], "--verbose") != 0 && std::strcmp(argv[4], "--scan-baud") != 0
            && std::strcmp(argv[4], "--scan-trials") != 0) {
            const int pb = parseBitsToken(argv[4]);
            if (pb >= 0)
                bits = pb;
        }
    }

    if (scanBaud)
        return runBaudScan(dev, node, bits, scanTrials);

    amt21::Rs485Port port;
    if (!port.open(dev, baud)) {
        std::cerr << "open " << dev << " @ " << baud << " failed (errno " << errno << ")\n";
        return 1;
    }

    amt21::Amt21Driver enc(port);
    enc.setNodeBase(node);

    std::cerr << amt21::model::kPartNumber << " " << dev << " baud=" << baud << " node=0x" << std::hex
              << static_cast<int>(node) << std::dec << " bits=" << bits << (withTurns ? " +turns" : "")
              << (once ? " once\n" : " (Ctrl+C)\n");

    for (;;) {
        std::optional<uint16_t> p;
        if (verbose) {
            for (int a = 0;; ++a) {
                const auto raw = enc.readPositionRawFrame();
                if (!raw) {
                    std::cerr << "[verbose] try " << a << ": 타임아웃(2바이트 미수신 또는 전송 실패)\n";
                } else {
                    const uint8_t b0 = static_cast<uint8_t>(*raw & 0xFFu);
                    const uint8_t b1 = static_cast<uint8_t>((*raw >> 8) & 0xFFu);
                    const bool csOk = amt21::verifyResponseChecksum(*raw);
                    std::cerr << "[verbose] try " << a << ": low_byte_first b0=0x" << std::hex
                              << std::uppercase << std::setw(2) << std::setfill('0')
                              << static_cast<unsigned>(b0) << " b1=0x" << std::setw(2)
                              << static_cast<unsigned>(b1) << " word=0x" << std::setw(4) << *raw
                              << std::dec << " checksum=" << (csOk ? "ok" : "FAIL") << "\n";
                    if (bits == 14)
                        p = amt21::decodePosition14(*raw);
                    else
                        p = amt21::decodePosition12(*raw);
                    if (p) {
                        std::cerr << "[verbose] 디코드 성공 (시도 " << a << ", checksum=ok)\n";
                        break;
                    }
                }
                if (p)
                    break;
                ::usleep(amt21::kAmt21ReadRetryDelayUs);
            }
        } else {
            if (bits == 14)
                p = enc.readPosition14();
            else
                p = enc.readPosition12();
        }

        if (bits == 14) {
            if (p)
                std::cout << "position14 " << *p;
            else
                std::cout << "position read failed";
        } else {
            if (p)
                std::cout << "position12 " << *p;
            else
                std::cout << "position read failed";
        }
        if (withTurns) {
            const auto t = enc.readTurns14();
            if (t)
                std::cout << " turns14 " << *t;
            else
                std::cout << " turns_read_failed";
        }
        std::cout << "\n";
        if (once)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
