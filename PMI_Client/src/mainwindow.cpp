// PMI client main window: Qt UI plus TcpClient; network callbacks are marshaled to the GUI thread.

#include "mainwindow.h"

#include "tcp_client.h"
#include "ui_mainwindow.h"

#include <QAbstractItemView>
#include <QByteArray>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QHeaderView>
#include <QMessageBox>
#include <QSettings>
#include <QStringList>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableWidgetItem>
#include <QTimer>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>

namespace {

QString opModeToText(uint8_t op)
{
    switch (op) {
    case 0:
        return QStringLiteral("Current");
    case 1:
        return QStringLiteral("Velocity");
    case 3:
        return QStringLiteral("Position");
    case 4:
        return QStringLiteral("Extended Position");
    case 5:
        return QStringLiteral("Current-based Position");
    case 16:
        return QStringLiteral("PWM");
    default:
        return QStringLiteral("Unknown (%1)").arg(op);
    }
}

double toJointDeg(size_t axis, double motorDeg)
{
    constexpr double kGear[4] = {32.0 / 60.0, 54.0 / 360.0, 108.0 / 360.0, 108.0 / 360.0};
    if (axis >= 4 || !std::isfinite(motorDeg))
        return std::numeric_limits<double>::quiet_NaN();
    return motorDeg * kGear[axis];
}

/// DYNAMIXEL X 시리즈 Address 70 (Hardware Error Status) 비트 해석 (XM540 등).
QString hardwareErrorToText(uint8_t hw)
{
    if (hw == 0)
        return QCoreApplication::translate("MainWindow", "정상");

    QStringList parts;
    if (hw & 0x01)
        parts.append(QCoreApplication::translate("MainWindow", "입력 전압"));
    if (hw & 0x04)
        parts.append(QCoreApplication::translate("MainWindow", "과열"));
    if (hw & 0x08)
        parts.append(QCoreApplication::translate("MainWindow", "모터 엔코더"));
    if (hw & 0x10)
        parts.append(QCoreApplication::translate("MainWindow", "전기적 충격"));
    if (hw & 0x20)
        parts.append(QCoreApplication::translate("MainWindow", "과부하"));

    const uint8_t knownMask = 0x01u | 0x04u | 0x08u | 0x10u | 0x20u;
    const uint8_t unknown = static_cast<uint8_t>(hw & ~knownMask);
    if (unknown != 0) {
        parts.append(QCoreApplication::translate("MainWindow", "기타(0x%1)")
                         .arg(unknown, 2, 16, QChar('0'))
                         .toUpper());
    }

    if (parts.isEmpty())
        return QCoreApplication::translate("MainWindow", "알 수 없음 (0x%1)")
            .arg(hw, 2, 16, QChar('0'))
            .toUpper();

    return parts.join(QStringLiteral(", "))
        + QStringLiteral(" (0x")
        + QString::number(hw, 16).toUpper().rightJustified(2, QChar('0'))
        + QChar(')');
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(std::make_unique<Ui::MainWindow>())
    , m_cmdNet(std::make_unique<TcpClient>())
    , m_telemetryNet(std::make_unique<TcpClient>())
{
    ui->setupUi(this);

    m_cmdNet->setCallbacks(
        [this]() {
            QTimer::singleShot(0, this, [this]() {
                m_cmdConnected = true;
                updateConnectionUiState();
            });
        },
        [this]() {
            QTimer::singleShot(0, this, [this]() {
                m_cmdConnected = false;
                updateConnectionUiState();
            });
        },
        [this](std::vector<uint8_t> chunk) {
            onCommandBytesFromWorker(std::move(chunk));
        },
        [this](std::string message) {
            const QString qmsg = QString::fromUtf8(message.data(), static_cast<int>(message.size()));
            QTimer::singleShot(0, this, [this, qmsg]() { onNetError(tr("Command channel: %1").arg(qmsg)); });
        });
    m_telemetryNet->setCallbacks(
        [this]() {
            QTimer::singleShot(0, this, [this]() {
                m_telemetryConnected = true;
                updateConnectionUiState();
            });
        },
        [this]() {
            QTimer::singleShot(0, this, [this]() {
                m_telemetryConnected = false;
                updateConnectionUiState();
            });
        },
        [this](std::vector<uint8_t> chunk) {
            onTelemetryBytesFromWorker(std::move(chunk));
        },
        [this](std::string message) {
            const QString qmsg = QString::fromUtf8(message.data(), static_cast<int>(message.size()));
            QTimer::singleShot(0, this, [this, qmsg]() { onNetError(tr("Telemetry channel: %1").arg(qmsg)); });
        });
    m_cmdNet->start();
    m_telemetryNet->start();
    m_logCountdownTimer = new QTimer(this);
    m_logCountdownTimer->setInterval(100);
    connect(m_logCountdownTimer, &QTimer::timeout, this, [this]() {
        const qint64 remainMs = m_logEndMs - QDateTime::currentMSecsSinceEpoch();
        if (remainMs <= 0) {
            stopLogCountdown();
            ui->labelLogStatus->setText(tr("Status: logging duration elapsed"));
            return;
        }
        ui->labelLogStatus->setText(tr("Status: logging... %1 s left")
                                        .arg(QString::number(static_cast<double>(remainMs) / 1000.0, 'f', 1)));
    });

    connect(ui->btnConnect, &QPushButton::toggled, this, &MainWindow::onConnectToggled);
    connect(ui->btnServoOn, &QPushButton::clicked, this, &MainWindow::onServoOnClicked);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui->btnZero, &QPushButton::clicked, this, &MainWindow::onZeroClicked);
    connect(ui->btnReset, &QPushButton::clicked, this, &MainWindow::onResetClicked);
    connect(ui->btnCurrent, &QPushButton::clicked, this, &MainWindow::onModeCurrentClicked);
    connect(ui->btnVelocity, &QPushButton::clicked, this, &MainWindow::onModeVelocityClicked);
    connect(ui->btnPosition, &QPushButton::clicked, this, &MainWindow::onModeExtendedPosClicked);
    connect(ui->btnCurrentBasedPosition, &QPushButton::clicked, this, &MainWindow::onModeCurrentBasedPosClicked);
    connect(ui->btnJogPlus, &QPushButton::clicked, this, &MainWindow::onJogPlusClicked);
    connect(ui->btnJogMinus, &QPushButton::clicked, this, &MainWindow::onJogMinusClicked);
    connect(ui->btnJogStop, &QPushButton::clicked, this, &MainWindow::onJogStopClicked);
    connect(ui->btnSendWaypoints, &QPushButton::clicked, this, &MainWindow::onSendWaypointsClicked);
    connect(ui->btnPlanPath, &QPushButton::clicked, this, &MainWindow::onPlanPathClicked);
    connect(ui->btnTrajStart, &QPushButton::clicked, this, &MainWindow::onStartTrajectoryClicked);
    connect(ui->btnTrajStop, &QPushButton::clicked, this, &MainWindow::onStopTrajectoryClicked);
    connect(ui->btnSendInitialPose, &QPushButton::clicked, this, &MainWindow::onSendInitialPoseClicked);
    connect(ui->btnSendZeroPose, &QPushButton::clicked, this, &MainWindow::onSendZeroPoseClicked);
    connect(ui->btnLogStart, &QPushButton::clicked, this, &MainWindow::onLogStartClicked);
    connect(ui->btnLogStop, &QPushButton::clicked, this, &MainWindow::onLogStopClicked);

    setUiConnected(false);
    setServoButtonState(false);
    setupTelemetryTable();
    ui->waypointTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->jogSpeedEdit->setReadOnly(true);
    ui->jogSpeedEdit->setText(QStringLiteral("0.0"));

    QSettings settings;
    const QByteArray geom = settings.value(QStringLiteral("mainwindow/geometry")).toByteArray();
    if (!geom.isEmpty())
        restoreGeometry(geom);
    ui->initQ1Edit->setText(settings.value(QStringLiteral("trajectory/init_q1_deg"),
                              settings.value(QStringLiteral("trajectory/init_q1_rad"), QStringLiteral("0.0"))).toString());
    ui->initQ2Edit->setText(settings.value(QStringLiteral("trajectory/init_q2_deg"),
                              settings.value(QStringLiteral("trajectory/init_q2_rad"), QStringLiteral("0.0"))).toString());
    ui->initQ3Edit->setText(settings.value(QStringLiteral("trajectory/init_q3_deg"),
                              settings.value(QStringLiteral("trajectory/init_q3_rad"), QStringLiteral("0.0"))).toString());
    ui->initQ4Edit->setText(settings.value(QStringLiteral("trajectory/init_q4_deg"),
                              settings.value(QStringLiteral("trajectory/init_q4_rad"), QStringLiteral("0.0"))).toString());
    for (int r = 0; r < ui->waypointTable->rowCount(); ++r) {
        for (int c = 0; c < ui->waypointTable->columnCount(); ++c) {
            const QString key = QStringLiteral("trajectory/waypoint_r%1_c%2").arg(r).arg(c);
            const QString v = settings.value(key, QString()).toString();
            if (v.isEmpty())
                continue;
            if (!ui->waypointTable->item(r, c))
                ui->waypointTable->setItem(r, c, new QTableWidgetItem(v));
            else
                ui->waypointTable->item(r, c)->setText(v);
        }
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QSettings settings;
    settings.setValue(QStringLiteral("mainwindow/geometry"), saveGeometry());
    settings.setValue(QStringLiteral("trajectory/init_q1_deg"), ui->initQ1Edit->text().trimmed());
    settings.setValue(QStringLiteral("trajectory/init_q2_deg"), ui->initQ2Edit->text().trimmed());
    settings.setValue(QStringLiteral("trajectory/init_q3_deg"), ui->initQ3Edit->text().trimmed());
    settings.setValue(QStringLiteral("trajectory/init_q4_deg"), ui->initQ4Edit->text().trimmed());
    for (int r = 0; r < ui->waypointTable->rowCount(); ++r) {
        for (int c = 0; c < ui->waypointTable->columnCount(); ++c) {
            const QString key = QStringLiteral("trajectory/waypoint_r%1_c%2").arg(r).arg(c);
            const QTableWidgetItem *item = ui->waypointTable->item(r, c);
            settings.setValue(key, item ? item->text().trimmed() : QString());
        }
    }
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() = default;

void MainWindow::onConnectToggled(bool checked)
{
    if (!checked) {
        m_cmdNet->requestDisconnect();
        m_telemetryNet->requestDisconnect();
        return;
    }

    const QString host = ui->ipEdit->text().trimmed();
    if (host.isEmpty()) {
        QMessageBox::warning(this, tr("Input error"), tr("Enter a valid IP address."));
        ui->btnConnect->setChecked(false);
        return;
    }

    const QByteArray hostUtf8 = host.toUtf8();
    const std::string hostStr(hostUtf8.constData(), static_cast<size_t>(hostUtf8.size()));
    m_telemetryNet->requestConnect(hostStr, 9000);
    m_cmdNet->requestConnect(hostStr, 9001);
}

void MainWindow::onNetConnected()
{
    m_cmdConnected = true;
    m_telemetryConnected = true;
    updateConnectionUiState();
}

void MainWindow::updateConnectionUiState()
{
    const bool connected = m_cmdConnected && m_telemetryConnected;
    setUiConnected(connected);
    if (connected)
        sendClientCmd(pmi::kCmdPing);
}

void MainWindow::onNetDisconnected()
{
    m_cmdConnected = false;
    m_telemetryConnected = false;
    m_telemetryRx.clear();
    m_commandRx.clear();
    clearTelemetryTable();
    setServoButtonState(false);
    setUiConnected(false);
    if (ui->btnConnect->isChecked())
        ui->btnConnect->setChecked(false);
}

void MainWindow::onNetError(const QString &message)
{
    QMessageBox::warning(this, tr("Network error"), message);
    m_cmdConnected = false;
    m_telemetryConnected = false;
    if (ui->btnConnect->isChecked())
        ui->btnConnect->setChecked(false);
    m_telemetryRx.clear();
    m_commandRx.clear();
    clearTelemetryTable();
    setServoButtonState(false);
    setUiConnected(false);
}

void MainWindow::setUiConnected(bool connected)
{
    ui->ipEdit->setEnabled(!connected);
    ui->portEdit->setEnabled(false);
    ui->portEdit->setText(QStringLiteral("9000/9001"));
    ui->btnConnect->setText(connected ? tr("Disconnect") : tr("Connect"));
    ui->btnSendWaypoints->setEnabled(connected);
    ui->btnSendInitialPose->setEnabled(connected);
    ui->btnSendZeroPose->setEnabled(connected);
    ui->btnLogStart->setEnabled(connected);
    ui->btnLogStop->setEnabled(connected);
    ui->jogAxisCombo->setEnabled(connected);
    ui->jogSpeedEdit->setEnabled(connected);
    ui->btnJogPlus->setEnabled(connected);
    ui->btnJogMinus->setEnabled(connected);
    ui->btnJogStop->setEnabled(connected);
    if (!connected)
        m_waypointSent = false;
    updateTrajectoryButtonState(connected);
}

void MainWindow::updateTrajectoryButtonState(bool connected)
{
    const bool enabled = connected && m_waypointSent;
    ui->btnPlanPath->setEnabled(enabled);
    ui->btnTrajStart->setEnabled(enabled);
    ui->btnTrajStop->setEnabled(enabled);
}

void MainWindow::sendClientCmd(uint8_t cmd)
{
    if (!(m_cmdConnected && m_telemetryConnected))
        return;
    const std::vector<uint8_t> frame = pmi::buildClientFrame(cmd, {});
    m_cmdNet->requestSend(frame);
}

void MainWindow::setServoButtonState(bool servoOn)
{
    m_servoOn = servoOn;
    ui->btnServoOn->setText(m_servoOn ? tr("Servo Off") : tr("Servo On"));
}

void MainWindow::onTelemetryBytesFromWorker(std::vector<uint8_t> chunk)
{
    auto shared = std::make_shared<std::vector<uint8_t>>(std::move(chunk));
    QTimer::singleShot(0, this, [this, shared]() {
        m_telemetryRx.insert(m_telemetryRx.end(), shared->begin(), shared->end());
        pmi::feedServerRxStream(
            m_telemetryRx,
            [this](const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]) { updateTelemetryTable(axes); });
    });
}

void MainWindow::onCommandBytesFromWorker(std::vector<uint8_t> chunk)
{
    auto shared = std::make_shared<std::vector<uint8_t>>(std::move(chunk));
    QTimer::singleShot(0, this, [this, shared]() {
        m_commandRx.insert(m_commandRx.end(), shared->begin(), shared->end());
        pmi::feedClientRxStream(m_commandRx, [this](uint8_t msg, const std::vector<uint8_t> &payload) {
            if (msg != pmi::kSrvAck)
                return;
            const QString text = QString::fromUtf8(reinterpret_cast<const char *>(payload.data()),
                static_cast<int>(payload.size()));
            if (text.startsWith(QStringLiteral("LOG_START_OK:"))) {
                ui->labelLogStatus->setText(tr("Status: log file: %1").arg(text.mid(13)));
                return;
            }
            if (text.startsWith(QStringLiteral("LOG_STOP_OK:"))) {
                stopLogCountdown();
                ui->labelLogStatus->setText(tr("Status: logging stopped"));
                return;
            }
            if (text.startsWith(QStringLiteral("LOG_START_FAIL:"))) {
                stopLogCountdown();
                ui->labelLogStatus->setText(tr("Status: logging start failed (%1)").arg(text.mid(15)));
                return;
            }
            if (text.startsWith(QStringLiteral("INIT_POSE_PROGRESS:"))) {
                const double p = text.mid(19).toDouble();
                ui->labelTrajectoryStatus->setText(
                    tr("Status: init pose moving (%1%)").arg(QString::number(p, 'f', 1)));
                return;
            }
            if (text == QStringLiteral("INIT_POSE_DONE")) {
                ui->labelTrajectoryStatus->setText(tr("Status: init pose completed (100.0%)"));
                return;
            }
            if (text.startsWith(QStringLiteral("PLAN_OK_CLIPPED:"))) {
                const QString rest = text.mid(16);
                const int atIdx = rest.indexOf(QLatin1Char('@'));
                const QString samples = (atIdx > 0) ? rest.left(atIdx) : rest;
                const QString clippedAt = (atIdx > 0) ? rest.mid(atIdx + 1) : QStringLiteral("?");
                ui->labelTrajectoryStatus->setText(
                    tr("Status: planned %1 samples — clipped to joint limits (first @ %2)")
                        .arg(samples).arg(clippedAt));
                return;
            }
            if (text.startsWith(QStringLiteral("PLAN_OK:"))) {
                ui->labelTrajectoryStatus->setText(tr("Status: planned %1 samples").arg(text.mid(8)));
                return;
            }
            if (text == QStringLiteral("PLAN_FAIL")) {
                ui->labelTrajectoryStatus->setText(tr("Status: plan failed"));
                return;
            }
            ui->labelLogStatus->setText(tr("Status: %1").arg(text));
        });
    });
}

void MainWindow::setupTelemetryTable()
{
    constexpr int kCols = 12;
    m_telemetryModel = new QStandardItemModel(static_cast<int>(pmi::kTelemetryAxisCount), kCols, this);
    m_telemetryModel->setHorizontalHeaderLabels({
        tr("Axis"),
        tr("ID"),
        tr("Op"),
        tr("State"),
        tr("Motor Position (deg)"),
        tr("Encoder Position (deg)"),
        tr("Joint Position (deg)"),
        tr("Motor Velocity (deg/s)"),
        tr("Motor Current (A)"),
        tr("Goal Velocity (deg/s)"),
        tr("Goal Current (A)"),
        tr("HW Error"),
    });

    const QString dash = QStringLiteral("—");
    for (int row = 0; row < static_cast<int>(pmi::kTelemetryAxisCount); ++row) {
        for (int col = 0; col < kCols; ++col) {
            QStandardItem *item = new QStandardItem(col == 0 ? QString::number(row) : dash);
            item->setEditable(false);
            item->setTextAlignment(col == 0 ? Qt::AlignCenter : Qt::AlignRight | Qt::AlignVCenter);
            m_telemetryModel->setItem(row, col, item);
        }
    }

    ui->telemetryTableView->setModel(m_telemetryModel);
    ui->telemetryTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->telemetryTableView->setSelectionMode(QAbstractItemView::NoSelection);
    ui->telemetryTableView->verticalHeader()->setVisible(false);
    ui->telemetryTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->telemetryTableView->horizontalHeader()->setStretchLastSection(true);
}

void MainWindow::clearTelemetryTable()
{
    if (!m_telemetryModel)
        return;
    const QString dash = QStringLiteral("—");
    for (int row = 0; row < static_cast<int>(pmi::kTelemetryAxisCount); ++row) {
        for (int col = 1; col < m_telemetryModel->columnCount(); ++col)
            m_telemetryModel->item(row, col)->setText(dash);
    }
}

void MainWindow::updateTelemetryTable(const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount])
{
    if (!m_telemetryModel)
        return;
    for (int row = 0; row < static_cast<int>(pmi::kTelemetryAxisCount); ++row) {
        const pmi::ServoTelemetry &t = axes[static_cast<size_t>(row)];
        m_telemetryModel->item(row, 1)->setText(QString::number(row + 1));
        m_telemetryModel->item(row, 2)->setText(QStringLiteral("-"));
        m_telemetryModel->item(row, 3)->setText(QString::number(t.servo_state));
        m_telemetryModel->item(row, 4)->setText(QString::number(t.present_position, 'f', 4));
        m_telemetryModel->item(row, 5)->setText(std::isfinite(t.encoder_position) ? QString::number(t.encoder_position, 'f', 4)
                                                                                   : QStringLiteral("—"));
        // External encoder is treated as joint-side angle; use gear conversion only for motor-side fallback.
        const double jointDeg = std::isfinite(t.encoder_position) ? t.encoder_position
                                                                   : toJointDeg(static_cast<size_t>(row), t.present_position);
        m_telemetryModel->item(row, 6)->setText(std::isfinite(jointDeg) ? QString::number(jointDeg, 'f', 4) : QStringLiteral("—"));
        m_telemetryModel->item(row, 7)->setText(QString::number(t.present_velocity, 'f', 4));
        m_telemetryModel->item(row, 8)->setText(QString::number(t.present_current, 'f', 4));
        m_telemetryModel->item(row, 9)->setText(QString::number(t.goal_velocity, 'f', 4));
        m_telemetryModel->item(row, 10)->setText(QString::number(t.goal_current, 'f', 4));
        m_telemetryModel->item(row, 11)->setText(hardwareErrorToText(t.error_state));
    }

}

void MainWindow::onServoOnClicked()
{
    if (m_servoOn) {
        sendClientCmd(pmi::kCmdStop);
        setServoButtonState(false);
        return;
    }
    sendClientCmd(pmi::kCmdServoOn);
    setServoButtonState(true);
}

void MainWindow::onStopClicked()
{
    // Hold at current joint pose and reset the planned path on the server.
    // Torque stays ON, so don't toggle the servo button.
    sendClientCmd(pmi::kCmdHoldStop);
    m_waypointSent = false;
    updateTrajectoryButtonState(ui->btnConnect->isChecked());
    ui->labelTrajectoryStatus->setText(tr("Status: stopped (waypoints cleared, holding pose)"));
}

void MainWindow::onZeroClicked()
{
    sendClientCmd(pmi::kCmdSetZero);
}

void MainWindow::onResetClicked()
{
    sendClientCmd(pmi::kCmdResetError);
    // Server reboots all axes (~800 ms). Torque comes back OFF — reflect that locally.
    setServoButtonState(false);
}

void MainWindow::onModeCurrentClicked()
{
    sendClientCmd(pmi::kCmdModeCurrent);
}

void MainWindow::onModeVelocityClicked()
{
    sendClientCmd(pmi::kCmdModeVelocity);
}

void MainWindow::onModeExtendedPosClicked()
{
    sendClientCmd(pmi::kCmdModeExtendedPos);
}

void MainWindow::onModeCurrentBasedPosClicked()
{
    sendClientCmd(pmi::kCmdModeCurrentBasedPos);
}

bool MainWindow::sendJogVelocityCommand(double signedJointVelDegPerSec, QString *errorOut)
{
    if (!ui->btnConnect->isChecked()) {
        if (errorOut)
            *errorOut = tr("Server is not connected.");
        return false;
    }

    const int axis = ui->jogAxisCombo->currentIndex();
    if (axis < 0 || axis >= static_cast<int>(pmi::kTelemetryAxisCount)) {
        if (errorOut)
            *errorOut = tr("Invalid jog axis selection.");
        return false;
    }

    std::vector<uint8_t> payload;
    payload.reserve(9);
    payload.push_back(static_cast<uint8_t>(axis));
    std::uint64_t u = 0;
    std::memcpy(&u, &signedJointVelDegPerSec, sizeof(double));
    for (int b = 0; b < 8; ++b)
        payload.push_back(static_cast<uint8_t>((u >> (8 * b)) & 0xFFu));

    m_cmdNet->requestSend(pmi::buildClientFrame(pmi::kCmdJogVelocity, payload));
    return true;
}

void MainWindow::onJogPlusClicked()
{
    bool ok = false;
    const double current = ui->jogSpeedEdit->text().trimmed().toDouble(&ok);
    const double next = (ok ? current : 0.0) + 1.0;
    QString error;
    if (!sendJogVelocityCommand(next, &error)) {
        QMessageBox::warning(this, tr("Jog error"), error);
        return;
    }
    ui->jogSpeedEdit->setText(QString::number(next, 'f', 1));
}

void MainWindow::onJogMinusClicked()
{
    bool ok = false;
    const double current = ui->jogSpeedEdit->text().trimmed().toDouble(&ok);
    const double next = (ok ? current : 0.0) - 1.0;
    QString error;
    if (!sendJogVelocityCommand(next, &error)) {
        QMessageBox::warning(this, tr("Jog error"), error);
        return;
    }
    ui->jogSpeedEdit->setText(QString::number(next, 'f', 1));
}

void MainWindow::onJogStopClicked()
{
    QString error;
    if (!sendJogVelocityCommand(0.0, &error)) {
        QMessageBox::warning(this, tr("Jog error"), error);
        return;
    }
    ui->jogSpeedEdit->setText(QStringLiteral("0.0"));
}

bool MainWindow::parseWaypointInput(std::vector<std::array<double, 4>> &waypoints, QString &error) const
{
    waypoints.clear();
    const int rows = ui->waypointTable->rowCount();
    for (int r = 0; r < rows; ++r) {
        bool rowEmpty = true;
        for (int c = 0; c < 4; ++c) {
            const QTableWidgetItem *item = ui->waypointTable->item(r, c);
            if (item && !item->text().trimmed().isEmpty()) {
                rowEmpty = false;
                break;
            }
        }
        if (rowEmpty)
            continue;

        std::array<double, 4> wp{};
        bool ok[4] = {false, false, false, false};
        for (int k = 0; k < 4; ++k)
            wp[static_cast<size_t>(k)] =
                (ui->waypointTable->item(r, k) ? ui->waypointTable->item(r, k)->text().trimmed() : QString()).toDouble(&ok[k]);
        if (!(ok[0] && ok[1] && ok[2] && ok[3])) {
            error = tr("Row %1 contains invalid number").arg(r + 1);
            return false;
        }
        waypoints.push_back(wp);
    }

    if (waypoints.size() < 2) {
        error = tr("At least 2 waypoint rows are required.");
        return false;
    }
    for (size_t i = 1; i < waypoints.size(); ++i) {
        if (waypoints[i][0] <= waypoints[i - 1][0]) {
            error = tr("Waypoint time must be strictly increasing.");
            return false;
        }
    }
    if (waypoints.size() > 7) {
        error = tr("Current protocol supports up to 7 waypoints per packet.");
        return false;
    }
    return true;
}

std::vector<uint8_t> MainWindow::buildWaypointPayload(const std::vector<std::array<double, 4>> &waypoints) const
{
    std::vector<uint8_t> payload;
    payload.reserve(1 + waypoints.size() * 32);
    payload.push_back(static_cast<uint8_t>(waypoints.size()));
    for (const auto &wp : waypoints) {
        for (double v : wp) {
            std::uint64_t u = 0;
            std::memcpy(&u, &v, sizeof(double));
            for (int b = 0; b < 8; ++b)
                payload.push_back(static_cast<uint8_t>((u >> (8 * b)) & 0xFFu));
        }
    }
    return payload;
}

void MainWindow::onSendWaypointsClicked()
{
    std::vector<std::array<double, 4>> waypoints;
    QString error;
    if (!parseWaypointInput(waypoints, error)) {
        QMessageBox::warning(this, tr("Waypoint error"), error);
        return;
    }
    m_cmdNet->requestSend(pmi::buildClientFrame(pmi::kCmdSetWaypointBatch, buildWaypointPayload(waypoints)));
    m_waypointSent = true;
    updateTrajectoryButtonState(ui->btnConnect->isChecked());
    ui->labelTrajectoryStatus->setText(tr("Status: waypoint sent (%1)").arg(static_cast<int>(waypoints.size())));
}

void MainWindow::onPlanPathClicked()
{
    sendClientCmd(pmi::kCmdPlanPath);
    ui->labelTrajectoryStatus->setText(tr("Status: plan requested"));
}

void MainWindow::onStartTrajectoryClicked()
{
    sendClientCmd(pmi::kCmdStartTrajectoryIk);
    ui->labelTrajectoryStatus->setText(tr("Status: trajectory running"));
}

void MainWindow::onStopTrajectoryClicked()
{
    sendClientCmd(pmi::kCmdStopTrajectoryIk);
    ui->labelTrajectoryStatus->setText(tr("Status: trajectory stopped"));
}

bool MainWindow::parseInitialPose(std::array<double, 4> &jointDeg, QString &error) const
{
    const QLineEdit *edits[4] = {ui->initQ1Edit, ui->initQ2Edit, ui->initQ3Edit, ui->initQ4Edit};
    for (int i = 0; i < 4; ++i) {
        bool ok = false;
        const double v = edits[i]->text().trimmed().toDouble(&ok);
        if (!ok) {
            error = tr("q%1 is invalid. Enter degree value.").arg(i + 1);
            return false;
        }
        jointDeg[static_cast<size_t>(i)] = v;
    }
    return true;
}

void MainWindow::onSendInitialPoseClicked()
{
    std::array<double, 4> qJointDeg{};
    QString error;
    if (!parseInitialPose(qJointDeg, error)) {
        QMessageBox::warning(this, tr("Initial pose error"), error);
        return;
    }

    std::vector<uint8_t> payload;
    payload.reserve(32);
    for (double vDeg : qJointDeg) {
        const double vRad = vDeg * M_PI / 180.0;
        std::uint64_t u = 0;
        std::memcpy(&u, &vRad, sizeof(double));
        for (int b = 0; b < 8; ++b)
            payload.push_back(static_cast<uint8_t>((u >> (8 * b)) & 0xFFu));
    }
    m_cmdNet->requestSend(pmi::buildClientFrame(pmi::kCmdSetInitialJointPose, payload));
    ui->labelTrajectoryStatus->setText(tr("Status: initial pose sent"));
}

void MainWindow::onSendZeroPoseClicked()
{
    std::vector<uint8_t> payload;
    payload.reserve(32);
    for (int i = 0; i < 4; ++i) {
        const double vRad = 0.0;
        std::uint64_t u = 0;
        std::memcpy(&u, &vRad, sizeof(double));
        for (int b = 0; b < 8; ++b)
            payload.push_back(static_cast<uint8_t>((u >> (8 * b)) & 0xFFu));
    }
    m_cmdNet->requestSend(pmi::buildClientFrame(pmi::kCmdSetInitialJointPose, payload));
    ui->labelTrajectoryStatus->setText(tr("Status: zero pose sent"));
}

void MainWindow::onLogStartClicked()
{
    bool ok = false;
    const double durationSec = ui->logDurationEdit->text().trimmed().toDouble(&ok);
    if (!ok || durationSec <= 0.0) {
        QMessageBox::warning(this, tr("Logging error"), tr("Enter a valid logging duration in seconds."));
        return;
    }
    std::vector<uint8_t> payload;
    payload.reserve(8);
    std::uint64_t u = 0;
    std::memcpy(&u, &durationSec, sizeof(double));
    for (int b = 0; b < 8; ++b)
        payload.push_back(static_cast<uint8_t>((u >> (8 * b)) & 0xFFu));
    m_cmdNet->requestSend(pmi::buildClientFrame(pmi::kCmdLogStart, payload));
    startLogCountdown(durationSec);
    ui->labelLogStatus->setText(tr("Status: logging start requested (%1 s)").arg(durationSec, 0, 'f', 1));
}

void MainWindow::onLogStopClicked()
{
    sendClientCmd(pmi::kCmdLogStop);
    stopLogCountdown();
    ui->labelLogStatus->setText(tr("Status: logging stop requested"));
}

void MainWindow::startLogCountdown(double durationSec)
{
    m_logEndMs = QDateTime::currentMSecsSinceEpoch() + static_cast<qint64>(durationSec * 1000.0);
    if (m_logCountdownTimer)
        m_logCountdownTimer->start();
}

void MainWindow::stopLogCountdown()
{
    if (m_logCountdownTimer && m_logCountdownTimer->isActive())
        m_logCountdownTimer->stop();
    m_logEndMs = 0;
}
