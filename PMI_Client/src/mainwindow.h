#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// PMI (Position Marking Indicator) client — Qt Widgets UI + TCP worker.

#include <QMainWindow>

#include <array>
#include <memory>
#include <vector>

#include "pmi_protocol.h"

class QCloseEvent;
class QStandardItemModel;
class QTimer;

namespace Ui {
class MainWindow;
}

class TcpClient;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void onConnectToggled(bool checked);
    void onNetConnected();
    void onNetDisconnected();
    void onNetError(const QString &message);
    void onServoOnClicked();
    void onStopClicked();
    void onZeroClicked();
    void onResetClicked();
    void onModeCurrentClicked();
    void onModeVelocityClicked();
    void onModeExtendedPosClicked();
    void onModeCurrentBasedPosClicked();
    void onJogPlusClicked();
    void onJogMinusClicked();
    void onJogStopClicked();
    void onSendWaypointsClicked();
    void onPlanPathClicked();
    void onStartTrajectoryClicked();
    void onStopTrajectoryClicked();
    void onSendInitialPoseClicked();
    void onSendZeroPoseClicked();
    void onLogStartClicked();
    void onLogStopClicked();

private:
    void setUiConnected(bool connected);
    void sendClientCmd(uint8_t cmd);
    void onTelemetryBytesFromWorker(std::vector<uint8_t> chunk);
    void onCommandBytesFromWorker(std::vector<uint8_t> chunk);
    void updateConnectionUiState();
    void setupTelemetryTable();
    void updateTelemetryTable(const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]);
    void clearTelemetryTable();
    void setServoButtonState(bool servoOn);
    void updateTrajectoryButtonState(bool connected);
    void startLogCountdown(double durationSec);
    void stopLogCountdown();
    bool parseWaypointInput(std::vector<std::array<double, 4>> &waypoints, QString &error) const;
    std::vector<uint8_t> buildWaypointPayload(const std::vector<std::array<double, 4>> &waypoints) const;
    bool parseInitialPose(std::array<double, 4> &jointDeg, QString &error) const;
    bool sendJogVelocityCommand(double signedJointVelDegPerSec, QString *errorOut = nullptr);

    std::unique_ptr<Ui::MainWindow> ui;
    std::unique_ptr<TcpClient> m_cmdNet;
    std::unique_ptr<TcpClient> m_telemetryNet;
    QStandardItemModel *m_telemetryModel = nullptr;
    std::vector<uint8_t> m_telemetryRx;
    std::vector<uint8_t> m_commandRx;
    bool m_cmdConnected = false;
    bool m_telemetryConnected = false;
    bool m_servoOn = false;
    bool m_waypointSent = false;
    QTimer *m_logCountdownTimer = nullptr;
    qint64 m_logEndMs = 0;
};

#endif
