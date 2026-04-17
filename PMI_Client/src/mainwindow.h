#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// PMI (Position Marking Indicator) client — Qt Widgets UI + TCP worker.

#include <QMainWindow>

#include <memory>
#include <vector>

#include "pmi_protocol.h"

class QCloseEvent;
class QStandardItemModel;

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
    void onModeCurrentClicked();
    void onModeVelocityClicked();
    void onModeExtendedPosClicked();

private:
    void setUiConnected(bool connected);
    void sendClientCmd(uint8_t cmd);
    void onNetBytesFromWorker(std::vector<uint8_t> chunk);
    void setupTelemetryTable();
    void updateTelemetryTable(const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]);
    void clearTelemetryTable();

    std::unique_ptr<Ui::MainWindow> ui;
    std::unique_ptr<TcpClient> m_net;
    QStandardItemModel *m_telemetryModel = nullptr;
    std::vector<uint8_t> m_protocolRx;
};

#endif
