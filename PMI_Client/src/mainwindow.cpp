// PMI client main window: Qt UI plus TcpClient; network callbacks are marshaled to the GUI thread.

#include "mainwindow.h"

#include "tcp_client.h"
#include "ui_mainwindow.h"

#include <QAbstractItemView>
#include <QByteArray>
#include <QCloseEvent>
#include <QHeaderView>
#include <QMessageBox>
#include <QSettings>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTimer>

#include <memory>
#include <utility>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(std::make_unique<Ui::MainWindow>())
    , m_net(std::make_unique<TcpClient>())
{
    ui->setupUi(this);

    m_net->setCallbacks(
        [this]() {
            QTimer::singleShot(0, this, [this]() { onNetConnected(); });
        },
        [this]() {
            QTimer::singleShot(0, this, [this]() { onNetDisconnected(); });
        },
        [this](std::vector<uint8_t> chunk) {
            onNetBytesFromWorker(std::move(chunk));
        },
        [this](std::string message) {
            const QString qmsg = QString::fromUtf8(message.data(), static_cast<int>(message.size()));
            QTimer::singleShot(0, this, [this, qmsg]() { onNetError(qmsg); });
        });
    m_net->start();

    connect(ui->btnConnect, &QPushButton::toggled, this, &MainWindow::onConnectToggled);
    connect(ui->btnServoOn, &QPushButton::clicked, this, &MainWindow::onServoOnClicked);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui->btnZero, &QPushButton::clicked, this, &MainWindow::onZeroClicked);
    connect(ui->btnCurrent, &QPushButton::clicked, this, &MainWindow::onModeCurrentClicked);
    connect(ui->btnVelocity, &QPushButton::clicked, this, &MainWindow::onModeVelocityClicked);
    connect(ui->btnPosition, &QPushButton::clicked, this, &MainWindow::onModeExtendedPosClicked);

    setUiConnected(false);
    setupTelemetryTable();

    QSettings settings;
    const QByteArray geom = settings.value(QStringLiteral("mainwindow/geometry")).toByteArray();
    if (!geom.isEmpty())
        restoreGeometry(geom);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QSettings settings;
    settings.setValue(QStringLiteral("mainwindow/geometry"), saveGeometry());
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() = default;

void MainWindow::onConnectToggled(bool checked)
{
    if (!checked) {
        m_net->requestDisconnect();
        return;
    }

    const QString host = ui->ipEdit->text().trimmed();
    bool ok = false;
    const quint16 port = ui->portEdit->text().trimmed().toUShort(&ok);

    if (host.isEmpty() || !ok || port == 0) {
        QMessageBox::warning(this, tr("Input error"), tr("Enter a valid IP address and port (1–65535)."));
        ui->btnConnect->setChecked(false);
        return;
    }

    const QByteArray hostUtf8 = host.toUtf8();
    m_net->requestConnect(std::string(hostUtf8.constData(), static_cast<size_t>(hostUtf8.size())), port);
}

void MainWindow::onNetConnected()
{
    setUiConnected(true);
    sendClientCmd(pmi::kCmdPing);
}

void MainWindow::onNetDisconnected()
{
    m_protocolRx.clear();
    clearTelemetryTable();
    setUiConnected(false);
    if (ui->btnConnect->isChecked())
        ui->btnConnect->setChecked(false);
}

void MainWindow::onNetError(const QString &message)
{
    QMessageBox::warning(this, tr("Network error"), message);
    if (ui->btnConnect->isChecked())
        ui->btnConnect->setChecked(false);
    m_protocolRx.clear();
    clearTelemetryTable();
    setUiConnected(false);
}

void MainWindow::setUiConnected(bool connected)
{
    ui->ipEdit->setEnabled(!connected);
    ui->portEdit->setEnabled(!connected);
    ui->btnConnect->setText(connected ? tr("Disconnect") : tr("Connect"));
}

void MainWindow::sendClientCmd(uint8_t cmd)
{
    if (!ui->btnConnect->isChecked())
        return;
    const std::vector<uint8_t> frame = pmi::buildClientFrame(cmd, {});
    m_net->requestSend(frame);
}

void MainWindow::onNetBytesFromWorker(std::vector<uint8_t> chunk)
{
    auto shared = std::make_shared<std::vector<uint8_t>>(std::move(chunk));
    QTimer::singleShot(0, this, [this, shared]() {
        m_protocolRx.insert(m_protocolRx.end(), shared->begin(), shared->end());
        pmi::pruneServerRxToLatestCompleteFrame(m_protocolRx);
        pmi::feedServerRxStream(m_protocolRx, [this](const pmi::ServoTelemetry axes[pmi::kTelemetryAxisCount]) {
            updateTelemetryTable(axes);
        });
    });
}

void MainWindow::setupTelemetryTable()
{
    constexpr int kCols = 11;
    m_telemetryModel = new QStandardItemModel(static_cast<int>(pmi::kTelemetryAxisCount), kCols, this);
    m_telemetryModel->setHorizontalHeaderLabels({
        tr("축"),
        tr("ID"),
        tr("Op"),
        tr("상태"),
        tr("현재 위치 (°)"),
        tr("현재 속도 (°/s)"),
        tr("현재 전류 (A)"),
        tr("목표 위치 (°)"),
        tr("목표 속도 (°/s)"),
        tr("목표 전류 (A)"),
        tr("에러"),
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
        m_telemetryModel->item(row, 1)->setText(QString::number(pmi::telemetryIdFromIdOp(t.id_op_mode)));
        m_telemetryModel->item(row, 2)->setText(QString::number(pmi::telemetryOpModeFromIdOp(t.id_op_mode)));
        m_telemetryModel->item(row, 3)->setText(QString::number(t.servo_state));
        m_telemetryModel->item(row, 4)->setText(QString::number(t.present_position, 'f', 4));
        m_telemetryModel->item(row, 5)->setText(QString::number(t.present_velocity, 'f', 4));
        m_telemetryModel->item(row, 6)->setText(QString::number(t.present_current, 'f', 4));
        m_telemetryModel->item(row, 7)->setText(QString::number(t.goal_position, 'f', 4));
        m_telemetryModel->item(row, 8)->setText(QString::number(t.goal_velocity, 'f', 4));
        m_telemetryModel->item(row, 9)->setText(QString::number(t.goal_current, 'f', 4));
        m_telemetryModel->item(row, 10)->setText(QString::number(t.error_state));
    }
}

void MainWindow::onServoOnClicked()
{
    sendClientCmd(pmi::kCmdServoOn);
}

void MainWindow::onStopClicked()
{
    sendClientCmd(pmi::kCmdStop);
}

void MainWindow::onZeroClicked()
{
    // PMI 프로토콜에 제로/홈 명령이 없으면 서버 연동 후 추가.
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
