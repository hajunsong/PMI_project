#include "mainwindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QSocketNotifier>

#include <array>
#include <csignal>
#include <cstdint>

#include <unistd.h>

// Entry point: Qt event loop and main window (PMI = Position Marking Indicator).

namespace {

std::array<int, 2> g_signalPipe = {-1, -1};

void signalHandler(int)
{
    const std::uint8_t byte = 1;
    if (g_signalPipe[1] >= 0) {
        const ssize_t written = ::write(g_signalPipe[1], &byte, sizeof(byte));
        (void)written;
    }
}

} // namespace

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("PMI_client"));
    QApplication::setOrganizationName(QStringLiteral("PMI"));

    if (::pipe(g_signalPipe.data()) == 0) {
        auto *notifier = new QSocketNotifier(g_signalPipe[0], QSocketNotifier::Read, &app);
        QObject::connect(notifier, &QSocketNotifier::activated, &app, [notifier]() {
            notifier->setEnabled(false);
            std::uint8_t drain = 0;
            (void)::read(g_signalPipe[0], &drain, sizeof(drain));
            QCoreApplication::quit();
        });
        QObject::connect(&app, &QCoreApplication::aboutToQuit, &app, []() {
            if (g_signalPipe[0] >= 0) {
                ::close(g_signalPipe[0]);
                g_signalPipe[0] = -1;
            }
            if (g_signalPipe[1] >= 0) {
                ::close(g_signalPipe[1]);
                g_signalPipe[1] = -1;
            }
        });
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
    }

    MainWindow window;
    window.show();

    return app.exec();
}
