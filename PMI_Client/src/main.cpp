#include "mainwindow.h"

#include <QApplication>

// Entry point: Qt event loop and main window (PMI = Position Marking Indicator).

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("PMI_client"));
    QApplication::setOrganizationName(QStringLiteral("PMI"));

    MainWindow window;
    window.show();

    return app.exec();
}
