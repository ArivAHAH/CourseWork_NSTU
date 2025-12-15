#include <QApplication>
#include "mainwindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QApplication::setApplicationName("Симуляция трафика Новосибирска");
    QApplication::setOrganizationName("NSTU");

    MainWindow w;
    w.resize(1100, 700);
    w.show();

    return app.exec();
}
