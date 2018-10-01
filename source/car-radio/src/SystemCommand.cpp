#include "SystemCommand.h"

#include <QDebug>
#include <QWindow>
#include <QGuiApplication>

#include <time.h>

SystemCommand::SystemCommand()
{
}

SystemCommand::~SystemCommand()
{
}

void SystemCommand::invoke(const QString &command)
{
    qDebug() << "SystemCommand:invoke ->" << command.toStdString().c_str() << "\n";

    int ret = system(command.toStdString().c_str());
    ret = ret;
}

void SystemCommand::redraw()
{
    struct timespec ts = { 1, 0 };

    nanosleep(&ts, NULL);
#if 0
    QWindow* activeWindow = QCoreApplication::;

    if (NULL != activeWindow)
    {
        QSize newSize = activeWindow->size();
        newSize.setHeight(newSize.height() - 1);
        newSize.setWidth(newSize.width() - 1);
        activeWindow->resize(newSize);
        activeWindow->repaint();
        newSize.setHeight(newSize.height() + 1);
        newSize.setWidth(newSize.width() + 1);
        activeWindow->resize(newSize);
        activeWindow->repaint();
    }
#endif
}
