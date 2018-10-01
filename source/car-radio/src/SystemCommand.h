#ifndef SYSTEMCOMANND_H
#define SYSTEMCOMANND_H

#include <QObject>

class SystemCommand : public QObject
{
    Q_OBJECT

public:

    SystemCommand();
    virtual ~SystemCommand();

    Q_INVOKABLE void invoke(const QString& command);
    Q_INVOKABLE void redraw();
};

#endif // SYSTEMCOMANND_H
