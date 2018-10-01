#ifndef TUNERCONTROL_H
#define TUNERCONTROL_H

#include <QObject>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include "FileUtil.h"


class RdsThread : public QThread
{
    Q_OBJECT

public:
    QMutex mutex;

signals:
    void rdsAvailable();
    void tmsAvailable();

public slots:

private:
    void run();
};

class TunerControl : public QObject
{
    Q_OBJECT

    QTimer m_timer;
    FileUtil m_fileutil;
    RdsThread rdsThread;

    int next(int dev, bool up);
    int setThreshold(int dev, int thres_bb, int thres_det, int thres_adj);
    QString getStationString(int entry);

public:

    TunerControl();
    virtual ~TunerControl();

    Q_INVOKABLE int      tune(int dev, const QString &band, const QString &f);
    Q_INVOKABLE int      scan(int dev, const QString &band);
    Q_INVOKABLE void     clearstations();
    Q_INVOKABLE int      nextup();
    Q_INVOKABLE int      nextdown();
    Q_INVOKABLE void     mute();
    Q_INVOKABLE void     unmute();
    Q_INVOKABLE void     volume(double volume);
    Q_INVOKABLE int      getactualindex(void);
    Q_INVOKABLE int      getquality(int dev);

    /*
     * get rds text line 0=actual, 1=last
     */
    Q_INVOKABLE QString  getrdstext(int line);
    /*
     * get actual station logo from rds pi list
     */
    Q_INVOKABLE QString  getstationlogo();
    Q_INVOKABLE QString  getnamelogo(QString station);

    /*
     * get actual rds station name from pi list
     */
    Q_INVOKABLE int      getrdspi();
    Q_INVOKABLE QString  getrdsname();
    Q_INVOKABLE int      storestationlist(QString filename);
    Q_INVOKABLE int      qstring2station(QString line, int entry);
    Q_INVOKABLE int      gettmsstatus();

signals:

    void unlock();

private slots:

    void rdsReceived();
    void tmsReceived();
};


#endif // TUNERCONTROL_H
