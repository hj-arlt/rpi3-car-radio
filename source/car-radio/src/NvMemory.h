/*
 * NvMemory.h
 *
 *  Created on: 28.04.2017
 *      Author: hj.arlt@online.de
 */
#ifndef NVMEMORY_H
#define NVMEMORY_H

#include <QObject>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include "FileUtil.h"

class NvMemory : public QObject
{
    Q_OBJECT

    FileUtil m_fileutil;

    QMutex m_mutex;
    bool m_found;

public:

    NvMemory();
    virtual ~NvMemory();

    enum {
        TAG_UNKNOWN = 0,
        TAG_VOLUME_CAPTURE,
        TAG_VOLUME_PLAYBACK,
        TAG_STATION_T1,
        TAG_STATION_T2,
        TAG_TITLE_AUDIO,
        TAG_POS_AUDIO,
        TAG_TITLE_VIDEO,
        TAG_POS_VIDEO,
    };

    Q_INVOKABLE int readEeprom(int addr, quint8 *pdata, int len);
    Q_INVOKABLE int writeEeprom(int addr, quint8 *pdata, int len);
    Q_INVOKABLE int getEeprom(int tag, int *value);
    Q_INVOKABLE int setEeprom(int tag, int value);

signals:

private slots:

};

#endif // NVMEMORY_H
