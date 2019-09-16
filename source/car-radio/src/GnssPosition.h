#ifndef POSITIONING_H
#define POSITIONING_H

#include <QtCore>
#include <QtQuick>

#include <QAbstractListModel>
#include <QSet>
#include <QtQml/qqml.h>
#include <QtQml/QQmlParserStatus>
#include <QNmeaPositionInfoSource>
#include <QGeoSatelliteInfoSource>
#include <QIODevice>
#include <QSerialPort>

class GnssPosition : public QObject
{
    Q_OBJECT

public:

    GnssPosition();
    virtual ~GnssPosition();

    /* QML or public slot */
    Q_INVOKABLE bool isConnected();

signals:

    void newPosition(double lat, double lon, double alt, double dir, double speed);
    void entryCountChanged();

private:

    QNmeaPositionInfoSource *source;
    QSerialPort *m_serial;
    QGeoSatelliteInfoSource *m_satinfo;
    QList <QGeoSatelliteInfo> knownSatellites;
    QSet<int> knownSatelliteIds;
    QSet<int> satellitesInUse;

private slots:

    void positionUpdated(const QGeoPositionInfo &info);
    void satellitesInViewUpdated(const QList<QGeoSatelliteInfo> &infos);
    void satellitesInUseUpdated(const QList<QGeoSatelliteInfo> &infos);

};

#endif // POSITIONING_H
