
#include "GnssPosition.h"
#include <QDebug>

GnssPosition::GnssPosition()
{
    const QString gnssdevice = "/dev/ttyUSB0";
    const QString gnssdevice2 = "/dev/ttyACM0";

    QString isgnss = gnssdevice;
    source = NULL;
    m_serial = new QSerialPort(this);
    m_serial->setPortName(gnssdevice);
    if (! m_serial->open(QIODevice::ReadWrite)) {
        delete(m_serial);

        m_serial = new QSerialPort(this);
        m_serial->setPortName(gnssdevice2);
        isgnss = gnssdevice2;
        if (! m_serial->open(QIODevice::ReadWrite)) {
            qDebug() << "GnssPosition: NO devive found!";
            return;
        }
    }
    qDebug() << "GnssPosition: devive" << isgnss << " open";

    if (gnssdevice == "/dev/ttyUSB0")
          m_serial->setBaudRate(QSerialPort::Baud4800);
    else  m_serial->setBaudRate(QSerialPort::Baud115200);

    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);

    source = new QNmeaPositionInfoSource(QNmeaPositionInfoSource::RealTimeMode, this);
    if (source) {
        source->setDevice((QIODevice*)m_serial);
        source->setUpdateInterval(1000);
        connect(source, SIGNAL(positionUpdated(QGeoPositionInfo)),
                  this, SLOT(positionUpdated(QGeoPositionInfo)));
        source->startUpdates();
#ifdef SAT_INFO
        /* geoclues not usable */
        m_satinfo = QGeoSatelliteInfoSource::createDefaultSource(this);
        if (m_satinfo) {
            QStringList sourcesList = QGeoSatelliteInfoSource::availableSources();
            qDebug() << "Satellites sources count: " << sourcesList.count();
            foreach (const QString &src, sourcesList) {
               qDebug() << "source in list: " << src;
            }

            qDebug() << "GnssPosition: sat info from " << m_satinfo->sourceName();

            m_satinfo->setUpdateInterval(3000);
            connect(m_satinfo, SIGNAL(satellitesInViewUpdated(QList<QGeoSatelliteInfo>)),
                     this, SLOT(satellitesInViewUpdated(QList<QGeoSatelliteInfo>)));
            connect(m_satinfo, SIGNAL(satellitesInUseUpdated(QList<QGeoSatelliteInfo>)),
                     this, SLOT(satellitesInUseUpdated(QList<QGeoSatelliteInfo>)));
            m_satinfo->startUpdates();
        } else {
            qDebug() << "GnssPosition: NO sat info source!";
        }
#endif
    }
    if (! source) {
        qDebug() << "GnssPosition: NO devive found!";
    }
}

GnssPosition::~GnssPosition()
{
    if (m_satinfo) m_satinfo->stopUpdates();
    if (source)    source->stopUpdates();
}

bool  GnssPosition::isConnected()
{
    if (! source)
        return false;
    return true;
}

void GnssPosition::positionUpdated(const QGeoPositionInfo &info)
{
//  qDebug() << "Position Sat updated:" << info;
    emit newPosition(info.coordinate().latitude()
                   , info.coordinate().longitude()
                   , info.coordinate().altitude());
}

void GnssPosition::satellitesInViewUpdated(const QList<QGeoSatelliteInfo> &infos)
{
    if (!m_satinfo)
        return;

    int oldEntryCount = knownSatellites.count();

    qDebug() << "GnssPosition: sat in view " << oldEntryCount;

    QSet<int> satelliteIdsInUpdate;
    foreach (const QGeoSatelliteInfo &info, infos)
        satelliteIdsInUpdate.insert(info.satelliteIdentifier());

    QSet<int> toBeRemoved = knownSatelliteIds - satelliteIdsInUpdate;

    //We reset the model as in reality just about all entry values change
    //and there are generally a lot of inserts and removals each time
    //Hence we don't bother with complex model update logic beyond resetModel()

    //beginResetModel();

    knownSatellites = infos;

    //sort them for presentation purposes
    //std::sort(knownSatellites.begin(), knownSatellites.end());

    //remove old "InUse" data
    //new satellites are by default not in "InUse"
    //existing satellites keep their "inUse" state
    satellitesInUse -= toBeRemoved;

    knownSatelliteIds = satelliteIdsInUpdate;
    // endResetModel();

    if (oldEntryCount != knownSatellites.count()) {
        qDebug() << "GnssPosition: sat in use " << satellitesInUse.count() << " known " + knownSatellites.count();

        emit entryCountChanged();
    }
}

void GnssPosition::satellitesInUseUpdated(const QList<QGeoSatelliteInfo> &infos)
{
    if (!m_satinfo)
        return;

    //beginResetModel();

    int cnt = 0;
    satellitesInUse.clear();
    foreach (const QGeoSatelliteInfo &info, infos) {
        satellitesInUse.insert(info.satelliteIdentifier());
        cnt++;
    }

    qDebug() << "GnssPosition: sat in use " << cnt;

    //endResetModel();
}
