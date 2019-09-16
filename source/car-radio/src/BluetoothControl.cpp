/*
 *  buetooth.cpp
 *
 *  control of bluetooth device without wigdets
 *  connected to qml interface
 *
 *  Created on: 02.06.2018
 *      Author: hj.arlt@online.de
 */
#include <QDebug>

#include "BluetoothControl.h"

/* ********************************************************************
 * class bluetoothControl
 * - control bluetooth device
 ********************************************************************** */

BluetoothControl::BluetoothControl()
    : m_command(), localDevice(new QBluetoothLocalDevice)
{
    qDebug() << "BtCtrl open..";

    localDevice->powerOn();

    connect(localDevice, SIGNAL(pairingDisplayPinCode(QBluetoothAddress,QString)),
                     this, SLOT(displayPinCode(QBluetoothAddress,QString)));
    connect(localDevice, SIGNAL(pairingDisplayConfirmation(QBluetoothAddress,QString)),
                    this, SLOT(displayParingConfirmation(QBluetoothAddress,QString)));
    connect(localDevice, SIGNAL(error(QBluetoothLocalDevice::Error)),
                    this, SLOT(pairingError(QBluetoothLocalDevice::Error)));

    discoveryAgent = new QBluetoothDeviceDiscoveryAgent();

    connect(discoveryAgent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)),
                      this, SLOT(addDevice(QBluetoothDeviceInfo)));
    connect(discoveryAgent, SIGNAL(finished()),
                      this, SLOT(scanDeviceFinished()));

    discoverySrvAgent = new QBluetoothServiceDiscoveryAgent(this);

    connect(discoverySrvAgent, SIGNAL(serviceDiscovered(QBluetoothServiceInfo)),
                         this, SLOT(addService(QBluetoothServiceInfo)));
    connect(discoverySrvAgent, SIGNAL(finished()),
                         this, SLOT(scanServiceFinished()));
}

BluetoothControl::~BluetoothControl()
{
    delete discoveryAgent;
    delete discoverySrvAgent;

    qDebug() << "BtCtrl closed";
}

void BluetoothControl::startScan()
{
    m_devices = 0;
    memset(m_device, 0, sizeof(device_t) * MAX_DEVICES);

    myDevice = "";
    myAddress = "";
    activeDevice = -1;

    if (discoveryAgent)
        discoveryAgent->start();

    qDebug() << "BtCtrl scan started..";
}

void BluetoothControl::stopScan()
{
    discoveryAgent->stop();
}

/*
 *  selected device
 */
void BluetoothControl::connectDevice(int idx)
{
    qDebug() << "BtCtrl: connect device" << getDevice(idx);

    activeDevice = idx;
    myDevice = getDevice(idx);
    myAddress = getAddress(idx);

    QBluetoothAddress address(myAddress);

    discoverySrvAgent->setRemoteAddress(address);
    discoverySrvAgent->start();
}

/*
 * pair device
 */
void BluetoothControl::pairingDevice()
{
    qDebug() << "BtCtrl: pairing " << myAddress;

    QBluetoothAddress address(myAddress);
    localDevice->requestPairing(address, QBluetoothLocalDevice::Paired);
}

void BluetoothControl::pairingError(QBluetoothLocalDevice::Error error)
{
    if (error != QBluetoothLocalDevice::PairingError)
        return;

    qDebug() << "BtCtrl pairing error " << myAddress;
}

void BluetoothControl::displayPinCode(const QBluetoothAddress &address, QString code)
{
    qDebug() << "BtCtrl: display " << address << " pin code " << code;
    localDevice->pairingConfirmation(true);
}

void BluetoothControl::displayParingConfirmation(const QBluetoothAddress &address, QString code)
{
    qDebug() << "BtCtrl: confirm " << address << " pin code " << code;
    localDevice->pairingConfirmation(true);
}

int BluetoothControl::deviceFound()
{
    return (m_devices);
}

bool BluetoothControl::getPaired(int idx)
{
    return(m_device[idx].paired);
}

QString BluetoothControl::getDevice(int idx)
{
    return QString(m_device[idx].name);
}

QString BluetoothControl::getAddress(int idx)
{
    return QString(m_device[idx].address);
}

void BluetoothControl::addDevice(const QBluetoothDeviceInfo &info)
{
    qDebug() << "BtCtrl: add device" << info.address() << info.name();
    newDevice(info.name(),info.address().toString(), "");
}

void BluetoothControl::addService(const QBluetoothServiceInfo &info)
{
    if (info.serviceName().isEmpty())
        return;

    //qDebug() << "BtCtrl: add service" << info.serviceName();
    newDevice(myDevice, myAddress, info.serviceName());
}

void BluetoothControl::scanDeviceFinished()
{
    qDebug() << "BtCtrl: scan device finished";
    emit scanFinished();
/*
    myDevice = "hja F5";
    myAddress = "00:73:8D:0C:99:10";

    QBluetoothAddress address(myAddress);

    discoverySrvAgent->setRemoteAddress(address);
    discoverySrvAgent->start();
*/
}

void BluetoothControl::scanServiceFinished()
{
    qDebug() << "BtCtrl: scan service finished";
    emit scanCompleted();
}

/*
 * QML source
 */
int BluetoothControl::newDevice(QString device, QString address, QString service)
{
    int i;
    qDebug() << "BtCtrl: new device" << device << address << service;

    for (i=0; i<MAX_DEVICES; i++) {
        if (strcmp(m_device[i].name,  device.toStdString().c_str()) == 0) {
            break;
        }
    }
    /* new device */
    if (i >= MAX_DEVICES) {
        for (i=0; i<MAX_DEVICES; i++) {
            if (m_device[i].name[0] == 0) {
                m_devices++;
                break;
            }
        }
    }
    /* list full */
    if (i >= MAX_DEVICES) {
        return -1;
    }
    /* device */
    strcpy(m_device[i].name, device.toStdString().c_str());
    strcpy(m_device[i].address, address.toStdString().c_str());
    m_device[i].paired = localDevice->pairingStatus(QBluetoothAddress(address));

    qDebug() << "BtCtrl: add device" << i << device << " paired " << m_device[i].paired;

    if (service.isEmpty()) {
        return 0;
    }
    /* services */
    strcpy(m_device[i].service[m_device[i].services].name, service.toStdString().c_str());
    m_device[i].service[m_device[i].services].registered = false;
    if (m_device[i].services < (MAX_SERVICES-1))
        m_device[i].services++;

    return 0;
}

int BluetoothControl::printDevice()
{
    int i, j;
    qDebug() << "BtCtrl: ### DEVICES ###";
    for (i=0; i<m_devices; i++) {
        qDebug() << "BtCtrl: " << i << "dev" << m_device[i].name
                                    << "addr" << m_device[i].address
                                    << " paired " << m_device[i].paired;
        for (j=0; j<m_device[i].services; j++) {
            qDebug() << "BtCtrl:        srv" << j << m_device[i].service[j].name
                                            << "reg"  << m_device[i].service[j].registered;
        }
    }

    return m_devices;
}

int BluetoothControl::callNumber(QString number)
{
    const QString m_path = "/hfp/org/bluez/hci0/dev_1234";

    QDBusConnection::systemBus().connect("org.ofono", m_path, "org.ofono.VoiceCallManager"
                                         , "CallAdded", this, SLOT(callAdded(QString,QMap<QString,QVariant>)));
    return 0;
}

void BluetoothControl::callAdded(QString, QMap<QString,QVariant>)
{

}

int BluetoothControl::hangUp()
{
    return 0;
}
