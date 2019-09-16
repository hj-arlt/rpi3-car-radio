#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <string.h>
#include <QObject>
#include <QString>
//#include <QFile>
//#include <QTextStream>
//#include <QTimer>

#include <qbluetoothglobal.h>
#include <qbluetoothaddress.h>
#include <qbluetoothdevicediscoveryagent.h>
#include <qbluetoothservicediscoveryagent.h>
#include <qbluetoothlocaldevice.h>
#include <qbluetoothserviceinfo.h>
#include <qbluetoothuuid.h>
#include <qbluetoothtransferrequest.h>
#include <qbluetoothtransferreply.h>
#include <QDBusConnection>
#include "SystemCommand.h"

class BluetoothControl : public QObject
{
    Q_OBJECT

#define MAX_DEVICES   16
#define MAX_SERVICES  16

    typedef struct {
        char name[64];
        bool registered;
    } service_t;

    typedef struct {
        char address[64];
        char name[64];
        bool paired;
        int services;
        service_t service[MAX_SERVICES];
    } device_t;

    SystemCommand         m_command;
    int                   m_devices;
    device_t              m_device[MAX_DEVICES];
    QString               myDevice;
    QString               myAddress;
    int                   activeDevice;

    QBluetoothLocalDevice          *localDevice;
    QBluetoothDeviceDiscoveryAgent *discoveryAgent;
    QBluetoothServiceDiscoveryAgent *discoverySrvAgent;

public:

    BluetoothControl();
    virtual ~BluetoothControl();

    Q_INVOKABLE void startScan();
    Q_INVOKABLE void stopScan();
    Q_INVOKABLE int  newDevice(QString device, QString address, QString service);
    Q_INVOKABLE int  printDevice();
    Q_INVOKABLE int  deviceFound();
    Q_INVOKABLE bool getPaired(int idx);
    Q_INVOKABLE QString getDevice(int idx);
    Q_INVOKABLE QString getAddress(int idx);
    Q_INVOKABLE void connectDevice(int idx);
    Q_INVOKABLE void pairingDevice();
    Q_INVOKABLE int callNumber(QString number);
    Q_INVOKABLE int hangUp();

signals:

    void scanFinished();
    void scanCompleted();

private slots:

    void addDevice(const QBluetoothDeviceInfo& info);
    void addService(const QBluetoothServiceInfo &info);
    void scanDeviceFinished();
    void scanServiceFinished();
    void displayPinCode(const QBluetoothAddress &address, QString code);
    void displayParingConfirmation(const QBluetoothAddress &address, QString code);
    void pairingError(QBluetoothLocalDevice::Error error);
    void callAdded(QString, QMap<QString,QVariant>);
};

#endif // BLUETOOTH_H
