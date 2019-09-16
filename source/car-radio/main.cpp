#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlEngine>
#include <QQmlContext>
#include <QQuickView>
#include <QQuickItem>
#include <QDir>
#include <QFileInfo>
#include <QDebug>

#include "SystemCommand.h"
#include "AudioControl.h"
#include "FileUtil.h"
#include "TunerControl.h"
#include "GnssPosition.h"
#include "TcpClient.h"
#include "BluetoothControl.h"
//#include "CanControl.h"
//#include "NvMemory.h"

int main(int argc, char *argv[])
{
    QLoggingCategory::setFilterRules(QStringLiteral("qt.bluetooth.bluez = false"));
    QLoggingCategory::setFilterRules(QStringLiteral("qt.bluetooth.qml = true"));

    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
    QGuiApplication application(argc, argv);
    /* open a new window */
    QQuickView *view = new QQuickView(NULL);
    QQmlEngine *engine = view->engine(); //new QQmlEngine;

    QObject::connect(engine, SIGNAL(quit()), qApp, SLOT(quit()));
    /* the same like qApp */
//  QObject::connect(engine, SIGNAL(quit()), QCoreApplication::instance(), SLOT(quit()));

    view->setResizeMode(QQuickView::SizeRootObjectToView);

    /* register cpp source to qml */
    qmlRegisterType<FileUtil,1>     ("AEC", 1, 0, "FileUtil");
    qmlRegisterType<SystemCommand,1>("AEC", 1, 0, "SystemCommand");
    qmlRegisterType<TunerControl,1> ("AEC", 1, 0, "TunerControl");
    qmlRegisterType<RdsThread,1>    ("AEC", 1, 0, "RdsThread");
    qmlRegisterType<AudioControl,1> ("AEC", 1, 0, "AudioControl");
    qmlRegisterType<GnssPosition,1> ("AEC", 1, 0, "GnssPosition");
    qmlRegisterType<TcpClient,1>    ("AEC", 1, 0, "TcpClient");
    qmlRegisterType<BluetoothControl,1>    ("AEC", 1, 0, "BluetoothControl");
//    qmlRegisterType<CanControl,1>   ("AEC", 1, 0, "CanControl");
//    qmlRegisterType<NvMemory,1>     ("AEC", 1, 0, "NvMemory");

#ifdef TARGET_MODE
#define APP_PATH "/home/pi/car-radio/"
#else
#define APP_PATH "./"
#endif

    /* load qml data/applications */
    QDir dir(QLatin1String(APP_PATH) + QLatin1String("qml/dummydata"), "*.qml");
    qDebug() << "QmlAppviewLoad: " << dir.absolutePath() << " " << dir.count();

    QStringList list = dir.entryList();
    for (int i = 0; i < list.size(); ++i) {
        QString qml = list.at(i);
        qDebug() << "QmlAppviewLoad" << i <<":" << qml;

        QQmlComponent comp(engine, dir.filePath(qml));
        QObject *dummyData = comp.create();

        if (comp.isError()) {
                qWarning("Error when loading dummydata ");
        }
        if (dummyData) {
            qml.truncate(qml.length()-4); // remove .qml

            qDebug() << "QmlAppviewLoad" << i <<": qml " << qml;

            engine->rootContext()->setContextProperty(qml, dummyData);
            dummyData->setParent(view);
        }
    }
    /* start main application */
    view->setSource(QUrl(QLatin1String(APP_PATH) + QLatin1String("qml/CarMMI.qml")));
    view->setMouseGrabEnabled(true);
    view->show();

    return application.exec();
}
