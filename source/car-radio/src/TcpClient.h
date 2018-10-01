#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QObject>
#include <QtNetwork>
#include <QNetworkRequest>

class TcpClient : public QObject
{
    Q_OBJECT

    QTcpSocket *m_socket;
    bool m_error;

public:

    TcpClient();
    virtual ~TcpClient();

    Q_INVOKABLE bool checkConnectToWeb();
    Q_INVOKABLE void connectToHost(const QString& address, int port);
    Q_INVOKABLE bool connectToHostSync(const QString& address, int port);
    Q_INVOKABLE void disconnectFromHost();
    Q_INVOKABLE bool isConnected();
    Q_INVOKABLE void send(const QString& message);

signals:

    void connected();
    void disconnected();
    void messageAvailable(const QString& message);

private slots:

    void socketConnected();
    void socketDisconnected();
    void readyRead();
};

#endif // TCPCLIENT_H
