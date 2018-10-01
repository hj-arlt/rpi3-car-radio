#include <QDebug>

#include "TcpClient.h"

TcpClient::TcpClient()
{
    qDebug() << "TcpClient open..";
    m_socket = new QTcpSocket(this);
    if (! m_socket) {
        qDebug() << "TcpClient ERR socket!";
        m_error = true;
        return;
    }
    m_error = false;
    connect(m_socket, SIGNAL(connected()), this, SLOT(socketConnected()));
    connect(m_socket, SIGNAL(disconnected()), this, SLOT(socketDisconnected()));
}

TcpClient::~TcpClient()
{

}

/* not realy cool */
bool TcpClient::checkConnectToWeb()
{
    QNetworkAccessManager nam;
    QNetworkRequest req(QUrl("www.google.com"));
    QNetworkReply *reply = nam.get(req);
    QEventLoop loop;
    connect(reply, SIGNAL(finished()), &loop, SLOT(quit()));
    loop.exec();
    if (reply->bytesAvailable()) {
        qDebug() << "You are connected to the internet";
        return true;
    }
    qDebug() << "You are not connected to the internet";
    return false;
}

void TcpClient::connectToHost(const QString &address, int port)
{
    if (m_error)
        return;
    m_socket->connectToHost(address, port);
}

bool TcpClient::connectToHostSync(const QString& address, int port)
{
    if (m_error)
        return false;

    qDebug() << "TcpClient connect to " << address << " port " << port;

    connectToHost(address, port);
    return (m_socket->waitForConnected(5000));
}

void TcpClient::disconnectFromHost()
{
    m_socket->disconnectFromHost();
}

bool TcpClient::isConnected()
{
    if (m_error)
        return false;

    return (m_socket->state() == QAbstractSocket::ConnectedState);
}

void TcpClient::send(const QString& message)
{
    if (m_error)
        return;

    qDebug() << "TcpClient send message " << message << "\n";
    if ((m_socket->isWritable()) && (m_socket->state() == QAbstractSocket::ConnectedState))
    {
        QByteArray buffer = message.toUtf8();
        buffer.push_back('\0');
        m_socket->write(buffer);
        m_socket->flush();
    }
}

void TcpClient::socketConnected()
{
    qDebug() << "TcpClient connected\n";
    connect(m_socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
    emit connected();
}

void TcpClient::socketDisconnected()
{
    qDebug() << "TcpClient disconnected\n";
    emit disconnected();
}

void TcpClient::readyRead()
{
    qDebug() << "TcpClient has " << m_socket->bytesAvailable() << " bytes available to read\n";

    if (m_socket->bytesAvailable() > 0)
    {
        QString message = QString::fromUtf8(m_socket->readAll());
        emit messageAvailable(message);
    }
}


