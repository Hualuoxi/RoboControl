#include "clinet.h"

Clinet::Clinet(QObject *parent,quint16 port)
{
    //监听套接字，指定父对象，让其自动回收空间
    mTcpServer = new QTcpServer(this);
    mTcpServer->listen(QHostAddress::Any, port);
    connect(mTcpServer,&QTcpServer::newConnection,this,&Clinet::newConnection_slot);

}

void Clinet::newConnection_slot()
{
    //取出建立好连接的套接字
    mTcpSocket = mTcpServer->nextPendingConnection();

    //获取对方的IP和端口
    QString ip = mTcpSocket->peerAddress().toString();
    qint16 port = mTcpSocket->peerPort();
    QString str = QString("[%1:%2]:successfully connect").arg(ip).arg(port);
    qDebug()<<str;
    connect(mTcpSocket,&QTcpSocket::readyRead,this,&Clinet::RcvClinet_slot);

}

void Clinet::RcvClinet_slot()
{
    QByteArray array = mTcpSocket->read(sizeof(CMD_frame));

}

void Clinet::TranClinet_slot(QString str)
{
    mTcpSocket->write(str.toUtf8().data());
}

