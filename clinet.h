#ifndef CLINET_H
#define CLINET_H
#include <QTcpServer>
#include <QTcpSocket>
#include <QHostAddress>
#include <QThread>
#include "Utilities.h"
class Clinet :public QObject
{
    Q_OBJECT
public:
    Clinet(QObject *parent = 0, quint16 port=20020);
private:
    QTcpServer *mTcpServer;
    QTcpSocket *mTcpSocket;
public slots:
    void newConnection_slot();
    void RcvClinet_slot();
    void TranClinet_slot(QString str);
};

#endif // CLINET_H
