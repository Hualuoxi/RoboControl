#ifndef CLINET_H
#define CLINET_H
#include <QTcpServer>
#include <QTcpSocket>
#include <QHostAddress>
#include <QThread>
#include <QTimer>
#include "Utilities.h"
class Clinet :public QObject
{
    Q_OBJECT
public:
    Clinet(QObject *parent = 0, quint16 port=20200);
private:
    QTcpServer *mTcpServer;
    QTcpSocket *mTcpSocket;
signals:
    void shutDownApp();
    void StartRun_sig(int speed_, char dir_1, char dir_2);
    void StopRun_sig();
    void StartCoarsen_sig(int speed, char direction);
    void StopCoarsen_sig();
    void turn_sig(int turnDegree);
public slots:
    void newConnection_slot();
    void RcvClinet_slot();
    void TranClinet_slot(RobotFadebck_frame* frame);

};

#endif // CLINET_H
