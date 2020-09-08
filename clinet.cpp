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
    mTcpSocket->write(str.toUtf8().data());
    connect(mTcpSocket,&QTcpSocket::readyRead,this,&Clinet::RcvClinet_slot);
}

void Clinet::RcvClinet_slot()
{
    QByteArray array = mTcpSocket->read(sizeof(CMD_frame));
    CMD_frame *cmdInfo = (CMD_frame*)array.data();
    if('A' == cmdInfo->header[0] && 'B' == cmdInfo->header[1]){
            if('B' == cmdInfo->cmd[0] && 'G' == cmdInfo->cmd[1]){
                qDebug() << "机器人开始工作了。" ;
                emit StartRun_sig(cmdInfo->move_sp, cmdInfo->cmd_para[1], cmdInfo->cmd_para[2]);
                QThread::sleep(1);
                emit StartCoarsen_sig(cmdInfo->coasen_sp, cmdInfo->cmd_para[0]);

            } else if('S' == cmdInfo->cmd[0] && 'T' == cmdInfo->cmd[1]){
                qDebug() << "Robot Stop " << endl;
                emit StopCoarsen_sig();
                emit StopRun_sig();

            } else if('M' == cmdInfo->cmd[0] && 'B' == cmdInfo->cmd[1]){
                qDebug() << "Robot Start Run" << cmdInfo->move_sp << endl;
                emit StartRun_sig(cmdInfo->move_sp, cmdInfo->cmd_para[1], cmdInfo->cmd_para[2]);

            } else if('M' == cmdInfo->cmd[0] && 'S' == cmdInfo->cmd[1]){
                qDebug() << "Robot Stop Run" << endl;
                emit StopRun_sig();

            } else if('C' == cmdInfo->cmd[0] && 'B' == cmdInfo->cmd[1]){
                qDebug() << "Robot Start Coasen" << endl;
                emit StartCoarsen_sig(cmdInfo->coasen_sp, cmdInfo->cmd_para[0]);

            } else if('C' == cmdInfo->cmd[0] && 'S' == cmdInfo->cmd[1]){
                qDebug() << "Robot Stop Coasen" << endl;
                emit StopCoarsen_sig();

            } else if('T' == cmdInfo->cmd[0] && ('R' == cmdInfo->cmd[1] || 'L' == cmdInfo->cmd[1])){ // R > 0; L < 0
                if('R' == cmdInfo->cmd[1]) qDebug() << "Robot Turn Right。" << endl;
                else if('L' == cmdInfo->cmd[1]) qDebug() << "Robot Turn Left" << endl;
                int turnDegree = cmdInfo->turn_pa;
                emit turn_sig(turnDegree);

            } else if('S' == cmdInfo->cmd[0] && 'D' == cmdInfo->cmd[1]){
                emit shutDownApp();
                qDebug() << "shutdown application." << endl;
            }
        }
}

void Clinet::TranClinet_slot(RobotFadebck_frame* frame)
{
    mTcpSocket->write((char*)frame);
}

