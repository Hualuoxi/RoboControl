#ifndef USART_H
#define USART_H
#include <QObject>
#include <QDebug>
#include <QThread>
#include "Utilities.h"
#include <unistd.h>       /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>        /*文件控制定义*/
#include <errno.h>        /*错误号定义*/
#include <termios.h>      /*POSIX 终端控制定义*/
#include <QVector>
class USART_RcvThread :public QObject
{
    Q_OBJECT
public:
    USART_RcvThread(QObject *parent = 0) ;
public slots:
    void RcvMegThread(int fd);
    void stopRcvThread();
private:
    bool isStop=false;
signals:
    void sendGyroDate_signal(QVector<u8>a);
};


class USART:public QObject
{
    Q_OBJECT

#define usart1 "/dev/ttyS1"
enum USART_BAUDRATE
{
    mB0=1,mB4800,mB9600,mB19200,mB38400,mB57600,mB115200
};
public:
    USART(const char * dev);
    ~USART();
    void startRcv();
    void stopRcv();
    int Receive(u8 * buf);
    int Transmit(u8 * buf);
    int SetBaudrate(u8 baudrate);
    int getBaudrate(void)
    {
        return Baudrate;
    }
    int SetReadmode(int mode);
private:
    int fd;
    struct termios options;
    int ret;
    pthread_t SendMsg_id,RecvMsg_id;
    u8 Baudrate;
    u8 FlagRecvPthread;
    u8 USART_RX_STA;
    USART_RcvThread *mRcvThread;
    QThread *mQThread;
signals:
    void startRcvThread(int fd);
    void stopRcvThread();
    void sendGyroDate_signal(QVector<u8>a);
public slots:
    void sendGyroDate_slot(QVector<u8>a);
};

#endif // USART_H
