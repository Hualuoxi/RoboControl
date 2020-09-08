#ifndef CAN_H
#define CAN_H
#include <QObject>
#include <QThread>
#include <QtCore>
#include <fcntl.h>        /*文件控制定义*/
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <Utilities.h>
#include <QDebug>
#include <unistd.h>
//class CAN;
class CAN_RcvThread :public QObject
{
    Q_OBJECT
public:
    CAN_RcvThread(QObject *parent = 0) ;
signals:
    void setMotorCurrent_sig(int id,float value);
public slots:
    void RcvMegThread(int s);
    void stopRcvThread();
private:
    bool isStop=false;
};

class CAN:public QObject
{
    Q_OBJECT

enum CAN_BITRATE
{
    R5K=1,R10K,R20K,R50K,R100K,R125K,R250K,R500K,R800K,R1M
};

public:
    CAN(const char * CANx);
    ~CAN();
    void start();
    void stop();

    void setLoopback(int loopback);
    int  getLoopbackStatus(void){
        return LoopBackStatus;
    }

    void setSocketLoopback(int ro);
    int  getSocketLoopbackStatus(void){
        return SocketLoopBackStatus;
    }

    void setFilterNum(u8 num);
    u8   getFilterNum(void){
        return CAN_FILTER_NUM;
    }

    void setFilter(u8 Number, u32 id, u32 mask);
    u32  getFilterId(u8 Number){
        return rfilter[Number].can_id;
    }
    u32  getFilterMask(u8 Number){
        return rfilter[Number].can_mask;
    }

    int  setReadmode(int mode);
    int  getReadmode(void){
        return ReadMode;
    }

    int setCANRate(u8 rate);
    int getCANRate(void){
        return Bitrate;
    }
    int Receive(can_frame *pFrame);

    void startRcv();
    void stopRcv();
private:

    CAN_RcvThread *mRcvThread;
    QThread *mQThread;
    int s;
    const char * mCAN;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter *rfilter;
    u8 CAN_FILTER_NUM = 1;   // 注意：定义了过滤器数组之后必须全部赋值 否则会收到所有的报文
    int LoopBackStatus;
    int SocketLoopBackStatus;
    int ReadMode;
    u8 Bitrate;

    int ret = 0;
    pthread_t CANSendMsg_id, CANRecvMsg_id;
    u8 FlagSendPthread, FlagRecvPthread;

signals:
    void startRcvThread(int);
    void stopRcvThread();
    void setMotorCurrent_sig(int id,float value);

public slots:
    void Transmit(can_frame pFrame);
    void setMotorCurrent_slot(int id,float value);

};

#endif // CAN_H
