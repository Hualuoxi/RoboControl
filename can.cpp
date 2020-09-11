#include "can.h"
/**
 * @brief CAN对象构造函数
 * @param CAN设备名
 *        can0 or can1
 * @note  默认比特率为1M
 *        默认滤波器数量为1
 *        默认报文读取模式为快速响应模式
 *        默认报文过滤规则为不接受所有报文
 */
CAN::CAN(const char * CANx)
{
    mCAN = CANx;
    Bitrate = R1M;
    if(!strcmp(mCAN, "can0"))
    {
        system("ifconfig can0 down");                               //close can0
        system("ip link set can0 type can bitrate 1000000");        //set can0 bitrate 1M
        system("ifconfig can0 up");                                 //open can0
    }
    else if(!strcmp(mCAN,"can1"))
    {
        system("ifconfig can1 down");                               //close can0
        system("ip link set can1 type can bitrate 1000000");        //set can0 bitrate 1M
        system("ifconfig can1 up");                                 //open can0
    }
    else
    {
        qDebug() << "Input CANx Error , Please Input can0 or can1 for the 9x35";
    }
    CAN_FILTER_NUM = 1;          // 初始化CAN过滤器数量
    rfilter = (struct can_filter*)malloc((sizeof(rfilter) * CAN_FILTER_NUM));

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr.ifr_name,mCAN);

    ReadMode = FNDELAY;
    //fcntl(s, F_SETFL, ReadMode);        //标志FNDELAY可以保证read函数在端口上读不到字符的时候返回0
    fcntl(s, F_SETFL, 0);            //回到正常（阻塞）模式

    ioctl(s, SIOCGIFINDEX, &ifr); //指定 CANx 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));// 将套接字与 CANx 绑定
    //设置过滤规则 默认不接收任何数据
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, sizeof(rfilter));
    setFilter(0, 0x00, 0x00);

    mQThread = new QThread(this);
    mRcvThread = new CAN_RcvThread;
    mRcvThread->moveToThread(mQThread);

    qRegisterMetaType<can_frame>("can_frame");
    qRegisterMetaType<u8>("u8");
    connect(mQThread, &QThread::finished, mRcvThread, &QObject::deleteLater);
    connect(mQThread, &QThread::finished, mQThread, &QThread::deleteLater);
    connect(this, &CAN::startRcvThread, mRcvThread, &CAN_RcvThread::RcvMegThread);
    connect(this, &CAN::stopRcvThread, mRcvThread, &CAN_RcvThread::stopRcvThread);
    connect(mRcvThread,&CAN_RcvThread::setMotorCur_sig,this,&CAN::setMotorCur_slot);
    connect(mRcvThread,&CAN_RcvThread::setMotorSpd_sig,this,&CAN::setMotorSpd_slot);
    connect(mRcvThread,&CAN_RcvThread::setMotorPos_sig,this,&CAN::setMotorPos_slot);
    mQThread->start();



}

CAN::~CAN() {
    stop();
    if(mQThread)
        mQThread->quit();
    mQThread->wait();
}

/**
 * @brief 关闭CAN设备
 */
void CAN::stop()
{
    FlagSendPthread = 0;
    FlagRecvPthread = 0;
    if(!strcmp(mCAN,"can0"))
    {
        system("ifconfig can0 down");                               //close can0
    }
    else if(!strcmp(mCAN,"can1"))
    {
        system("ifconfig can1 down");                               //close can1
    }
}

/**
 * @brief 开启CAN设备
 */
void CAN::start()
{
    if(!strcmp(mCAN,"can0"))
    {
        system("ifconfig can0 up");                                 //open can0
    }
    else if(!strcmp(mCAN,"can1"))
    {
        system("ifconfig can1 up");                                 //open can1
    }
}

/**
 * @brief 设置本地回环
 * @param loopback = 0 关闭状态
 *        loopback = 1 开启状态 (默认)
 */
void CAN::setLoopback(int loopback)
{
    LoopBackStatus = loopback;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &LoopBackStatus, sizeof(LoopBackStatus));
}


/**
 * @brief 设置套接字接收回环的自己发送的报文
 * @param ro = 0 关闭状态 (默认)
 *        ro = 1 开启状态
 */
void CAN::setSocketLoopback(int ro)
{
    SocketLoopBackStatus = ro;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &SocketLoopBackStatus, sizeof(SocketLoopBackStatus));
}


/**
 * @brief 设置滤波器个数
 * @param num 滤波器个数
 */
void CAN::setFilterNum(u8 num)
{
    CAN_FILTER_NUM = num;
}


/**
 * @brief 设置指定滤波器的过滤报文
 * @param Number 滤波器编号 有效参数为滤波器个数-1
 * @note  定义的滤波器数组必须全部赋值 否则滤波器将接收所有数据
 */
void CAN::setFilter(u8 Number,u32 id ,u32 mask)
{
    if(Number < CAN_FILTER_NUM)
    {
        rfilter[Number].can_id = id;
        rfilter[Number].can_mask = mask;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    }
    else
      qDebug() << "SetCANFilter error!\n"<<endl;
}

/**
 * @brief 设置读取模式
 * @param mode = 0       默认堵塞模式  读不到数据则堵塞线程 一直等待
 *        mode = FNDELAY(04000)   快速响应模式  读不到数据则返回0
 */
int CAN::setReadmode(int mode)
{
    ReadMode = mode;
    if(ReadMode)
        return fcntl(s, F_SETFL, 0);
    else if(ReadMode == FNDELAY)
        return fcntl(s, F_SETFL, FNDELAY);
    else
    {
        qDebug() << "Readmode input error!\n"<<endl;
        return 0;
    }
}

/**
 * @brief 设置CAN传输速率
 * @param  rate 取值为CAN_BITRATE中的值
 */
int CAN::setCANRate(u8 rate)
{
    Bitrate = rate;
    switch(Bitrate)
    {
        case R5K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 5000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R10K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R20K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R50K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R100K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R125K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R250K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R500K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R800K:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        case R1M:
            system("ifconfig can0 down");                               //close can0
            system("ip link set can0 type can bitrate 10000");        //set can0 bitrate 1M
            system("ifconfig can0 up");                                 //open can0
        break;
        default:
            qDebug() <<mCAN<< " Bitrate input Error! \n"<<endl;
            return 0;
    }
    return 1;
}


/**
 * @brief 接收CAN报文
 * @param 用于接收的报文
 * @return 接收到的数据的长度
 */

int CAN::Receive(can_frame *Rx_Message)
{
    return read(s, Rx_Message, sizeof(can_frame));
}

/**
 * @brief 发送CAN报文
 * @param Tx_Message 用于发送的报文
 * @return nbytes 发送的数据的长度
 */
void CAN::Transmit(can_frame pFrame)
{
    int nbytes = write(s, &pFrame, sizeof(can_frame)); //发送数据
    if(nbytes != sizeof(can_frame)){ //如果 nbytes 不等于帧长度，就说明发送失败
        qDebug() << mCAN << " Send Error! ";
    }
}

void CAN::startRcv()
{
    emit startRcvThread(s);
}

void CAN::stopRcv()
{
    emit stopRcvThread();
}

void CAN::setMotorCur_slot(u8 id,float value)
{
    emit setMotorCur_sig(id,value);
}

void CAN::setMotorPos_slot(u8 id,int value)
{
    emit setMotorPos_sig(id,value);
}

void CAN::setMotorSpd_slot(u8 id,float value)
{
    emit setMotorSpd_slot(id,value);
}


CAN_RcvThread::CAN_RcvThread(QObject *parent )
{

}

void CAN_RcvThread::RcvMegThread(int s)
{
    qDebug()<<"CAN_RcvThread is begin"<<QThread::currentThread();
    can_frame *rx_msg = new can_frame();
    int byte;
    int i=0;
    while(!isStop)
    {
//        qDebug()<<"CAN_RcvThread is run";
        byte=read(s,rx_msg,sizeof (can_frame));
//        qDebug()<<byte;
        if(byte>0)
        {
            if(rx_msg->can_id > ELMO_FBCK_ID_BASE && rx_msg->can_id <= ELMO_FBCK_ID_BASE + ELMO_NUM) // coarsening motor message
            {
                if('I' == rx_msg->data[0] && 'Q' == rx_msg->data[1]) // query current
                {
                    float value = 0;
                    Char2Float(&value, &rx_msg->data[4]);
                    emit setMotorCur_sig((u8)(rx_msg->can_id-ELMO_FBCK_ID_BASE),value);
//                    qDebug() << "elmo_id = " << rx_msg->can_id << ", current = " << value << endl;
                }
                else if('V' == rx_msg->data[0] && 'X' == rx_msg->data[1]) // query px
                {
                    float value = 0;
                    Char2Float(&value, &rx_msg->data[4]);
                    emit setMotorSpd_sig((u8)(rx_msg->can_id-ELMO_FBCK_ID_BASE),value);

                }
                else if('P' == rx_msg->data[0] && 'X' == rx_msg->data[1]) // query px
                {
                    int value = 0;
                    Char2Int(&value, &rx_msg->data[4])
                    emit setMotorPos_sig((u8)(rx_msg->can_id-ELMO_FBCK_ID_BASE),value);
//                    qDebug() << rx_msg->can_id << ", PX = " << value <<endl ;
                }
                else if('E' == rx_msg->data[0] && 'C' == rx_msg->data[1])
                {
                    int value = 0;
                    Char2Int(&value, &rx_msg->data[4])
                    qDebug() <<QString("Leg[%1] error code %2").arg(rx_msg->can_id-ELMO_FBCK_ID_BASE).arg(value);
                }
            }
        }
        else
        {
            perror("read:");
        }

    }
}

void CAN_RcvThread::stopRcvThread()
{
    isStop = true;
}
