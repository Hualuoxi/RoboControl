#include "usart.h"

USART::USART(const char * dev)
{
    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);  //fd为打开的终端文件描述符
    if(fd < 0)
        qDebug() << dev <<"open error";

    //fcntl(fd, F_SETFL, FNDELAY);        //标志FNDELAY可以保证read函数在端口上读不到字符的时候返回0
    fcntl(fd, F_SETFL, 0);            //回到正常（阻塞）模式

    memset(&options, 0, sizeof(options)) ;

    if(0 != (tcgetattr(fd, &options)))    //取得设备终端属性
    {
        qDebug() <<"tcgetattr() failed:"<<strerror(errno);
        close(fd) ;
        qDebug() << dev <<"closed!" ;
        return;
    }

    cfsetispeed(&options, B115200);     //设置输入波特率 115200
    cfsetospeed(&options, B115200);     //设置输出波特率 115200

    options.c_cflag |= CLOCAL | CREAD;  //通过位掩码的方式激活本地连接和接受使能选项：CLOCAL和CREAD
    options.c_cflag &= (u32)~CSIZE;     //在设置数据位时 必须先使用CSIZE做位屏蔽
    options.c_cflag |= CS8;             //设置数据位8
    options.c_cflag &= (u32)~PARENB;    //无校验位
    options.c_cflag &= (u32)~CSTOPB;    //一位停止位
    options.c_cflag &= (u32)~CRTSCTS;

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
    tcflush(fd ,TCIFLUSH);          //tcflush清空终端未完成的输入/输出请求及数据 TCIFLUSH表示清空正收到的数据，且不读取出来
    USART_RX_STA = 0;
    if( 0 !=(tcsetattr(fd, TCSANOW, &options)))
    {
        qDebug() <<"tcsetattr failed:"<<strerror(errno);
        close(fd);
        qDebug() << dev <<" closed!";
    }
    else
        qDebug() << dev <<" open successfully!"<< endl <<" The baud rate is 115200 ";

    mQThread = new QThread(this);
    mRcvThread = new USART_RcvThread;
    mRcvThread->moveToThread(mQThread);
    connect(mQThread, &QThread::finished, mRcvThread, &QObject::deleteLater);
    connect(mQThread, &QThread::finished, mQThread, &QThread::deleteLater);
    connect(this, &USART::startRcvThread, mRcvThread, &USART_RcvThread::RcvMegThread);
    connect(this, &USART::stopRcvThread, mRcvThread, &USART_RcvThread::stopRcvThread);
    mQThread->start();
    qRegisterMetaType<QVector<u8>>("QVector<u8>");
    connect(mRcvThread, &USART_RcvThread::sendGyroDate_signal, this, &USART::sendGyroDate_slot);
}

USART::~USART()
{
    close(fd);
}

int USART::SetBaudrate(u8 baudrate)
{
    Baudrate = baudrate;
    switch (Baudrate)
    {
        case mB115200:
            return ((cfsetispeed(&options, B115200))&(cfsetospeed(&options, B115200)));
        case mB57600:
            return ((cfsetispeed(&options, B57600))&(cfsetospeed(&options, B57600)));
        case mB38400:
            return ((cfsetispeed(&options, B38400))&(cfsetospeed(&options, B38400)));
        case mB19200:
            return ((cfsetispeed(&options, B19200))&(cfsetospeed(&options, B19200)));
        case mB9600:
            return ((cfsetispeed(&options, B9600))&(cfsetospeed(&options, B9600)));
        case mB4800:
            return ((cfsetispeed(&options, B4800))&(cfsetospeed(&options, B4800)));
        case mB0:
            return ((cfsetispeed(&options, B0))&(cfsetospeed(&options, B0)));
        default:
            qDebug() << "Baudrate input error!"<<endl;
            return -1;
    }
}

int USART::SetReadmode(int mode)
{
    if(mode)
        return fcntl(fd, F_SETFL, 0);
    else if(mode == FNDELAY)
        return fcntl(fd, F_SETFL, FNDELAY);
    else
    {
        qDebug() << "Readmode input error!"<<endl;
        return 0;
    }
}

int USART::Transmit(u8 * buf)
{
    return write(fd, buf, sizeof (buf));
}

int USART::Receive(u8 * buf)
{
    return read(fd, buf, sizeof(u8));
}

void USART::startRcv()
{
    emit startRcvThread(fd);
}

void USART::stopRcv()
{
    emit stopRcvThread();
}

void USART::sendGyroDate_slot(QVector<u8>a)
{
    emit sendGyroDate_signal(a);
}



USART_RcvThread::USART_RcvThread(QObject *parent )
{

}

void USART_RcvThread::RcvMegThread(int fd)
{
    qDebug()<<"USART_RcvThread is begin"<<QThread::currentThread();
    u8 temp=0;
    u8 byte = 0;
    QVector<u8> Data_Yesense(21);
    Data_Yesense[0]=0x59;
    Data_Yesense[1]=0x53;
    while(!isStop)
    {
        if(read(fd, &temp, sizeof(u8)))
        {
            if(temp == Data_Yesense[0])					//接收到帧头
            {
                if(read(fd, &temp, sizeof(u8)))
                {
                    if(temp == Data_Yesense[1])					//接收到帧头
                    {
                        byte = read(fd, &Data_Yesense[2], 19);
                        if(byte==19)
                        {
                            //qDebug()<<"Gyro successful";
                            emit sendGyroDate_signal(Data_Yesense);
                            byte = 0;
                        }

                    }
                }
            }
        }
    }
}

void USART_RcvThread::stopRcvThread()
{
    isStop = true;
}
