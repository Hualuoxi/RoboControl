#include "robot.h"

Robot::Robot(QObject * parent)
{
    InitRobot();
    ConnetSlots();
    mTimer = new QTimer();
    mTimer->setInterval(50);
    connect(mTimer,&QTimer::timeout,this,&Robot::setAngle_slot);
    //mTimer->start();

}

Robot::~Robot()
{
    if(mGyroQThread)
        mGyroQThread->quit();
    mGyroQThread->wait();
    if(mClinetQThread)
        mClinetQThread->quit();
    mClinetQThread->wait();
    mTimer->stop();
    mTimer->deleteLater();

    mCAN->deleteLater();
    mUSART->deleteLater();
    for(int i = 0;i<6;i++)
    {
        Leg[i]->deleteLater();
    }
}


void Robot::InitRobot()
{
    mCAN = new CAN("can1");
    mCAN->startRcv(); //

    mGyro = new Gyro();
    mGyroQThread = new QThread(this);
    mGyro->moveToThread(mGyroQThread);
    mGyroQThread->start();
    connect(mGyroQThread,&QThread::finished,mGyroQThread,&QThread::deleteLater);
    connect(mGyroQThread,&QThread::finished,mGyro,&QObject::deleteLater);

    mClient = new Clinet(this,20200);
    mClinetQThread = new QThread();
    mClient->moveToThread(mClinetQThread);
    connect(mClinetQThread,&QThread::finished,mClinetQThread,&QThread::deleteLater);
    connect(mClinetQThread,&QThread::finished,mClient,&QObject::deleteLater);
    mClinetQThread->start();





    BroadCast = new Motor(0,POSITON_MODE);
    connect(BroadCast,&Motor::sendCANMsg_sig,mCAN,&CAN::Transmit);
    for(int i = 0;i<6;i++)
    {
        Leg[i]=new Motor((i+1),POSITON_MODE);
//        qDebug()<<QString("Leg[%1]->Kv_Encoder : %2").arg(i).arg(Leg[i]->Kv_Encoder);
        connect(Leg[i],&Motor::sendCANMsg_sig,mCAN,&CAN::Transmit);
        if(i==2)
        {
            Leg[i]->EncoderCnt = 512;
            Leg[i]->Kv_Encoder= Leg[i]->EncoderCnt * 4./60;
            Leg[i]->Leg_Reduction = 19.2;
            Leg[i]->initMotor();
        }
        else
            Leg[i]->initMotor();
    }

    mRoboDriver = new RoboDriver(Leg,BroadCast);
    mRoboDriverQThread = new QThread();
    mRoboDriver->moveToThread(mRoboDriverQThread);
    connect(mRoboDriverQThread,&QThread::finished,mRoboDriverQThread,&QThread::deleteLater);
    connect(mRoboDriverQThread,&QThread::finished,mRoboDriver,&QObject::deleteLater);
    connect(this,&Robot::run_sig,mRoboDriver,&RoboDriver::run);
    mRoboDriverQThread->start();


}

void Robot::run()
{
    emit run_sig();
}

void Robot::InitPIDParam()
{
    memset(&PID_Yaw,0,sizeof(RobotPID));
    memset(&PID_Pitch,0,sizeof(RobotPID));
    memset(&AngleWant,0,sizeof(RobotAngle));
}

void Robot::ConnetSlots()
{
    qRegisterMetaType<u8>("u8");
    qRegisterMetaType<s16>("s16");
    connect(this,&Robot::sendCANMsg_sig,mCAN,&CAN::Transmit);
    connect(this,&Robot::TranClinet_sig,mClient,&Clinet::TranClinet_slot);
    connect(mClient,&Clinet::shutDownApp,qApp,&QCoreApplication::quit);

    connect(mCAN,&CAN::setMotorCur_sig,this,&Robot::setMotorCur_slot);
    connect(mCAN,&CAN::setMotorPos_sig,this,&Robot::setMotorPos_slot);
    connect(mCAN,&CAN::setMotorSpd_sig,this,&Robot::setMotorSpd_slot);
    connect(mCAN,&CAN::setPVT_sig,this,&Robot::setPVT_slot);
    connect(mCAN,&CAN::sendPVTPrama_sig,this,&Robot::sendPVTPrama_slot);

}

void Robot::sendPVTPrama_slot(u8 id,s16 Wptr,s16 Rptr)
{
    PVT_Prama temp;

    Leg[id-1]->PVTPrama_Mutex.lock();
    if(!Leg[id-1]->PVTQueue.empty())
    {
        temp = Leg[id-1]->PVTQueue.dequeue();
        Leg[id-1]->SendPVT((int)(temp.pos),(int)(temp.vel),(u8)temp.tim);
    }
    Leg[id-1]->PVTPrama_Mutex.unlock();
    Leg[id-1]->setRPtr(Rptr);
    Leg[id-1]->setWPtr(Wptr);
}

void Robot::setAngle_slot()
{
    AngleNow=mGyro->getAngleNow();
  qDebug()<<"Yaw: "<<AngleNow.yaw<<" Roll:"<<AngleNow.roll<<" Pitch:"<<AngleNow.pitch;
}


/**l
 * @brief set Motor Current
 * @param Motor id
 * @param Current
 */
void Robot::setMotorCur_slot(unsigned char id,float value)
{
    Leg[id-1]->setMotorCur(value);
}

/**
 * @brief set Motor Current
 * @param Motor id
 * @param Current
 */
void Robot::setMotorPos_slot(u8 id,int value)
{
//    qDebug()<<QString("Leg[%1]->setMotorPos(%2)").arg(id).arg(value);
    Leg[id-1]->setMotorPos(value);
}

/**
 * @brief set Motor Current
 * @param Motor id
 * @param Current
 */
void Robot::setMotorSpd_slot(u8 id,float value)
{
    Leg[id-1]->setMotorSpd(value);
}

/**
 * @brief set pvt
 * @param Motor id
 * @param Position
 *        PVT buffer head pointer
 *        PVT buffer tail pointer
 */
void Robot::setPVT_slot(u8 id ,int pos,s16 Hptr,s16 Tptr)
{
    Leg[id-1]->setMotorPos(pos);
    Leg[id-1]->setWPtr(Hptr);
    Leg[id-1]->setRPtr(Tptr);

}
