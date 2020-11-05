#include "robot.h"


Robot::Robot(QObject * parent)
{
    InitRobot();
    ConnetSlots();
    mTimer = new QTimer();
    mTimer->setInterval(100);
    connect(mTimer,&QTimer::timeout,this,&Robot::SavePv_slot);
    
#if 0
    mTimer->start();
    QDir csvdir;
    QString currentDir = csvdir.currentPath();
    if(!csvdir.exists("csv"))
    {
        csvdir.mkdir("csv");
    }
    CurData = new QFile(currentDir+"/csv/vel-11031131.csv");
    if(CurData->open(QFile::WriteOnly | QFile::Truncate))
    {
        QTextStream CurOut(CurData);
        CurOut<<"pos[0],"<<"vel[0],"
//              <<"poswant[0],"<<"velwant[0],"
              <<"pos[1],"<<"vel[1],"
                <<"pos[2],"<<"vel[2],"
                  <<"pos[4],"<<"vel[4],"
//              <<"poswant[1],"<<"velwant[1],"
              <<"\n";
    }
#endif
    
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
    CurData->close();
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
        if(i==4)
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
    //connect(mClient,&Clinet::shutDownApp,qApp,&QCoreApplication::quit);

    connect(mClient,&Clinet::RcvClinet_sig,this,&Robot::RcvDHPClinet_slot);

    connect(this,&Robot::PVTGoForward_sig,mRoboDriver,&RoboDriver::PVTGoForward);
    connect(this,&Robot::PVTWalkForward_sig,mRoboDriver,&RoboDriver::PVTWalkForward);
    connect(this,&Robot::PVTRun_sig,mRoboDriver,&RoboDriver::PVTRun);
    connect(this,&Robot::PVTReady_sig,mRoboDriver,&RoboDriver::PVTReady);
    connect(this,&Robot::PVTRotate_sig,mRoboDriver,&RoboDriver::PVTRotate);
    connect(this,&Robot::PVTWalkBack_sig,mRoboDriver,&RoboDriver::PVTWalkBack);
    connect(this,&Robot::PVTTurnLeft_sig,mRoboDriver,&RoboDriver::PVTTurnLeft);
    connect(this,&Robot::PVTTurnRight_sig,mRoboDriver,&RoboDriver::PVTTurnRight);
    connect(this,&Robot::SearchZero_sig,mRoboDriver,&RoboDriver::SearchZero);

    connect(this->mCAN->mRcvThread,&CAN_RcvThread::sendPVTPrama_sig,this,&Robot::sendPVTPrama_slot);
    connect(this->mCAN->mRcvThread,&CAN_RcvThread::setMotorCur_sig,this,&Robot::setMotorCur_slot);
    connect(this->mCAN->mRcvThread,&CAN_RcvThread::setMotorSpd_sig,this,&Robot::setMotorSpd_slot);
    connect(this->mCAN->mRcvThread,&CAN_RcvThread::setMotorPos_sig,this,&Robot::setMotorPos_slot);
    connect(this->mCAN->mRcvThread,&CAN_RcvThread::setMotorPV_sig,this,&Robot::setMotorPV_slot);
    connect(this->mCAN->mRcvThread,&CAN_RcvThread::setPVT_sig,this,&Robot::setPVT_slot);
}


void Robot::RcvDHPClinet_slot(QByteArray a)
{
    CMD_frame *cmdInfo = (CMD_frame*)a.data();
    if(cmdInfo->Header[0] == 'R' && cmdInfo->Header[1] == 'H')
    {
        if(cmdInfo->Cmd[0] == 'G' && cmdInfo->Cmd[1] == 'F')
        {
            startRecord = true;
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            mRoboDriver->setSuspension_count(((float)(cmdInfo->Data[2]))/10.);
            emit PVTWalkForward_sig(cmdInfo->Data[1], cmdInfo->Data[0]);
            return;
        }
        if(cmdInfo->Cmd[0] == 'A' && cmdInfo->Cmd[1] == 'R')
        {
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            emit PVTRun_sig(1000,cmdInfo->Data[1],cmdInfo->Data[0],100);
            return;
        }
        if(cmdInfo->Cmd[0] == 'G' && cmdInfo->Cmd[1] == 'B')
        {
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            emit PVTWalkBack_sig(cmdInfo->Data[1], cmdInfo->Data[0]);
            return;
        }
        if(cmdInfo->Cmd[0] == 'R' && cmdInfo->Cmd[1] == 'O')
        {
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            emit PVTRotate_sig(cmdInfo->Data[1], cmdInfo->Data[0]);
            return;
        }
        if(cmdInfo->Cmd[0] == 'T' && cmdInfo->Cmd[1] == 'R')
        {
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            emit PVTTurnRight_sig(cmdInfo->Data[1], cmdInfo->Data[0]);
            return;
        }
        if(cmdInfo->Cmd[0] == 'T' && cmdInfo->Cmd[1] == 'L')
        {
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            emit PVTTurnLeft_sig(cmdInfo->Data[1], cmdInfo->Data[0]);
            return;
        }
        if(cmdInfo->Cmd[0] == 'S' && cmdInfo->Cmd[1] == 'T')
        {

            mRoboDriver->setStopRun(true);
            startRecord = false;
            for(int i =0; i<6 ;i++)
            {
                Leg[i]->setPVTflag(false);
                Leg[i]->ResetPVT();
            }
            return;
        }
        if(cmdInfo->Cmd[0] == 'R' && cmdInfo->Cmd[1] == 'E')
        {

            startRecord = false;
            BroadCast->Release();

            return;
        }
        if(cmdInfo->Cmd[0] == 'R' && cmdInfo->Cmd[1] == 'D')
        {
            mRoboDriver->setStopRun(false);
            for(int i =0; i<6 ;i++)
            {
               Leg[i]->setPVTflag(true);
            }
            emit PVTReady_sig(cmdInfo->Data[0]/10.,2000);
            return;
        }
        if(cmdInfo->Cmd[0] == 'S' && cmdInfo->Cmd[1] == 'Z')
        {
            emit SearchZero_sig();
            return;
        }
    }
}


void Robot::sendPVTPrama_slot(u8 id,s16 Wptr,s16 Rptr)
{
    PVT_Prama temp;

    Leg[id-1]->PVTPrama_Mutex.lock();
    if(!Leg[id-1]->PVTQueue.empty()&&Leg[id-1]->getPVTflag())
    {
        temp = Leg[id-1]->PVTQueue.dequeue();
        Leg[id-1]->SendPVT((int)(temp.pos),(int)(temp.vel),(u8)temp.tim);
        Leg[id-1]->setMotorPosWant((int)(temp.pos));
        Leg[id-1]->setMotorSpdWant((int)(temp.vel));
//        if(id==1)
//            qDebug()<<QString("pos = %1 , vel = %2 , tim = %3").arg(temp.pos).arg(temp.vel).arg(temp.tim);
    }
    else if (Leg[id-1]->getPVTflag())
    {
        qDebug() <<QString("Leg[%1] queue is empty!").arg(id-1);
    }
    Leg[id-1]->PVTPrama_Mutex.unlock();
    Leg[id-1]->setRPtr(Rptr);
    Leg[id-1]->setWPtr(Wptr);
}

void Robot::SaveCur_slot()
{

    for(int i =0; i<6 ;i++)
    {
       Leg[i]->QueryCur();
    }
    if(startRecord)
    {
        QTextStream CurOut(CurData);
        QTime mytime=QTime::currentTime();
        QString strTime = mytime.toString("hh-mm-ss-zzz");
        CurOut<<Leg[0]->getMotorCur()<<","
              <<Leg[1]->getMotorCur()<<","
              <<Leg[2]->getMotorCur()<<","
              <<Leg[3]->getMotorCur()<<","
              <<Leg[4]->getMotorCur()<<","
              <<Leg[5]->getMotorCur()<<","
              <<strTime<<"\n";
    }
}

void Robot::SavePv_slot()
{

    BroadCast->SendSYNC();
    QTextStream CurOut(CurData);
    CurOut<<Leg[0]->getMotorPos()<<","
          <<Leg[0]->getMotorSpd()<<","
          <<Leg[1]->getMotorPos()<<","
          <<Leg[1]->getMotorSpd()<<","
          <<Leg[2]->getMotorPos()<<","
          <<Leg[2]->getMotorSpd()<<","
          <<Leg[4]->getMotorPos()<<","
          <<Leg[4]->getMotorSpd()<<","
          <<"\n";
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
 * @brief set Motor Pos and Vel
 * @param Motor id
 * @param Current
 */
void Robot::setMotorPV_slot(u8 id,int pos , int vel)
{
//    qDebug()<<QString("Leg[%1]->setMotorPos(%2)").arg(id).arg(value);
    Leg[id-1]->setMotorPos(pos);
    Leg[id-1]->setMotorSpd(vel);

}

/**
 * @brief set Motor Speed
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
