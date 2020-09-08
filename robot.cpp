#include "robot.h"

Robot::Robot(QObject * parent)
{
    InitRobot();
    ConnetSlots();
    InitPIDParam();
    SubThread();
//    mTimer = new QTimer();
//    mTimer->setInterval(1000);
//    connect(mTimer,&QTimer::timeout,this,&Robot::setAngle_slot);
//    mTimer->start();

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
    Wheel_1->deleteLater();
    Wheel_2->deleteLater();
    Cutter_1->deleteLater();
    Cutter_2->deleteLater();
    Turn_1->deleteLater();
    Turn_2->deleteLater();

    if(PIDQThread)
        PIDQThread->quit();
    PIDQThread->wait();

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

    mClient = new Clinet(this,20020);
    mClinetQThread = new QThread(this);
    mClient->moveToThread(mClinetQThread);
    mClinetQThread->start();
    connect(mClinetQThread,&QThread::finished,mClinetQThread,&QThread::deleteLater);
    connect(mClinetQThread,&QThread::finished,mClient,&QObject::deleteLater);

    Cutter_1 = new Motor(ELMO_CUTTER_1,SPEED_MODE);
    Cutter_2 = new Motor(ELMO_CUTTER_2,SPEED_MODE);
    Wheel_1 = new Motor(ELMO_WHEEL_1,SPEED_MODE);
    Wheel_2 = new Motor(ELMO_WHEEL_2,SPEED_MODE);
    Turn_1 = new Motor(ELMO_TURN_1,POSITON_MODE);
    Turn_2 = new Motor(ELMO_TURN_2,POSITON_MODE);

    //将各电机的发送信号与CAN发送槽连接
    connect(Cutter_1,&Motor::sendCANMsg,mCAN,&CAN::Transmit);
    connect(Cutter_2,&Motor::sendCANMsg,mCAN,&CAN::Transmit);
    connect(Wheel_1,&Motor::sendCANMsg,mCAN,&CAN::Transmit);
    connect(Wheel_2,&Motor::sendCANMsg,mCAN,&CAN::Transmit);
    connect(Turn_1,&Motor::sendCANMsg,mCAN,&CAN::Transmit);
    connect(Turn_2,&Motor::sendCANMsg,mCAN,&CAN::Transmit);

    //初始化电机，由于初始化的过程中需要发送报文，因此需要在电机与CAN建立连接之后才能调用
    Cutter_1->initMotor();
    Cutter_2->initMotor();
    Wheel_1->initMotor();
    Wheel_2->initMotor();
    Turn_1->initMotor();
    Turn_2->initMotor();

}

void Robot::SubThread()
{
    //PIDQThread = new QThread(this);

    //数据处理线程1000ms后开启 进入事件循环
    PIDQTimer = new QTimer();
    //PIDQTimer->moveToThread(PIDQThread);
    //PIDQTimer->setSingleShot(true);
    PIDQTimer->setInterval(50);
    //connect(PIDQTimer,&QTimer::timeout,this,&Robot::PID,Qt::DirectConnection);
    connect(PIDQTimer,&QTimer::timeout,this,&Robot::PID);
    //connect(PIDQThread,&QThread::finished,PIDQTimer,&QTimer::deleteLater);
    //connect(PIDQThread,&QThread::finished,PIDQThread,&QThread::deleteLater);
    //connect(PIDQThread,SIGNAL(started()),PIDQTimer,SLOT(start()));
    //PIDQThread->start();
    PIDQTimer->start();
}

void Robot::InitPIDParam()
{
    memset(&PID_Yaw,0,sizeof(RobotPID));
    memset(&PID_Pitch,0,sizeof(RobotPID));
    memset(&AngleWant,0,sizeof(RobotAngle));
}

void Robot::ConnetSlots()
{
    connect(this,&Robot::sendCANMsg_sig,mCAN,&CAN::Transmit);
    connect(this,&Robot::TranClinet_sig,mClient,&Clinet::TranClinet_slot);
    connect(mClient,&Clinet::shutDownApp,qApp,&QCoreApplication::quit);
    connect(mCAN,&CAN::setMotorCurrent_sig,this,&Robot::setMotorCurrent_slot);
    connect(mClient,&Clinet::StartRun_sig,this,&Robot::StartRun_slot);
    connect(mClient,&Clinet::StartCoarsen_sig,this,&Robot::StartCoarsen_slot);
    connect(mClient,&Clinet::StopCoarsen_sig,this,&Robot::StopCoarsen_slot);
    connect(mClient,&Clinet::StopRun_sig,this,&Robot::StopRun_slot);
    connect(mClient,&Clinet::turn_sig,this,&Robot::turn_slot);
}



void Robot::setAngle_slot()
{
    AngleNow=mGyro->getAngleNow();
//  qDebug()<<"Yaw: "<<AngleNow.yaw<<" Roll:"<<AngleNow.roll<<" Pitch:"<<AngleNow.pitch;
}

/**
 * @brief StartRun
 * @param speed_
 * @param dir_1: default 1, -1 for opposite direction
 * @param dir_2: default 1, -1 for opposite direction
 */
void Robot::StartRun_slot(int speed_, char dir_1, char dir_2)
{
    int g_dir = 1;
    Wheel_1->Go(- speed_ * 8 * 6 / 60 * dir_1 * g_dir);
    Wheel_2->Go(speed_ * 8 * 6 / 60 * dir_2 * g_dir);
}
void Robot::StopRun_slot()
{
    Wheel_1->StopByST();
    Wheel_2->StopByST();
}
/**
 * @brief StartCoarsen
 * @param speed
 * @param direction: default 1, -1 for opposite direction
 */
void Robot::StartCoarsen_slot(int speed, char direction)
{

    // set speed of the 2 coarsening motors
    int encoder_num = 500; // 512 for RE40; 2048 for EC45
    Cutter_1->Go(- speed * encoder_num * 4 * 4.3 / 15/ 60 * direction);
    Cutter_2->Go(speed * encoder_num * 4 / 60 * direction);
}
void Robot::StopCoarsen_slot()
{
    Cutter_1->Stop();
    Cutter_2->Stop();
}

/**
 * @brief turn robot
 * @param position
 * @param turnDegree: the count want to run
 */
void Robot::turn_slot(int turnDegree)
{

    Turn_1->RunByPos(turnDegree);
    Turn_2->RunByPos(turnDegree);
}

/**
 * @brief set Motor Current
 * @param position
 * @param turnDegree: the count want to run
 */
void Robot::setMotorCurrent_slot(int id,float value)
{
    switch (id)
    {
    case ELMO_CUTTER_1_RX:
        Cutter_1->setMotorCurrent(value);
        break;
    case ELMO_CUTTER_2_RX:
        Cutter_2->setMotorCurrent(value);
        break;
    case  ELMO_WHEEL_1_RX:
        Wheel_1->setMotorCurrent(value);
        break;
    case ELMO_WHEEL_2_RX:
        Wheel_2->setMotorCurrent(value);
        break;
    case ELMO_TURN_1_RX:
        Turn_1->setMotorCurrent(value);
        break;
    case ELMO_TURN_2_RX:
        Turn_2->setMotorCurrent(value);
        break;
    default:
        qDebug()<<"Elmo id error!";
    }
}

void Robot::PID()
{
    AngleNow=mGyro->getAngleNow();
//    float sign_Wheel1=(Wheel_1->getMotorSpeed()>0?1:-1);
//    float sign_Wheel2=(Wheel_2->getMotorSpeed()>0?1:-1);
    PID_Yaw.Ek[0]=AngleNow.yaw-AngleWant.yaw;
    PID_Yaw.OutPut =  PID_Yaw.Kp * PID_Yaw.Ek[0] + PID_Yaw.Ki * PID_Yaw.ErrSum + PID_Yaw.Kd * (PID_Yaw.Ek[0] - PID_Yaw.Ek[1]);
    PID_Yaw.Ek[1]= PID_Yaw.Ek[0];
    PID_Yaw.ErrSum+=PID_Yaw.Ek[0];

    PID_Pitch.Ek[0]=AngleNow.pitch-AngleWant.pitch;
    PID_Pitch.OutPut =  PID_Pitch.Kp * PID_Pitch.Ek[0] + PID_Pitch.Ki * PID_Pitch.ErrSum + PID_Pitch.Kd * (PID_Pitch.Ek[0] - PID_Pitch.Ek[1]);
    PID_Pitch.Ek[1]= PID_Pitch.Ek[0];
    PID_Pitch.ErrSum+=PID_Pitch.Ek[0];
    Wheel_1->Go(Wheel_1->getMotorSpeed()+PID_Yaw.OutPut+PID_Pitch.OutPut);
    Wheel_2->Go(Wheel_2->getMotorSpeed()-PID_Yaw.OutPut+PID_Pitch.OutPut);

}
