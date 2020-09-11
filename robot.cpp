#include "robot.h"

Robot::Robot(QObject * parent)
{
    InitRobot();
    ConnetSlots();
    //InitPIDParam();
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
    for(int i = 0;i<6;i++)
    {
        Leg[i]->deleteLater();
    }
    if(RunQThread)
        RunQThread->quit();
    RunQThread->wait();

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

    BroadCast = new Motor(0,POSITON_MODE);
    connect(BroadCast,&Motor::sendCANMsg_sig,mCAN,&CAN::Transmit);
    for(int i = 0;i<6;i++)
    {
        Leg[i]=new Motor((i+1),POSITON_MODE);
        connect(Leg[i],&Motor::sendCANMsg_sig,mCAN,&CAN::Transmit);
        Leg[i]->initMotor();
    }
    BroadCast->initMotor();
    Leg[2]->EncoderCnt = 512;
    Leg[2]->Kv_Encoder= Leg[3]->EncoderCnt * 4/60;
    Leg[2]->initMotor();

}

void Robot::SubThread()
{
    RunQThread = new QThread(this);
    //数据处理线程1000ms后开启 进入事件循环
    RunQTimer = new QTimer();
    RunQTimer->moveToThread(RunQThread);
    RunQTimer->setSingleShot(true);
    RunQTimer->setInterval(50);
    connect(RunQTimer,&QTimer::timeout,this,&Robot::run,Qt::DirectConnection);
    connect(RunQTimer,&QTimer::timeout,this,&Robot::run);
    connect(RunQThread,&QThread::finished,RunQTimer,&QTimer::deleteLater);
    connect(RunQThread,&QThread::finished,RunQThread,&QThread::deleteLater);
    connect(RunQThread,SIGNAL(started()),RunQTimer,SLOT(start()));


}
void Robot::StartRun()
{
    RunQThread->start();
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
    connect(this,&Robot::sendCANMsg_sig,mCAN,&CAN::Transmit);
    connect(this,&Robot::TranClinet_sig,mClient,&Clinet::TranClinet_slot);
    connect(mClient,&Clinet::shutDownApp,qApp,&QCoreApplication::quit);

    connect(mCAN,&CAN::setMotorCur_sig,this,&Robot::setMotorCur_slot);
    connect(mCAN,&CAN::setMotorPos_sig,this,&Robot::setMotorPos_slot);
    connect(mCAN,&CAN::setMotorSpd_sig,this,&Robot::setMotorSpd_slot);

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

}
void Robot::StopRun_slot()
{

}
/**
 * @brief StartCoarsen
 * @param speed
 * @param direction: default 1, -1 for opposite direction
 */
void Robot::StartCoarsen_slot(int speed, char direction)
{


}
void Robot::StopCoarsen_slot()
{

}

/**
 * @brief turn robot
 * @param position
 * @param turnDegree: the count want to run
 */
void Robot::turn_slot(int turnDegree)
{

}

/**
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

void Robot::run()
{
    if(isRun)
    {
        isRun = false;
        qDebug()<<"Robot run!"<<QThread::currentThread();
        Ready(1000,pi*5/3);
        GoForward(1000, 3);
        //BroadCast->EnableMotor(false);
    }

    while(RunQTimer->isActive())
    {
        RunQTimer->stop();
        QThread::msleep(10);
    }
}

/**
 * @brief  从复位状态切换至准备运动状态
 * @input  T              该动作周期    ms
 *         PositionWant   目标状态位置  rad
 */
void Robot::Ready(double T, double PositionWant)
{
    qDebug()<<"Robot Ready";
    BroadCast->EnableMotor(false);
    QThread::msleep(100);
    BroadCast->SetMode(TORQUE_MODE);
    QThread::msleep(100);
    BroadCast->EnableMotor(true);
    QThread::msleep(100);
    BroadCast->RunByCur(-0.4f);
    QThread::msleep(3000);
    int PosTemp[6]={0};
    BroadCast->QueryPos();
    QThread::msleep(100);
    int flag = 0;
    do
    {
        for(int i=0;i<6;i++)
            PosTemp[i]= Leg[i]->getMotorPos();
        BroadCast->QueryPos();
        QThread::msleep(100);
        if(abs(PosTemp[0]-Leg[0]->getMotorPos())<100&&
                abs(PosTemp[1]-Leg[1]->getMotorPos())<100&&
                abs(PosTemp[2]-Leg[2]->getMotorPos())<100&&
                abs(PosTemp[3]-Leg[3]->getMotorPos())<100&&
                abs(PosTemp[4]-Leg[4]->getMotorPos())<100&&
                abs(PosTemp[5]-Leg[5]->getMotorPos())<100)
            flag++;
        else
            flag=0;

    }while(flag<20);
    QThread::msleep(100);
    BroadCast->EnableMotor(false);
    QThread::msleep(100);
    BroadCast->SetZeroPos(0);
    QThread::msleep(100);
    BroadCast->SetMode(POSITON_MODE);
    QThread::msleep(1000);
    ControlParam parameter;
    for (int i = 0; i < 6; i++)
        parameter.T[i] = T;
    for (int i = 0; i < 6; i++)
        parameter.PW[i] = PositionWant;
    SetControlParam(&parameter);
    IndependentControl();

}

/**
 * @brief  向前走
 * @input  T              该动作周期    ms
 *         num            步数
 */
void Robot::GoForward(double T, int num)
{
    ControlParam parameter;

    for (int i = 0; i < 6; i++)
        parameter.T[i] = T;

    for (int i = 0; i < 6; i++)
        parameter.PW[i] = 2 * pi;
    parameter.StepNum = num;
    parameter.PD[0] = 0.0;
    parameter.PD[1] = 0.5;
    parameter.PD[2] = 0.5;
    parameter.PD[3] = 0;
    parameter.PD[4] = 0;
    parameter.PD[5] = 0.5;
    parameter.MoveStatus = RUN;
    SetControlParam(&parameter);
    IndependentControl();
}


/**
 * @brief  双足跑
 */
void Robot::BipedRun()
{
    //前跳实验
    ControlParam Param;
    int i = 0;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    SetControlParam(&Param);
    IndependentControl();

    for (int i = 0; i < 6; i++)
        Param.T[i] = 500;

    i = 0;
    Param.PW[i++] = -1.7 * pi;
    Param.PW[i++] = -1.7 * pi;
    Param.PW[i++] = -1.7 * pi;
    Param.PW[i++] = -1.7 * pi;
    Param.PW[i++] = -1.6 * pi;
    Param.PW[i++] = -1.6 * pi;

    i = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0.5;
    Param.PD[i++] = 0.5;

    SetControlParam(&Param);
    IndependentControl();

    QThread::msleep(450);

    for (int i = 0; i < 6; i++)
        Param.T[i] = 430;
    i = 0;
    Param.PW[i++] = 0;
    Param.PW[i++] = 0;
    Param.PW[i++] = 0;
    Param.PW[i++] = 0;
    Param.PW[i++] = 2 * pi;
    Param.PW[i++] = 2 * pi;

    Param.PD[4] = 0;
    Param.PD[5] = 0.5;
    Param.StepNum = 10;
    //Param.MoveStatus = BIPEDRUN;
    SetControlParam(&Param);
    IndependentControl();

    for (int i = 0; i < 6; i++)
        Param.T[i] = 450;
    Param.StepNum = 10;
    SetControlParam(&Param);
    IndependentControl();
}

/**
 * @brief  翻转
 */
void Robot::RollingOver()
{
    //前跳实验
    ControlParam Param;
    int i = 0;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    Param.PW[i++] = 0.5 * pi;
    SetControlParam(&Param);
    IndependentControl();

    for (int i = 0; i < 6; i++)
        Param.T[i] = 500;

    i = 0;
    Param.PW[i++] = -2.3 * pi;
    Param.PW[i++] = -2.3 * pi;
    Param.PW[i++] = -2.3 * pi;
    Param.PW[i++] = -2.3 * pi;
    Param.PW[i++] = -2.3 * pi;
    Param.PW[i++] = -2.3 * pi;

    i = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0.5;
    Param.PD[i++] = 0.5;

    SetControlParam(&Param);
    IndependentControl();
}

/**
 * @brief  从复位状态切换至准备运动状态
 * @input  T              该动作周期    ms
 *         PositionWant   目标状态位置  rad
 */
void Robot::IndependentControl(void)
{
    XVAS temp[6];
    double PositionStart[6];
    double TargetPosition[6] = { 0 };
    double CurrentPeriod_P[6] = { 0 };
    int i, j;
    int flag = 0;
    int step[6] = { 0 };
    double CurrentPeriod_t[6] = { 0 };
    double IsTurnPoint = 0;
//    double am[8] = { 0 };
//    double angle=0;
    BroadCast->QueryPos();
    QThread::msleep(1000);
    for (int j = 0; j < 6; j++)
    { 
        PositionStart[j] = Leg[j]->getMotorPos()*2*pi/Leg_Reduction[j] / Leg[j]->EncoderCnt / 4;
//        qDebug()<<QString("PositionStart[%1]:%2").arg(j).arg(PositionStart[j]);
        Leg[j]->EnableMotor(true);
    }
    QThread::msleep(2000);
    QDateTime begin_time =QDateTime::currentDateTime();
    QDateTime now_time = begin_time;
    double t = begin_time.msecsTo(now_time);
    //步态循环 全部腿部到达目标位置后终止
    while (flag!=0x3F)
    {
        now_time = QDateTime::currentDateTime();
        t = begin_time.msecsTo(now_time);
         //qDebug()<<"t:"<<t;
//		FeedBack();
        for (j = 0; j < 6; j++)
        {
            if (t < (Param->PD[j]* Param->T[j]))
            {
                continue;
            }
            else if (t >= (Param->PD[j] * Param->T[j]) && t <= (Param->T[j] * (Param->StepNum + Param->PD[j])))
            {
                // 当前运动到的步数
                step[j] = (int)((t - Param->PD[j] * Param->T[j]) / Param->T[j]);
                // 当前周期内的时间变量t
                CurrentPeriod_t[j] = fmod((t - Param->PD[j] * Param->T[j]), Param->T[j]);

                //中途反转
                if (Param->TP[j] != 0.0 )
                {
                    if (CurrentPeriod_t[j] <= Param->TT[j])
                    {
                        TrigonalAcc(Param->TP[j], CurrentPeriod_t[j] / 1000., Param->TT[j] / 1000., &temp[j]);
                    }
                    else if (CurrentPeriod_t[j] > Param->TT[j])
                    {
                        TrigonalAcc(Param->TP[j] - Param->PW[j], (CurrentPeriod_t[j] - Param->TT[j]) / 1000., (Param->T[j] - Param->TT[j]) / 1000., &temp[j]);

                    }
                    if((int)(fmod((t - Param->PD[j] * Param->T[j]), Param->T[j]) / Param->TT[j]))
                        IsTurnPoint = 1 ;
                    CurrentPeriod_P[j] = PositionStart[j] + IsTurnPoint * Param->TP[j] + temp[j].x;

                    TargetPosition[j] = CurrentPeriod_P[j] + step[j] * (2 * Param->TP[j] - Param->PW[j]);
                }
                else
                {
                    TrigonalAcc(Param->PW[j], CurrentPeriod_t[j] / 1000., Param->T[j] / 1000., &temp[j]);
                    CurrentPeriod_P[j] = PositionStart[j] + temp[j].x;
                    TargetPosition[j] = CurrentPeriod_P[j] + step[j] * Param->PW[j];

                }

            }
            else
            {
                flag |= (0x01 << j);
            }
        }
        for (j = 0; j < 6; j++)
        {
            if (t > (Param->PD[j] * Param->T[j]) && t < (Param->T[j] * (Param->StepNum + Param->PD[j])))
            {
                int count = (int)(TargetPosition[j] * Leg_Reduction[j] * Leg[j]->EncoderCnt * 4 / (2*pi));
//                qDebug()<<QString("Leg[%1] count: %2 TargetAngle: %3").arg(j).arg(count).arg(TargetPosition[j]);
                //Leg[j]->SetRunByPosSpd((float)(temp[j].v/2/pi* Leg_Reduction[j] * Leg[j]->EncoderCnt * 4));
                Leg[j]->RunByPosA(count);
            }
        }

        //qDebug() << "IndependentControl t:" << t;
        //qDebug() << "The yaw angle is  " <<gyro->getValues();
    }
}


/**
 * @brief  从复位状态切换至准备运动状态
 * @input  T              该动作周期    ms
 *         PositionWant   目标状态位置  rad
 */
void Robot::SetControlParam(ControlParam* P)
{
    Param = P;
}

/**
 * @brief  修正梯形加速度规划
 * @input  最大加速度 当前时间戳 周期
 * @ouput  加速度a m/s^2  速度v 位移x 冲击度s
 */
void Robot::TrapezoidalAcc(double am, double t, double T, XVAS* xvas)
{
    XVAS temp;
        if (t <= (T / 8.))
        {
            xvas->a = am / 2 * (1 - cos(8 * pi * (1. / T) * t));
            xvas->s = (4 * am * pi * sin((8 * pi * t) / T)) / T;
            xvas->v = (am * t) / 2. - (T * am * sin((8 * pi * t) / T)) / (16 * pi);
            xvas->x = (am * t * t ) / 4. + (T * T * am * cos((8 * pi * t) / T)) / (128 * pi * pi);
        }
        else if (t <= (3. / 8 * T))
        {
            TrapezoidalAcc(am, T / 8., T, &temp);
            xvas->a = am;
            xvas->s = 0;
            xvas->v = am * t + (temp.v - am * T / 8.);
            xvas->x = (am * t * t) / 2 + (temp.v - am * T / 8.) * t + (am * T * T) / 128. - (temp.v * T) / 8. + temp.x;
        }
        else if (t <= (5. / 8 * T))
        {
            TrapezoidalAcc(am, T * 3 / 8., T, &temp);
            xvas->a = am * cos(4 * pi / T * (t - 3 / 8. * T));
            xvas->s = (4 * am * pi * sin((4 * pi * ((3 * T) / 8 - t)) / T)) / T;
            xvas->v = -(T * am * sin((4 * pi * ((3 * T) / 8 - t)) / T)) / (4 * pi) + temp.v;
            xvas->x = -(T * T * am * cos((4 * pi * ((3 * T) / 8 - t)) / T)) / (16 * pi * pi) + temp.v * t + (0.0063 * am * T * T) - (3 * temp.v * T) / 8. + temp.x;
        }
        else if (t <= (7. / 8 * T))
        {
            TrapezoidalAcc(am, T * 5 / 8., T, &temp);
            xvas->a = -am;
            xvas->s = 0;
            xvas->v = -am * t + (temp.v + am * 5 * T / 8.);
            xvas->x = -(am * t * t) / 2. + (temp.v + am * 5 * T / 8.) * t + temp.x - (5 * T * temp.v) / 8. - (25 * T * T * am) / 128.;
        }

        else
        {
            TrapezoidalAcc(am, T * 7 / 8., T, &temp);
            xvas->a = -am / 2 * (1 + cos(8 * pi / T * (t - 7 / 8. * T)));
            xvas->s = -(4 * am * pi * sin((8 * pi * ((7 * T) / 8. - t)) / T)) / T;
            xvas->v = (T * am * sin((8 * pi * ((7 * T) / 8. - t)) / T)) / (16 * pi) - (am * t) / 2 + temp.v + (7 * T * am) / 16.;
            xvas->x = (T * T * am * cos((8 * pi * ((7 * T) / 8. - t)) / T)) / (128 * pi * pi) - (am * t * t) / 4. + (temp.v + (7 * T * am) / 16.) * t + temp.x - (7 * T * temp.v) / 8. - (0.1922 * T * T * am);
        }
}

/**
 * @brief  三角形加速度规划
 * @input  最大加速度 当前时间戳 周期
 * @ouput  加速度a m/s^2  速度v 位移x 冲击度s
 */
void Robot::TrigonalAcc(double PositionWant, double t, double T, XVAS* xvas)
{
    double am = 8 * PositionWant / (T * T);
    if (t <= T / 4)
    {
        xvas->a = t * 4 / T * am;
        xvas->v = (2 * am * t * t) / T;
        xvas->x = (2 * am * t *t * t) / (3 * T);
    }

    else if (t <= 3 * T / 4.)
    {
        xvas->a = (T / 2 - t) * 4 / T * am;
        xvas->v = -(am * (T - 2 * t) * (T - 2 * t)) / (2 * T) + am * T / 4;
        xvas->x = -(am * (T - 2 * t) * (3 * T * T - 2 * (T - 2 * t) * (T - 2 * t))) / (24 * T) + (T * T * am) / 16;
    }

    else
    {
        xvas->a = -(T - t) * 4 / T * am;
        xvas->v = (2 * am * (T - t) * (T - t)) / T;
        xvas->x = -(2 * am * (T - t) * (T - t) * (T - t)) / (3 * T) + PositionWant;
    }
}
