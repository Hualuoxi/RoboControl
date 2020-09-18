#include "robodriver.h"

RoboDriver::RoboDriver(Motor * leg[],Motor* broadCast)
{

    for(int i = 0; i < 6; i++)
    {
        Leg[i]=leg[i];
    }
    BroadCast = broadCast;
}


void RoboDriver::run()
{
    qDebug()<<"Robot run!"<<QThread::currentThread();

//    PVTTest();
//    SearchZero();
    //PVTRunTest();
//    PVTReady(pi*1.6,2000);
//    PVTRotate(2000,5);
//    PVTRun(2000,2000,0,50);
    //Rotate(2500,2);
    //GoForward(1000, 3);
    //PVTReady(-pi*1.6,2000);
    QThread::msleep(5000);
    BroadCast->EnableMotor(false);
}


void RoboDriver::PVTRunTest()
{
    int id = 1;
    Leg[id]->EnableMotor(true);
    QThread::msleep(10);
    PVT_Prama mPVT;
    int T=1000;
    mPVT.tim = T/10.;
    for(int i=0;i<10;i++)
    {
        if(i<2)
        {
            mPVT.vel = 2*pi/3/T;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
        else if(i<5)
        {
            mPVT.vel = 2*pi/3/T+64*pi/3/T/T*(i-2)*mPVT.tim;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
        else if(i<8)
        {
            mPVT.vel = 2*pi/3/T+64*pi/3/T/T*(8-i)*mPVT.tim;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
        else
        {
            mPVT.vel = 2*pi/3/T;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
//        qDebug()<<QString("mPVT.pos = %1 , mPVT.vel = %2 , mPVT.tim = %3").arg(mPVT.pos).arg(mPVT.vel).arg(mPVT.tim)<<endl;
        Leg[id]->PVTPrama_Mutex.lock();
        Leg[id]->PVTQueue.enqueue(mPVT);
        Leg[id]->PVTPrama_Mutex.unlock();
    }
    for(int i = 0;i<4;i++)
    {
        Leg[id]->PVTPrama_Mutex.lock();
        mPVT = Leg[id]->PVTQueue.dequeue();
        Leg[id]->PVTPrama_Mutex.unlock();
        Leg[id]->SendPVT(Leg[id]->Rad2Cnt(mPVT.pos),Leg[id]->Rad2Cnt(mPVT.vel*1000),(u8)mPVT.tim);
    }
    Leg[id]->RunBypPVT(Leg[id]->getRPtr());
    QThread::msleep(10);
    Leg[id]->Begin();
    QThread::msleep(2000);
}

void RoboDriver::PVTTest()
{
    int id = 1;
    Leg[id]->EnableMotor(true);
    QThread::msleep(10);
    PVT_Prama mPVT;
    int T=3000;
    mPVT.tim = T/64.;
    for(int i=0;i<=64;i++)
    {
        if(i<16)
        {
            mPVT.vel = 2*pi/3/T;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
        else if(i<32)
        {
            mPVT.vel = 2*pi/3/T+64*pi/3/T/T*(i-16)*mPVT.tim;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
        else if(i<48)
        {
            mPVT.vel = 2*pi/3/T+64*pi/3/T/T*(48-i)*mPVT.tim;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
        else
        {
            mPVT.vel = 2*pi/3/T;
            mPVT.pos += mPVT.vel * mPVT.tim;
        }
//        qDebug()<<QString("mPVT.pos = %1 , mPVT.vel = %2 , mPVT.tim = %3").arg(mPVT.pos).arg(mPVT.vel).arg(mPVT.tim)<<endl;
        Leg[id]->PVTPrama_Mutex.lock();
        Leg[id]->PVTQueue.enqueue(mPVT);
        Leg[id]->PVTPrama_Mutex.unlock();
    }
    for(int i = 0;i<10;i++)
    {
        Leg[id]->PVTPrama_Mutex.lock();
        mPVT = Leg[id]->PVTQueue.dequeue();
        Leg[id]->PVTPrama_Mutex.unlock();
        Leg[id]->SendPVT(Leg[id]->Rad2Cnt(mPVT.pos),Leg[id]->Rad2Cnt(mPVT.vel*1000),(u8)mPVT.tim);
    }
    Leg[id]->RunBypPVT(Leg[id]->getRPtr());
    QThread::msleep(10);
    Leg[id]->Begin();
    QThread::msleep(5000);
}
/**
 * @brief  从复位状态切换至准备运动状态
 * @input  T              该动作周期    ms
 *         PositionWant   目标状态位置  rad
 */
void RoboDriver::Ready(float T, float PositionWant)
{
    qDebug()<<"Robot Ready";

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
//    if(PositionWant>0)
//        parameter.PW[2]=PositionWant+0.1*pi;
//    else
//        parameter.PW[2]=PositionWant-0.1*pi;
    SetControlParam(&parameter);
    IndependentControl();
}

void RoboDriver::SearchZero()
{
    qDebug()<<"Robot SearchZero";
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
    BroadCast->EnableMotor(true);

}



/**
 * @brief  向前走
 * @input  T              该动作周期    ms
 *         num            步数
 */
void RoboDriver::GoForward(float T, int num)
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

void RoboDriver::Rotate(float T, int num)
{
    ControlParam parameter;

    for (int i = 0; i < 6; i++)
        parameter.T[i] = T;


    parameter.PW[0] = 2 * pi;
    parameter.PW[1] = -2 * pi;
    parameter.PW[2] = 2 * pi;
    parameter.PW[3] = -2 * pi;
    parameter.PW[4] = 2 * pi;
    parameter.PW[5] = -2 * pi;

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
void RoboDriver::BipedRun()
{
    //前跳实验
    ControlParam Param;
    int i = 0;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    SetControlParam(&Param);
    IndependentControl();
    for (int i = 0; i < 6; i++)
        Param.T[i] = 500;

    i = 0;
    Param.PW[i++] = -1.7f * pi;
    Param.PW[i++] = -1.7f * pi;
    Param.PW[i++] = -1.7f * pi;
    Param.PW[i++] = -1.7f * pi;
    Param.PW[i++] = -1.6f * pi;
    Param.PW[i++] = -1.6f * pi;

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
    Param.PD[5] = 0.5f;
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
void RoboDriver::RollingOver()
{
    //前跳实验
    ControlParam Param;
    int i = 0;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    Param.PW[i++] = 0.5f * pi;
    SetControlParam(&Param);
    IndependentControl();

    for (int i = 0; i < 6; i++)
        Param.T[i] = 500;

    i = 0;
    Param.PW[i++] = -2.3f * pi;
    Param.PW[i++] = -2.3f * pi;
    Param.PW[i++] = -2.3f * pi;
    Param.PW[i++] = -2.3f * pi;
    Param.PW[i++] = -2.3f * pi;
    Param.PW[i++] = -2.3f * pi;

    i = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0;
    Param.PD[i++] = 0.5f;
    Param.PD[i++] = 0.5f;

    SetControlParam(&Param);
    IndependentControl();
}

/**
 * @brief  从复位状态切换至准备运动状态
 * @input  T              该动作周期    ms
 *         PositionWant   目标状态位置  rad
 */
void RoboDriver::IndependentControl(void)
{
    XVAS temp[6];
    float PositionStart[6];
    float TargetPosition[6] = { 0 };
    float CurrentPeriod_P[6] = { 0 };
    int j;
    int flag = 0;
    int step[6] = { 0 };
    float CurrentPeriod_t[6] = { 0 };
    float IsTurnPoint = 0;
//    double am[8] = { 0 };
//    double angle=0;
    BroadCast->QueryPos();
    QThread::msleep(100);
    for (int j = 0; j < 6; j++)
    {
        PositionStart[j] = Leg[j]->Cnt2Rad(Leg[j]->getMotorPos());
//        qDebug()<<QString("PositionStart[%1]:%2").arg(j).arg(PositionStart[j]);
        Leg[j]->EnableMotor(true);
    }
    QThread::msleep(100);
    QDateTime begin_time =QDateTime::currentDateTime();
    QDateTime now_time = begin_time;
    float t = begin_time.msecsTo(now_time);
    float t_old = t;
    //步态循环 全部腿部到达目标位置后终止
    while (flag!=0x3F)
    {
        now_time = QDateTime::currentDateTime();
        t_old = t;
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
                CurrentPeriod_t[j] = (float)fmod((double)(t - Param->PD[j] * Param->T[j]), (double)(Param->T[j]));

                //中途反转
                if (Param->TP[j] != 0.0f )
                {
                    if (CurrentPeriod_t[j] <= Param->TT[j])
                    {
                        TrigonalAcc(Param->TP[j], CurrentPeriod_t[j] / 1000.f, Param->TT[j] / 1000.f, &temp[j]);
                    }
                    else if (CurrentPeriod_t[j] > Param->TT[j])
                    {
                        TrigonalAcc(Param->TP[j] - Param->PW[j], (CurrentPeriod_t[j] - Param->TT[j]) / 1000.f, (Param->T[j] - Param->TT[j]) / 1000.f, &temp[j]);

                    }
                    if((int)(fmod((t - Param->PD[j] * Param->T[j]), Param->T[j]) / Param->TT[j]))
                        IsTurnPoint = 1 ;
                    CurrentPeriod_P[j] = PositionStart[j] + IsTurnPoint * Param->TP[j] + temp[j].x;

                    TargetPosition[j] = CurrentPeriod_P[j] + step[j] * (2 * Param->TP[j] - Param->PW[j]);
                }
                else
                {
                    TrigonalAcc(Param->PW[j], CurrentPeriod_t[j] / 1000.f, Param->T[j] / 1000.f, &temp[j]);
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
                int count = (int)Leg[j]->Rad2Cnt(TargetPosition[j]);
//                int speed = (int)Leg[j]->Red2Cnt(temp[j].v);
//                Leg[j]->SendPVT(count,speed,(int)(t-t_old));
//                if(abs(Leg[j]->getHPtr() - Leg[j]->getTPtr()))
//                {
//                    Leg[j]->RunBypPVT(Leg[j]->getTPtr());
//                }
                //if(Leg[j]getWritePtr>)
//                qDebug()<<QString("Leg[%1] count: %2 TargetAngle: %3").arg(j).arg(count).arg(TargetPosition[j]);
                //Leg[j]->SetRunByPosSpd((float)(temp[j].v/2/pi* Leg_Reduction[j] * Leg[j]->EncoderCnt * 4));
                Leg[j]->RunByPosA(count);
            }
        }
        BroadCast->Begin();
        //QThread::msleep(1);
        //qDebug() << "IndependentControl t:" << t;
        //qDebug() << "The yaw angle is  " <<gyro->getValues();
    }
    int Target[6]={0};
    bool wait = true;
    for (j = 0; j < 6; j++)
    {
        Target[j] =(int)Leg[j]->Rad2Cnt(PositionStart[j]+Param->StepNum * Param->PW[j]); // (int)((PositionStart[j]+Param->StepNum * Param->PW[j]) * Leg[j]->Leg_Reduction * Leg[j]->EncoderCnt * 4 / (2*pi));
    }
    do
    {
        for (j = 0; j < 6; j++)
        {
            Leg[j]->RunByPosA(Target[j]);
        }
        BroadCast->Begin();
        QThread::msleep(100);
        BroadCast->QueryPos();
        QThread::msleep(100);
        if(abs(Target[0]-Leg[0]->getMotorPos())<10&&
                        abs(Target[1]-Leg[1]->getMotorPos())<10&&
                        abs(Target[2]-Leg[2]->getMotorPos())<10&&
                        abs(Target[3]-Leg[3]->getMotorPos())<10&&
                        abs(Target[4]-Leg[4]->getMotorPos())<10&&
                        abs(Target[5]-Leg[5]->getMotorPos())<10)
                    wait = false;
    }while(wait);
}


/**
 * @brief  从复位状态切换至准备运动状态l
 * @input  T              该动作周期    ms
 *         PositionWant   目标状态位置  rad
 */
void RoboDriver::SetControlParam(ControlParam* P)
{
    Param = P;
}


/**
 * @brief  修正梯形加速度规划
 * @input  最大加速度 当前时间戳 周期
 * @ouput  加速度a m/s^2  速度v 位移x 冲击度s
 */
void RoboDriver::TrapezoidalAcc(float am, float t, float T, XVAS* xvas)
{
    XVAS temp;
        if (t <= (T / 8.f))
        {
            xvas->a = am / 2.f * (1.f - cos(8.f * pi * (1.f / T) * t));
            xvas->s = (4.f * am * pi * sin((8 * pi * t) / T)) / T;
            xvas->v = (am * t) / 2. - (T * am * sin((8 * pi * t) / T)) / (16 * pi);
            xvas->x = (am * t * t ) / 4. + (T * T * am * cos((8 * pi * t) / T)) / (128 * pi * pi);
        }
        else if (t <= (3. / 8 * T))
        {
            TrapezoidalAcc(am, T / 8.f, T, &temp);
            xvas->a = am;
            xvas->s = 0;
            xvas->v = am * t + (temp.v - am * T / 8.);
            xvas->x = (am * t * t) / 2 + (temp.v - am * T / 8.) * t + (am * T * T) / 128. - (temp.v * T) / 8. + temp.x;
        }
        else if (t <= (5. / 8 * T))
        {
            TrapezoidalAcc(am, T * 3 / 8.f, T, &temp);
            xvas->a = am * cos(4 * pi / T * (t - 3 / 8. * T));
            xvas->s = (4 * am * pi * sin((4 * pi * ((3 * T) / 8 - t)) / T)) / T;
            xvas->v = -(T * am * sin((4 * pi * ((3 * T) / 8 - t)) / T)) / (4 * pi) + temp.v;
            xvas->x = -(T * T * am * cos((4 * pi * ((3 * T) / 8 - t)) / T)) / (16 * pi * pi) + temp.v * t + (0.0063 * am * T * T) - (3 * temp.v * T) / 8. + temp.x;
        }
        else if (t <= (7. / 8 * T))
        {
            TrapezoidalAcc(am, T * 5 / 8.f, T, &temp);
            xvas->a = -am;
            xvas->s = 0;
            xvas->v = -am * t + (temp.v + am * 5 * T / 8.);
            xvas->x = -(am * t * t) / 2. + (temp.v + am * 5 * T / 8.) * t + temp.x - (5 * T * temp.v) / 8. - (25 * T * T * am) / 128.;
        }

        else
        {
            TrapezoidalAcc(am, T * 7 / 8.f, T, &temp);
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
void RoboDriver::TrigonalAcc(float PositionWant, float t, float T, XVAS* xvas)
{
    double am = 8.f* PositionWant / (T * T);
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




void RoboDriver::PVTGoForward(float T, int num)
{
    float PositionStart[6];
    BroadCast->QueryPos();
    QThread::msleep(100);
    for (int j = 0; j < 6; j++)
    {
        PositionStart[j] = Leg[j]->Cnt2Rad(Leg[j]->getMotorPos());
        Leg[j]->EnableMotor(true);
    }
    QThread::msleep(100);


    PVT_Prama mPVT;
    PVT_Prama mPVT_temp;
    mPVT.tim = T/64.;
    bool Begin1 = true;
    bool Begin2 = true;
    bool preSend = true;
    u8 step=0;

    QDateTime begin_time =QDateTime::currentDateTime();
    QDateTime now_time = begin_time;
    int t = (int)begin_time.msecsTo(now_time);
    while(t<(T*(num+0.5)))
    {
        now_time =QDateTime::currentDateTime();
        t = (int)begin_time.msecsTo(now_time);
        if(step<num&&t>=(step-0.5)*T)
        {
            step++;
            qDebug()<<QString("step=%1,t=%2").arg(step).arg(t);
            mPVT.pos=0;
            mPVT.tim = T/64.;
            mPVT.vel=0;
            for(int i=0;i<=64;i++)
            {
                if(i<16)
                {
                    mPVT.vel = 2*pi/3/T;
                    mPVT.pos += mPVT.vel * mPVT.tim;
                }
                else if(i<32)
                {
                    mPVT.vel = 2*pi/3/T+64*pi/3/T/T*(i-16)*mPVT.tim;
                    mPVT.pos += mPVT.vel * mPVT.tim;
                }
                else if(i<48)
                {
                    mPVT.vel = 2*pi/3/T+64*pi/3/T/T*(48-i)*mPVT.tim;
                    mPVT.pos += mPVT.vel * mPVT.tim;
                }
                else
                {
                    mPVT.vel = 2*pi/3/T;
                    mPVT.pos += mPVT.vel * mPVT.tim;
                }
        //        qDebug()<<QString("mPVT.pos = %1 , mPVT.vel = %2 , mPVT.tim = %3").arg(mPVT.pos).arg(mPVT.vel).arg(mPVT.tim)<<endl;
                mPVT_temp = mPVT;
                for(int j=0;j<6;j++)
                {
                    mPVT_temp.pos=mPVT.pos+2*pi*(step-1)+PositionStart[j];
                    Leg[j]->PVTPrama_Mutex.lock();
                    Leg[j]->PVTQueue.enqueue(mPVT_temp);
                    Leg[j]->PVTPrama_Mutex.unlock();
                }
            }
        }
        if(preSend)
        {
            qDebug()<<"preSend";
            for(int j=0;j<6;j++)
            {
                for(int i = 0;i<10;i++)
                {
                    Leg[j]->PVTPrama_Mutex.lock();
                    if(!Leg[j]->PVTQueue.empty())
                    {
                        mPVT = Leg[j]->PVTQueue.dequeue();
                        Leg[j]->SendPVT(Leg[j]->Rad2Cnt(mPVT.pos),Leg[j]->Rad2Cnt(mPVT.vel*1000),(u8)mPVT.tim);
                    }
                    Leg[j]->PVTPrama_Mutex.unlock();

                }
            }
            preSend = false;

        }
        if(Begin1)
        {
            qDebug()<<QString("Begin1,t=%1").arg(t);
            Leg[0]->RunBypPVT(Leg[0]->getRPtr());
            Leg[3]->RunBypPVT(Leg[3]->getRPtr());
            Leg[4]->RunBypPVT(Leg[4]->getRPtr());

            Leg[0]->Begin();
            Leg[3]->Begin();
            Leg[4]->Begin();
            Begin1 = false;
        }
        if(t>0.5*T&&Begin2)
        {
            qDebug()<<QString("Begin2,t=%1").arg(t);
            Leg[1]->RunBypPVT(Leg[1]->getRPtr());
            Leg[2]->RunBypPVT(Leg[2]->getRPtr());
            Leg[5]->RunBypPVT(Leg[5]->getRPtr());

            Leg[1]->Begin();
            Leg[2]->Begin();
            Leg[5]->Begin();
            Begin2 = false;
        }

    }
}

void RoboDriver::getRefPos(float Pos[])
{
    BroadCast->QueryPos();
    QThread::msleep(100);
    for (int j = 0; j < 6; j++)
    {
        Pos[j] = Leg[j]->getMotorPos();
        QThread::msleep(10);
        Leg[j]->EnableMotor(true);
    }
    QThread::msleep(100);
}

void RoboDriver::PVTRun(float Tb,float Te, int Anum,int num)
{
    float PositionStart[6];
    PVT_Prama mPVT[PVTPnt];

    PVT_Prama mPVT_temp;
    bool Begin1 = true;
    bool Begin2 = true;
    bool preSend = true;
    float step=0;
    float T=0;
    float tempT=0;

    getRefPos(PositionStart);
    QDateTime begin_time =QDateTime::currentDateTime();
    QDateTime now_time = begin_time;
    int t = (int)begin_time.msecsTo(now_time);
    while(t<((Tb*Anum-(Anum-1)/2.*(Tb-Te)+(num-Anum)*Te)+T*0.5))
    {
        now_time =QDateTime::currentDateTime();
        t = (int)begin_time.msecsTo(now_time);
//        qDebug()<<"t:"<<t;
        if(Anum>1.0)
        {
            if(step<Anum)
                tempT=Tb-(Tb-Te)/Anum*step;
            else
                tempT=Te;
        }
        else
            tempT=Tb;
        if(step<num&&t>=(step-0.5f)*T)
        {
//            qDebug()<<QString("step=%1,t=%2").arg(step).arg(t);
            for(int j=0;j<6;j++)
            {
                if(step<Anum)
                {
                    if(j==0||j==3||j==4)
                        T = tempT;
                    else
                        T = tempT - (Tb-Te)/Anum/2.;
                }
                else
                    T = tempT;

                PVTTrigonalAcc(2*pi,T,j,mPVT);
                for(int i=0;i<PVTPnt;i++)
                {
                    mPVT_temp = mPVT[i];
                    mPVT_temp.pos= mPVT[i].pos + step * Leg[j]->EncoderCnt*4*Leg[j]->Leg_Reduction + PositionStart[j];
                    mPVT_temp.vel*=1000;
                    Leg[j]->PVTPrama_Mutex.lock();
                    Leg[j]->PVTQueue.enqueue(mPVT_temp);
                    Leg[j]->PVTPrama_Mutex.unlock();
                }
            }
            step++;
        }
        if(preSend)
        {
            qDebug()<<"preSend";
            for(int j=0;j<6;j++)
            {
                for(int i = 0;i<4;i++)
                {
                    Leg[j]->PVTPrama_Mutex.lock();
                    if(!Leg[j]->PVTQueue.empty())
                    {
                        mPVT_temp = Leg[j]->PVTQueue.dequeue();
                        Leg[j]->SendPVT((int)(mPVT_temp.pos),(int)(mPVT_temp.vel),(u8)mPVT_temp.tim);
                    }
                    Leg[j]->PVTPrama_Mutex.unlock();

                }
            }
            preSend = false;

        }
        if(Begin1)
        {
            qDebug()<<QString("Begin1,t=%1").arg(t);
            Leg[0]->RunBypPVT(Leg[0]->getRPtr());
            Leg[3]->RunBypPVT(Leg[3]->getRPtr());
            Leg[4]->RunBypPVT(Leg[4]->getRPtr());

            Leg[0]->Begin();
            Leg[3]->Begin();
            Leg[4]->Begin();
            Begin1 = false;
        }
        if(t>0.5*T&&Begin2)
        {
            qDebug()<<QString("Begin2,t=%1").arg(t);
            Leg[1]->RunBypPVT(Leg[1]->getRPtr());
            Leg[2]->RunBypPVT(Leg[2]->getRPtr());
            Leg[5]->RunBypPVT(Leg[5]->getRPtr());

            Leg[1]->Begin();
            Leg[2]->Begin();
            Leg[5]->Begin();
            Begin2 = false;
        }

    }
}

void RoboDriver::setPVTPrama(PVTPP *prama)
{
    PVTPrama = prama;
}
void RoboDriver::PVTControl(void)
{
    float PositionStart[6];
    PVT_Prama mPVT[PVTPnt];
    PVT_Prama mPVT_temp;
    bool preSend[6] = {true,true,true,true,true,true};
    float step[6]={0};
    float T_cal=0;
    getRefPos(PositionStart);
    u8 StopFlag =0;
    QDateTime begin_time =QDateTime::currentDateTime();
    QDateTime now_time = begin_time;
    int t = (int)begin_time.msecsTo(now_time);
    //步态循环 全部腿部到达目标位置后终止
    while(StopFlag!=0x3F)
    {
        now_time =QDateTime::currentDateTime();
        t = (int)begin_time.msecsTo(now_time);
        for (int j = 0; j < 6; j++)
        {
            //步态 周期、相位差控制
            if(PVTPrama->StepAcc[j])
            {
                if(step[j]<PVTPrama->StepAcc[j])
                    PVTPrama->T[j] = PVTPrama->Tb[j] - (PVTPrama->Tb[j] - PVTPrama->Te[j]) / PVTPrama->StepAcc[j] * step[j];
                else
                    PVTPrama->T[j]=PVTPrama->Te[j];
                if(PVTPrama->KeepPD[j])
                    PVTPrama->T[j] = PVTPrama->T[j] - (PVTPrama->Tb[j] - PVTPrama->Te[j])/PVTPrama->StepAcc[j]/2.;
                //判断是否开始计算下一周期所用中间变量
                T_cal = (PVTPrama->Tb[j]*step[j]+(step[j]-1)*step[j]/2.*(PVTPrama->Tb[j] - PVTPrama->Te[j])/PVTPrama->StepAcc[j]) - 0.5f*PVTPrama->T[j];
            }
            else
                T_cal = PVTPrama->T[j]*(step[j]-0.5f);

            if((step[j] < PVTPrama->StepNum[j]) && (t >=T_cal))
            {
                PVTTrigonalAcc(PVTPrama->PosWant[j],PVTPrama->T[j],j,mPVT);
                for(int i=0;i<PVTPnt;i++)
                {
                    mPVT_temp = mPVT[i];
                    mPVT_temp.pos= mPVT[i].pos + step[j] * PVTPrama->PosWant[j] * Leg[j]->EncoderCnt*4*Leg[j]->Leg_Reduction /2./pi + PositionStart[j];
                    mPVT_temp.vel*=1000;
                    Leg[j]->PVTPrama_Mutex.lock();
                    Leg[j]->PVTQueue.enqueue(mPVT_temp);
                    Leg[j]->PVTPrama_Mutex.unlock();
                }
                step[j]++;
            }
            if(preSend[j])
            {
                for(int i = 0;i<PVTPnt/2;i++)
                {
                    Leg[j]->PVTPrama_Mutex.lock();
                    if(!Leg[j]->PVTQueue.empty())
                    {
                        mPVT_temp = Leg[j]->PVTQueue.dequeue();
                        Leg[j]->SendPVT((int)(mPVT_temp.pos),(int)(mPVT_temp.vel),(u8)mPVT_temp.tim);
                    }
                    Leg[j]->PVTPrama_Mutex.unlock();
                }
                Leg[j]->RunBypPVT(Leg[j]->getRPtr());
                preSend[j] = false;
            }
        }

        for (int j = 0; j < 6; j++)
        {
            // 启动
            if(PVTPrama->Bg[j] && (t > PVTPrama->T[j]*PVTPrama->PhaseDiff[j]))
            {

                Leg[j]->Begin();
                PVTPrama->Bg[j] = false;
            }
            // 终止
            if(PVTPrama->StepAcc[j])
            {
                if(t > ((PVTPrama->Tb[j]*PVTPrama->StepAcc[j]-(PVTPrama->StepAcc[j]-1)/2.*(PVTPrama->Tb[j]-PVTPrama->Te[j])+(PVTPrama->StepNum[j]-PVTPrama->StepAcc[j])*PVTPrama->Te[j])+PVTPrama->T[j]*PVTPrama->PhaseDiff[j]))
                    StopFlag |= (0x01<<j);
            }
            else
            {
                if(t > (PVTPrama->T[j]*(PVTPrama->StepNum[j]+PVTPrama->PhaseDiff[j])))
                    StopFlag |= (0x01<<j);
            }
            // 持续运动
            if(PVTPrama->ContinueRun[j])
                if(step[j] < PVTPrama->StepNum[j])
                    StopFlag |= (0x01<<j);

        }
    }
}
void RoboDriver::PVTReady(float PosWant,float T)
{
    PVTPP prama;
    for(int i = 0;i<6;i++)
       prama.T[i] = T;
    for(int i = 0;i<6;i++)
       prama.PosWant[i] = PosWant;
    setPVTPrama(&prama);
    PVTControl();

}
void RoboDriver::PVTRotate(float T, int num)
{
    PVTPP prama;

    for (int i = 0; i < 6; i++)
        prama.T[i] = T;
    for (int i = 0; i < 6; i++)
        prama.StepNum[i] = num;
    int i=0;
    prama.PosWant[i++] = 2 * pi;
    prama.PosWant[i++] = -2 * pi;
    prama.PosWant[i++] = 2 * pi;
    prama.PosWant[i++] = -2 * pi;
    prama.PosWant[i++] = 2 * pi;
    prama.PosWant[i++] = -2 * pi;

    i=0;
    prama.PhaseDiff[i++] = 0.0;
    prama.PhaseDiff[i++] = 0.5;
    prama.PhaseDiff[i++] = 0.5;
    prama.PhaseDiff[i++] = 0.0;
    prama.PhaseDiff[i++] = 0.0;
    prama.PhaseDiff[i++] = 0.5;

    setPVTPrama(&prama);
    PVTControl();
}


/**
 * @brief  翻转
 */
void RoboDriver::PVTRollingOver()
{
    PVTPP prama;

    for(int i = 0;i<6;i++)
       prama.PosWant[i] = 0.5 * pi;
    setPVTPrama(&prama);
    PVTControl();

    for (int i = 0; i < 6; i++)
    {
        prama.T[i] = 500;
        prama.PosWant[i] = -2.3f * pi;
        prama.Bg[i] = true;
    }

    prama.PhaseDiff[0] = 0.5f;
    prama.PhaseDiff[1] = 0.5f;

    setPVTPrama(&prama);
    PVTControl();
}

/*
void RoboDriver::PVTReady(float PosWant,float T)
{
    float PositionStart[6];
    PVT_Prama mPVT[PVTPnt];
    PVT_Prama mPVT_temp;
    getRefPos(PositionStart);

    QDateTime begin_time =QDateTime::currentDateTime();
    QDateTime now_time = begin_time;
    int t = (int)begin_time.msecsTo(now_time);

    for(int j=0;j<6;j++)
    {
        if(j==2)
        {
            if(PosWant>0)
                PVTTrigonalAcc(PosWant+0.05f,T,j,mPVT);
            else
                PVTTrigonalAcc(PosWant-0.05f,T,j,mPVT);
        }
        else
            PVTTrigonalAcc(PosWant,T,j,mPVT);
        for(int i=0;i<PVTPnt;i++)
        {
            mPVT_temp = mPVT[i];
            mPVT_temp.pos= mPVT[i].pos + PositionStart[j];
            mPVT_temp.vel*=1000;
            Leg[j]->PVTPrama_Mutex.lock();
            Leg[j]->PVTQueue.enqueue(mPVT_temp);
            Leg[j]->PVTPrama_Mutex.unlock();
        }
    }
    qDebug()<<"preSend";
    for(int j=0;j<6;j++)
    {
        for(int i = 0;i<4;i++)
        {
            Leg[j]->PVTPrama_Mutex.lock();
            if(!Leg[j]->PVTQueue.empty())
            {
                mPVT_temp = Leg[j]->PVTQueue.dequeue();
                Leg[j]->SendPVT((int)(mPVT_temp.pos),(int)(mPVT_temp.vel),(u8)mPVT_temp.tim);
            }
            Leg[j]->PVTPrama_Mutex.unlock();

        }
        Leg[j]->RunBypPVT(Leg[j]->getRPtr());
    }
    for(int j=0;j<6;j++)
    {
        Leg[j]->Begin();
    }
    while(t<(T+T/PVTPnt))
    {
        now_time =QDateTime::currentDateTime();
        t = (int)begin_time.msecsTo(now_time);
        QThread::msleep(1);
    }
}
*/



/**l
 * @brief  PVT三角形加速度规划
 * @input  PosWant T index
 * @ouput  PVT_Prama
 *
 *      ^
 *      |
 *      |                /\
 *      |               /  \
 *      |              /    \
 *      |             /      \
 *      |            /        \
 *      |           /          \
 *      |          /            \
 *      -----------              -----------
 *      |
 *      |
 *      ------------------------------------>
 *
 */
void RoboDriver::PVTTrigonalAcc(float PosWant,float T,u8 index,PVT_Prama *mPVT)
{
    float a  = 32*PosWant/3./T/T;
    float v0 = PosWant/3./T;
    float cnt_a=a*Leg[index]->EncoderCnt*4*Leg[index]->Leg_Reduction/2./pi;
    float cnt_v0=v0*Leg[index]->EncoderCnt*4*Leg[index]->Leg_Reduction/2./pi;
    for(int i=0;i<PVTPnt;i++)
    {
        mPVT[i].tim = T/PVTPnt;
        if(i<(PVTPnt/4.))
        {
            mPVT[i].vel = cnt_v0;
            mPVT[i].pos = mPVT[i].vel * mPVT[i].tim * (i+1);
        }
        else if(i<(PVTPnt/4.*2))
        {
            mPVT[i].vel = cnt_v0 + cnt_a *(i-(PVTPnt/4.)+1)*mPVT[i].tim;
            mPVT[i].pos = PosWant/12.*Leg[index]->EncoderCnt*4*Leg[index]->Leg_Reduction/2./pi + cnt_v0*(i-1)*mPVT[i].tim + 0.5* cnt_a*((i-(PVTPnt/4.)+1)*mPVT[i].tim)*((i-(PVTPnt/4.)+1)*mPVT[i].tim);
        }
        else if(i<(PVTPnt/4.*3))
        {
            mPVT[i].vel = cnt_v0 +cnt_a * T/4. - cnt_a*(i-(PVTPnt/4.*2)+1)*mPVT[i].tim;
            mPVT[i].pos = PosWant/2.*Leg[index]->EncoderCnt*4*Leg[index]->Leg_Reduction/2./pi + (cnt_v0 +cnt_a * T/4.) * (i-(PVTPnt/4.*2)+1)*mPVT[i].tim - 0.5f * cnt_a * (i-(PVTPnt/4.*2)+1)*mPVT[i].tim * (i-(PVTPnt/4.*2)+1)*mPVT[i].tim;
        }
        else
        {
            mPVT[i].vel = cnt_v0;
            mPVT[i].pos = 11./12. * PosWant * Leg[index]->EncoderCnt*4*Leg[index]->Leg_Reduction/2./pi + cnt_v0 * mPVT[i].tim * (i-(PVTPnt/4.f*3)+1);
        }
    }
}
