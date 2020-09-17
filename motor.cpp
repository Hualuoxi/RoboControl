#include "motor.h"

Motor::Motor(u8 id ,u8 mode)
{
    motor_id = id;
    motor_mode = mode;
    mDriver=new Driver(motor_id);
    connect(mDriver,&Driver::sendCANMsg_sig,this,&Motor::sendSignal_slot);
    //connect(mDriver,SIGNAL(sendCANMsg(can_frame)),this,[=](can_frame Tx_meg){emit sendCANMsg(Tx_meg);});
}

void Motor::initMotor()
{
    mDriver->InitElmo();
    SetMode(motor_mode);
    QThread::msleep(10);
    mDriver->ExeCMD("PX", 0);
    QThread::msleep(10);
    mDriver->ExeCMD("SP", (int)(2000*Kv_Encoder));
    QThread::msleep(10);
    mDriver->ExeCMD("SF", 0);
    mDriver->SetPVTPDOMapping();
    QThread::msleep(10);
    mDriver->SetFeedbackPDOMapping();
    QThread::msleep(10);
    mDriver->InitPVT();
    mDriver->ExeCMD("CL", 4.0f, 1); // CL[1] = 4;
    mDriver->ExeCMD("PL", 10.0f, 1); // PL[1] = 10;
}

void Motor::EnableMotor(bool action)
{
    bAction=action;
    //qDebug()<<"bAction:"<<bAction;
    mDriver->ExeCMD("MO", bAction ? 1 : 0);
}


void Motor::SetMode(u8 mode)
{
    motor_mode = mode;
    mDriver->ExeCMD("UM", mode);
}

void Motor::RunBySpd(float speed)
{
    motor_speed=speed;
    mDriver->ExeCMD("JV", (int)(speed*Kv_Encoder));
    mDriver->ExeCMD("BG");
}

void Motor::SendPVT(int position,int speed,int time)
{
    mDriver->ExeCMD(position,speed,time);
//    if(motor_id==2)
//        qDebug()<<QString("motor_2 = %1 ,vel = %2 ,tim = %3").arg(position).arg(speed).arg(time)<<endl;
//    if(motor_id==1)
//        qDebug()<<QString("motor_1 = %1 ,vel = %2 ,tim = %3").arg(position).arg(speed).arg(time)<<endl;
}

void Motor::RunBypPVT(int ReadPoint)
{
    mDriver->ExeCMD("PV", ReadPoint);
}

void Motor::SetZeroPos(int position)
{
    motor_position = position;
    mDriver->ExeCMD("PX", position);
}
void Motor::SetRunByPosSpd(float speed)
{
    motor_speed=speed;
    mDriver->ExeCMD("SP", (int)(speed*Kv_Encoder));
    mDriver->ExeCMD("BG");
}

void Motor::RunByPosA(int position)
{
    mDriver->ExeCMD("PA", position);
//    mDriver->ExeCMD("BG");
}

void Motor::RunByPosR(int position)
{
    mDriver->ExeCMD("PR", position);
    mDriver->ExeCMD("BG");
}

void Motor::RunByCur(float current)
{

   mDriver->ExeCMD("TC", current);
}


void Motor::Release()
{
    EnableMotor(false);
}

void Motor::Stop()
{
    mDriver->ExeCMD("ST");
}

void Motor::Begin()
{
    mDriver->ExeCMD("BG");
}

void Motor::QueryCur()
{
    mDriver->QueryCMD("IQ");
}

void Motor::QueryErr()
{
    mDriver->QueryCMD("EC");
}

void Motor::QueryPos()
{
    mDriver->QueryCMD("PX");
}

void Motor::QuerySpd()
{
    mDriver->QueryCMD("VX");
}

int Motor::Rad2Cnt(float red)
{
    return (int)(red * Leg_Reduction * EncoderCnt *4 /2 /pi);
}
float Motor::Cnt2Rad(int cnt)
{
    return cnt * 2.f * pi / Leg_Reduction / EncoderCnt / 4.f;
}
void Motor::sendSignal_slot(can_frame Tx_Msg)
{
    emit sendCANMsg_sig(Tx_Msg);
};
