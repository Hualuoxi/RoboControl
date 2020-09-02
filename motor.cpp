#include "motor.h"

Motor::Motor(u8 id ,u8 mode)
{
    motor_id = id;
    motor_mode = mode;
    mDriver=new Driver(motor_id);
    connect(mDriver,SIGNAL(sendCANMsg(can_frame)),this,SLOT(sendSignal(can_frame)));

    //connect(mDriver,SIGNAL(sendCANMsg(can_frame)),this,[=](can_frame Tx_meg){emit sendCANMsg(Tx_meg);});
}
void Motor::initMotor()
{
    mDriver->InitElmo();
    EnableMotor(false);
    SetMode(motor_mode); // coarsening motors use speed mode(2) ; direction control motors use Dual Feedback Position Control(4)
    mDriver->ExeCMD("PX", 0);
    mDriver->ExeCMD("CL", 4.0f, 1); // CL[1] = 10;
    mDriver->ExeCMD("PL", 10.0f, 1); // PL[1] = 10;
    EnableMotor(true);
}
void Motor::EnableMotor(bool bAction)
{
    mDriver->ExeCMD("MO", bAction ? 1 : 0);
}

void Motor::SetMode(u_char mode)
{
    mDriver->ExeCMD("UM", mode);
}

void Motor::Go(float speed)
{
    EnableMotor(true);
    mDriver->ExeCMD("JV", speed);
    mDriver->ExeCMD("BG");
}

void Motor::Stop()
{
    EnableMotor(false);
}

void Motor::StopByST()
{
    mDriver->ExeCMD("ST");
}

void Motor::QueryCurrent()
{
    mDriver->QueryCMD("IQ");
}

void Motor::sendSignal(can_frame Tx_Msg)
{
    emit sendCANMsg(Tx_Msg);
};
