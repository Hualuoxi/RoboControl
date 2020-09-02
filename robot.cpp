#include "robot.h"

Robot::Robot()
{
    mCAN = new CAN("can1");
    mCAN->startRcv(); //
    Cutter_1 = new Motor(1,SPEED_MODE);
    Cutter_2 = new Motor(2,SPEED_MODE);
    Wheel_1 = new Motor(3,SPEED_MODE);
    Wheel_2 = new Motor(4,SPEED_MODE);
    Turn_1 = new Motor(5,POSITON_MODE);
    Turn_2 = new Motor(6,POSITON_MODE);
    //将各电机的发送信号与CAN发送槽连接
    connect(Cutter_1,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));
    connect(Cutter_2,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));
    connect(Wheel_1,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));
    connect(Wheel_2,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));
    connect(Turn_1,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));
    connect(Turn_2,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));

    connect(this,SIGNAL(sendCANMsg(can_frame)),mCAN,SLOT(Transmit(can_frame)));
    //初始化电机，由于初始化的过程中需要发送报文，因此需要在电机与CAN建立连接之后才能调用
    Cutter_1->initMotor();
    Cutter_2->initMotor();
    Wheel_1->initMotor();
    Wheel_2->initMotor();
    Turn_1->initMotor();
    Turn_2->initMotor();
}
