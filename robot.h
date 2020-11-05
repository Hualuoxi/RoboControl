#ifndef ROBOT_H
#define ROBOT_H
#include <QTimer>
#include "can.h"
#include "motor.h"
#include "gyro.h"
#include "clinet.h"
#include "robodriver.h"
#include <QDateTime>
#include <QFile>
#include <QDir>
#include <QTextStream>
class Robot:public QObject
{
    Q_OBJECT



public:
    Robot(QObject * parent = nullptr);
    ~Robot();
    void InitRobot();
    void ConnetSlots();
    void run();
    void InitPIDParam();

    void StartRun();
private:
    bool startRecord = false;

private:
    QFile *CurData;
//    QTextStream CurOut;
    QThread *mClinetQThread;
    QThread *mGyroQThread;
    QThread *mRoboDriverQThread;

    RobotAngle AngleNow;
    RobotAngle AngleWant;
    QTimer *mTimer;
    QTimer *RunQTimer;
    QThread *RunQThread;
    RobotPID PID_Yaw;
    RobotPID PID_Pitch;
    ControlParam *Param = nullptr;

public:
    CAN *mCAN;
    Gyro *mGyro;
    Clinet *mClient;
    USART *mUSART;
    bool isRun = true;
    Motor *Leg[6];
    Motor *BroadCast;
    RoboDriver *mRoboDriver;
signals:
    void sendCANMsg_sig(can_frame Tx_Msg);
    void TranClinet_sig(RobotFadebck_frame* frame);
    void run_sig();
    void PVTGoForward_sig(float T, int num);
    void PVTRun_sig(float Tb,float Te, int Anum,int num);
    void PVTReady_sig(float PosWant,float T);
    void PVTRotate_sig(float T, int num);
    void SearchZero_sig();
    void PVTWalkForward_sig(float T, int num);
    void PVTWalkBack_sig(float T, int num);
    void PVTTurnLeft_sig(float T, int num);
    void PVTTurnRight_sig(float T, int num);
public slots:
    void SaveCur_slot();
    void SavePv_slot();
    void setMotorCur_slot(u8 id,float value);
    void setMotorPos_slot(u8 id,int value);
    void setMotorSpd_slot(u8 id,float value);
    void setMotorPV_slot(u8 id,int pos , int vel);
    void setPVT_slot(u8 id ,int pos,s16 Hptr,s16 Tptr);
    void sendPVTPrama_slot(u8 id,s16 Wptr,s16 Rptr);
    void RcvDHPClinet_slot(QByteArray a);

};

#endif // ROBOT_H
