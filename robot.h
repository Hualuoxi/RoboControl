#ifndef ROBOT_H
#define ROBOT_H
#include <QTimer>
#include "can.h"
#include "motor.h"
#include "gyro.h"
#include "clinet.h"
#include <QDateTime>

class Robot:public QObject
{
    Q_OBJECT

struct IndependentControlParam
{
    // 目标位置 转折点之后反转
    double PW[6] = { 0 };
    // 相位差
    double PD[6] = { 0 };
    // 运动周期
    double T[6] = { 1000,1000,1000,1000,1000,1000 };
    // 转向点
    double TP[6] = {0};
    // 转向时间
    double TT[6] = { 0 };
    // 步态循环次数
    int StepNum = 1;
    // 步态循环次数
    char MoveStatus = 0;
};
typedef IndependentControlParam ControlParam;

public:
    Robot(QObject * parent = nullptr);
    ~Robot();
    void InitRobot();
    void ConnetSlots();
    void SubThread();
    void InitPIDParam();
    void Ready(double T, double PositionWant);
    void GoForward(double T, int num);
    void GoBack(double T, int num);
    void Rotate(double T, int num);
    void TurnRight(double T, int num);
    void RollingOver();
    void BipedRun();
    void Jump();
    void StartRun();
private:
    void IndependentControl(void);
    void SetControlParam(ControlParam* P);
    void TrapezoidalAcc(double am, double t, double T, XVAS* xvas);
    void TrigonalAcc(double PositionWant, double t, double T, XVAS* xvas);
private:
    QThread *mClinetQThread;
    QThread *mGyroQThread;
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
signals:
    void sendCANMsg_sig(can_frame Tx_Msg);
    void TranClinet_sig(RobotFadebck_frame* frame);
public slots:
    void setAngle_slot();
    void StartRun_slot(int speed_, char dir_1, char dir_2);
    void StopRun_slot();
    void StartCoarsen_slot(int speed, char direction);
    void StopCoarsen_slot();
    void turn_slot(int turnDegree);
    void setMotorCur_slot(u8 id,float value);
    void setMotorPos_slot(u8 id,int value);
    void setMotorSpd_slot(u8 id,float value);
    void run();
};

#endif // ROBOT_H
