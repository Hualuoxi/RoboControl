#ifndef ROBOT_H
#define ROBOT_H
#include <QTimer>
#include "can.h"
#include "motor.h"
#include "gyro.h"
#include "clinet.h"
class Robot:public QObject
{
    Q_OBJECT
public:
    Robot(QObject * parent = nullptr);
    ~Robot();
    void InitRobot();
    void ConnetSlots();
    void SubThread();
    void InitPIDParam();
private:
    QThread *mClinetQThread;
    QThread *mGyroQThread;
    RobotAngle AngleNow;
    RobotAngle AngleWant;
    QTimer *mTimer;
    QTimer *PIDQTimer;
    QThread *PIDQThread;
    RobotPID PID_Yaw;
    RobotPID PID_Pitch;
public:
    CAN *mCAN;

    Gyro *mGyro;
    Clinet *mClient;

    USART *mUSART;
    Motor *Wheel_1;
    Motor *Wheel_2;
    Motor *Cutter_1;
    Motor *Cutter_2;
    Motor *Turn_1;
    Motor *Turn_2;
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
    void setMotorCurrent_slot(int id,float value);
    void PID();
};

#endif // ROBOT_H
