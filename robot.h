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
    Robot();
    void InitRobot();
private:
    QThread *mClinetQThread;
    QThread *mGyroQThread;
    RobotAngle AngleNow;
    QTimer *mTimer;
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
    void sendCANMsg(can_frame Tx_Msg);
public slots:
    void showAngle();
};

#endif // ROBOT_H
