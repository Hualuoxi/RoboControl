#ifndef ROBOT_H
#define ROBOT_H
#include "can.h"
#include "motor.h"

class Robot:public QObject
{
    Q_OBJECT
public:
    Robot();
private:

public:
    CAN *mCAN;
    Motor *Wheel_1;
    Motor *Wheel_2;
    Motor *Cutter_1;
    Motor *Cutter_2;
    Motor *Turn_1;
    Motor *Turn_2;
signals:
    void sendCANMsg(can_frame Tx_Msg);
};

#endif // ROBOT_H
