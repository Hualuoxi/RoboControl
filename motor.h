#ifndef MOTOR_H
#define MOTOR_H
#include "driver.h"
class Motor:public QObject
{
    Q_OBJECT
public:
    Motor(u8 id,u8 mode);
    void initMotor();
    void EnableMotor(bool bAction);
    void SetMode(u8 mode);
    void Go(float speed);
    void RunByPos(int position);
    void Stop();
    void StopByST();
    void QueryCurrent();
    void setMotorCurrent(float value){
        current = value;}
    float getMotorCurrent(){
        return current;}
    void setMotorSpeed(float value){
        speed=value;
    }
    float getMotorSpeed(){
        return speed;
    }

private:
    Driver *mDriver;
    float speed;
    float current;
    int position;
    u8 motor_mode;
    bool bAction;
    u8 motor_id;
signals:
    void sendCANMsg(can_frame Tx_Msg);
public slots:
    void sendSignal(can_frame Tx_Msg);
};

#endif // MOTOR_H
