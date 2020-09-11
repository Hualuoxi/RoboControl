#ifndef MOTOR_H
#define MOTOR_H
#include "driver.h"
#include <QDebug>
#include <QThread>
class Motor:public QObject
{
    Q_OBJECT
public:
    Motor(u8 id,u8 mode);
    void initMotor();
    void EnableMotor(bool bAction);
    void SetMode(u8 mode);
    void RunBySpd(float speed);
    void SetRunByPosSpd(float speed);
    void SetZeroPos(int position);
    void RunByPosA(int position);
    void RunByPosR(int position);
    void RunByCur(float current);
    void Release();
    void Stop();
    void QueryPos();
    void QuerySpd();
    void QueryCur();
    void QueryErr();
    void setMotorPos(int value){
        motor_position = value;}
    int getMotorPos(){
        return motor_position;}
    void setMotorCur(float value){
        motor_current = value;}
    float getMotorCur(){
        return motor_current;}
    void setMotorSpd(float value){
        motor_speed=value;
    }
    float getMotorSpd(){
        return motor_speed;
    }
public:
    u16 EncoderCnt =  2048;
    u16 Kv_Encoder = 2048 * 4 /60;
private:
    Driver *mDriver;
    float motor_speed;
    float motor_current;
    int motor_position;
    u8 motor_mode;
    bool bAction;
    u8 motor_id;

signals:
    void sendCANMsg_sig(can_frame Tx_Msg);
public slots:
    void sendSignal_slot(can_frame Tx_Msg);
};

#endif // MOTOR_H
