#ifndef MOTOR_H
#define MOTOR_H
#include "driver.h"
#include <QDebug>
#include <QThread>
#include <QQueue>
#include <Utilities.h>
#include <QMutex>
#include <QMutexLocker>
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
    void SendPVT(int position,int speed,int time);
    void RunBypPVT(int ReadPoint);
    void ResetPVT();

    int Rad2Cnt(float red);
    float Cnt2Rad(int cnt);

    void Release();
    void Stop();
    void Begin();
    void ClearProgram();
    void SendSYNC();
    void QueryPos();
    void QuerySpd();
    void QueryCur();
    void QueryErr();
    void setMotorPos(int value){
        motor_position = value;}
    int getMotorPos(){
        return motor_position;}

    void setMotorPosWant(int value){
        motor_positionWant = value;}
    int getMotorPosWant(){
        return motor_positionWant;}

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

    void setMotorSpdWant(float value){
        motor_speedWant=value;
    }
    float getMotorSpdWant(){
        return motor_speedWant;
    }

    void setWPtr(s16 ptr){
        QMutexLocker Wptr_Mutexlocker(&Wptr_Mutex);
        Wptr = ptr;
    }
    s16 getWPtr(){
        QMutexLocker Wptr_Mutexlocker(&Wptr_Mutex);
        return Wptr;
    }
    void setRPtr(s16 ptr){
        QMutexLocker Wptr_Mutexlocker(&Rptr_Mutex);
        Rptr = ptr ;
    }
    s16 getRPtr(){
        QMutexLocker Wptr_Mutexlocker(&Rptr_Mutex);
        return Rptr;
    }

    void setPVTflag(bool b){
        QMutexLocker PVTflag_Mutexlocker(&PVTflag_Mutex);
        PVTflag = b;
    }
    bool getPVTflag(){
        QMutexLocker PVTflag_Mutexlocker(&PVTflag_Mutex);
        return PVTflag;
    }

public:
    u16 EncoderCnt =  2048;
    float Kv_Encoder = 2048 * 4.f/60;
    float Leg_Reduction = 21.357f;
    QQueue<PVT_Prama> PVTQueue;
    QMutex PVTPrama_Mutex;
    QMutex Wptr_Mutex;
    QMutex Rptr_Mutex;
    QMutex PVTflag_Mutex;
private:

    s16 Wptr = 1;
    s16 Rptr = 1;
    bool PVTflag = true;
    Driver *mDriver;
    float motor_speed;
    float motor_speedWant;
    float motor_current;
    int motor_position;
    int motor_positionWant;
    u8 motor_mode;
    bool bAction;
    u8 motor_id;

signals:
    void sendCANMsg_sig(can_frame Tx_Msg);
public slots:
    void sendSignal_slot(can_frame Tx_Msg);
};

#endif // MOTOR_H
