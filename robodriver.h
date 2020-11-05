#ifndef ROOBODRIVER_H
#define ROOBODRIVER_H
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDateTime>
#include <QQueue>
#include <Utilities.h>
#include <motor.h>
#include <linux/can.h>
#include <math.h>
class RoboDriver : public QObject
{
    Q_OBJECT
public:
    RoboDriver(Motor * leg[],Motor* broadCast);
    void GoBack(float T, int num);
    void Jump();
    void TurnRight(float T, int num);
    void setStopRun(bool b){
        QMutexLocker stopRun_Mutexlocker(&stopRun_Mutex);
        stopRun = b;
    }
    bool getStopRun(){
        QMutexLocker stopRun_Mutexlocker(&stopRun_Mutex);
        return stopRun;
    }
    void setSuspension_count(float count){
        suspension_count = count;
    }
    float getSuspension_count()
    {
        return suspension_count;
    }
public:
    QMutex stopRun_Mutex;
private:
    bool stopRun = false;
    Motor* Leg[6]={nullptr};
    Motor* BroadCast;
    ControlParam *Param = nullptr;
    PVTPP* PVTPrama = nullptr;
    float suspension_count = 0.6;
signals:
    void sendCANMsg_sig(can_frame Tx_Msg);
    void TranClinet_sig(RobotFadebck_frame* frame);
private:
    void PVTTest();
    void PVTRunTest();
    void getRefPos(float Pos[]);
    void IndependentControl(void);
    void SetControlParam(ControlParam* P);
    void setPVTPrama(PVTPP *prama);
    void TrapezoidalAcc(float am, float t, float T, XVAS* xvas);
    void TrigonalAcc(float PositionWant, float t, float T, XVAS* xvas);
    void PVTControl(void);
    void DynamicsRun(float Tb,float Te,float Tsb,float Tse,int AccStep);
    void PVTTrigonalAcc(float PosWant,float T,u8 index,PVT_Prama *mPVT);
    void DynamicsRunAcc(float PosWant,float Ts,float T,u8 index,PVT_Prama *mPVT);
    void Rotate(float T, int num);
    void RollingOver();
    void BipedRun();
    void Ready(float T, float PositionWant);
    void GoForward(float T, int num);
public slots:
    void run();
    void PVTGoForward(float T, int num);
    void PVTWalkForward(float T, int num);
    void PVTWalkBack(float T, int num);
    void PVTTurnLeft(float T, int num);
    void PVTTurnRight(float T, int num);
    void PVTRun(float Tb,float Te, int Anum,int num);
    void PVTReady(float PosWant,float T);
    void PVTRotate(float T, int num);
    void PVTRollingOver();
    void SearchZero();

};

#endif // ROOBODRIVER_H
