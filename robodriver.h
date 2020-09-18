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
private:

    Motor* Leg[6]={nullptr};
    Motor* BroadCast;
    ControlParam *Param = nullptr;
    PVTPP* PVTPrama = nullptr;
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

    void PVTTrigonalAcc(float PosWant,float T,u8 index,PVT_Prama *mPVT);
public slots:
    void run();
    void Ready(float T, float PositionWant);
    void GoForward(float T, int num);
    void PVTGoForward(float T, int num);
    void PVTRun(float Tb,float Te, int Anum,int num);
    void PVTReady(float PosWant,float T);
    void PVTRotate(float T, int num);
    void PVTRollingOver();
    void Rotate(float T, int num);
    void RollingOver();
    void BipedRun();

    void SearchZero();

};

#endif // ROOBODRIVER_H
