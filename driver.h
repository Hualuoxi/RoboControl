#ifndef DRIVER_H
#define DRIVER_H
#include <QObject>
#include <linux/can.h>
#include "Utilities.h"
#include <QThread>

class Driver:public QObject
{
    Q_OBJECT
public:
    Driver(u8 id);
    void InitElmo();
    void InitPVT();
    void SetPVTPDOMapping();
    void SetFeedbackPDOMapping() ;
    void ExeCMD(char const* cmd);
    void ExeCMD(char const* cmd, int value, short index = 0);
    void ExeCMD(char const* cmd, float value, short index = 0);
    void ExeCMD(int pos,int vel, int time);
    void QueryCMD(char const* cmd, short index = 0);
private:
    int mCANId;
signals:
    void sendCANMsg_sig(can_frame Tx_Msg);
};

#endif // DRIVER_H
