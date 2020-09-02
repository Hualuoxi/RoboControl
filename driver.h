#ifndef DRIVER_H
#define DRIVER_H
#include <QObject>
#include <linux/can.h>
#include "Utilities.h"


class Driver:public QObject
{
    Q_OBJECT
public:
    Driver(u8 id);
    void InitElmo();


    void ExeCMD(char const* cmd);
    void ExeCMD(char const* cmd, int value, short index = 0);
    void ExeCMD(char const* cmd, float value, short index = 0);
    void QueryCMD(char const* cmd, short index = 0);
private:
    u8 mCANId;
signals:
    void sendCANMsg(can_frame Tx_Msg);
};

#endif // DRIVER_H
