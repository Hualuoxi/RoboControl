#include "main.h"

int main(int argc, char **argv)
{
   QCoreApplication a(argc,argv);
   qDebug()<<"Robot ready!"<<QThread::currentThread();
   Robot *MyRobot = new Robot();
   can_frame Tx_Meg;
   Tx_Meg.can_id=0x300;
   Tx_Meg.can_dlc=0x03;
   Tx_Meg.data[0]= 'W';
   Tx_Meg.data[1]= 'H';
   Tx_Meg.data[2]= 'U';

   //MyRobot->mCAN->Transmit(&Tx_Meg);
   return a.exec();
}
