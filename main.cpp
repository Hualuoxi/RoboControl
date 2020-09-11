#include "main.h"

int main(int argc, char **argv)
{
   QCoreApplication a(argc,argv);
   qDebug()<<"Robot ready!"<<QThread::currentThread();
   Robot *MyRobot = new Robot();
   MyRobot->StartRun();
   return a.exec();
}
