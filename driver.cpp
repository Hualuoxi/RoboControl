#include "driver.h"

Driver::Driver(u8 id)
{   
    mCANId = ELMO_CTRL_ID_BASE+(int)id;
}

void Driver::InitElmo()
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = 0;
    frame.can_dlc = 8;
    frame.data[0] = 1;
    frame.data[1] = mCANId-ELMO_CTRL_ID_BASE;
    for(int i = 1; i < 8; i++){
        frame.data[i] = 0;
    }
    emit sendCANMsg_sig(frame);
}

/**
 * @brief SetPVTPDOMapping
 *      将PVT参数（0x2001）映射到RPDO2，用于快速传递PVT参数。
 */

void Driver::SetPVTPDOMapping()
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    //0x1601(0)设置为0  Stop all emissions of RPDO2
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x16;
    frame.data[3] = 0x0;
    frame.data[4] = 0x0;
    frame.data[5] = 0x0;
    frame.data[6] = 0x0;
    frame.data[7] = 0x0;
    emit sendCANMsg_sig(frame);

    //0x1601(1)设置为0x20010040
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x16;
    frame.data[3] = 0x1;
    frame.data[4] = 0x40;
    frame.data[5] = 0x0;
    frame.data[6] = 0x01;
    frame.data[7] = 0x20;
    emit sendCANMsg_sig(frame);

    //0x1401(2)设置为255
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x14;
    frame.data[3] = 0x02;
    frame.data[4] = 0xFF;
    emit sendCANMsg_sig(frame);

    //0x1601(0)设置为1
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x16;
    frame.data[3] = 0x0;
    frame.data[4] = 0x1;
    frame.data[5] = 0x0;
    frame.data[6] = 0x0;
    frame.data[7] = 0x0;
    emit sendCANMsg_sig(frame);
}

/**
 * @brief ElmoDrive::SetFeedbackPDOMapping
 * 将实际位置（0x6064)和实际速度（0x606c）映射到TPDO3（
 */
void Driver::SetFeedbackPDOMapping()
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    //0x1A02(0)设置为0
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x1A;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    emit sendCANMsg_sig(frame);

    //0x1A02(1)设置为0x60640020，即实际位置
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x1A;
    frame.data[3] = 0x01;
    frame.data[4] = 0x20;
    frame.data[5] = 0x00;
    frame.data[6] = 0x64;
    frame.data[7] = 0x60;
    emit sendCANMsg_sig(frame);

    //0x1A02(2)设置为0x2F110010，即PVT buffer head pointer
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x1A;
    frame.data[3] = 0x02;
    frame.data[4] = 0x10;
    frame.data[5] = 0x00;
    frame.data[6] = 0x11;
    frame.data[7] = 0x2F;
    emit sendCANMsg_sig(frame);

    //0x1A02(3)设置为0x2F120010，即PVT buffer tail pointer
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x1A;
    frame.data[3] = 0x03;
    frame.data[4] = 0x10;
    frame.data[5] = 0x00;
    frame.data[6] = 0x12;
    frame.data[7] = 0x2F;
    emit sendCANMsg_sig(frame);

    //0x1802(2)设置为x，即xms反馈一次位置和速度，
    //也可以设置成发送一个SYNC反馈一次
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x18;
    frame.data[3] = 0x02;
    frame.data[4] = 1;//100ms发送一次
    emit sendCANMsg_sig(frame);

    //0x1A02(0)设置为3
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x600;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x02;
    frame.data[2] = 0x1A;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    emit sendCANMsg_sig(frame);


}
void Driver::InitPVT()
{
    ExeCMD("MP",1,1); //第一个下标
    QThread::msleep(10);
    ExeCMD("MP",PVTPnt,2); //最后一个下标
    QThread::msleep(10);
    ExeCMD("MP",1,3); //1： 运动是周期性的  0： QP[N]阵列最后一个（ MP[2]）元素执行后运动停止
    QThread::msleep(10);
    ExeCMD("MP",2,5);
    QThread::msleep(10);
    ExeCMD("MP",1,6);
}
void Driver::ExeCMD(int pos,int vel, int time)
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId - ELMO_CTRL_ID_BASE + 0x400;
    frame.can_dlc = 8;
    //位置
    frame.data[0] = (pos & 0x000000FF);
    frame.data[1] = ((pos >> 8) & 0x000000FF);
    frame.data[2] = ((pos >> 16) & 0x000000FF);
    frame.data[3] = ((pos >> 24) & 0x000000FF);
    //速度
    frame.data[4] = (vel & 0x000000FF);
    frame.data[5] = ((vel >> 8) & 0x000000FF);
    frame.data[6] = ((vel >> 16) & 0x000000FF);
    //时间
    frame.data[7] = (time & 0x000000FF);
    emit sendCANMsg_sig(frame);
}

void Driver::ExeCMD(char const* cmd)
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = mCANId;
    frame.can_dlc = 4;
    frame.data[0] = cmd[0];
    frame.data[1] = cmd[1];
    frame.data[2] = 0;
    frame.data[3] = 0;
    emit sendCANMsg_sig(frame);
}

void Driver::ExeCMD(char const* cmd, int value, short index)
{
    // cout << "value = " << value << endl;
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id =   mCANId;
    frame.can_dlc = 8;
    frame.data[0] = cmd[0];
    frame.data[1] = cmd[1];
    frame.data[2] = (index >> 0) & 0x00FF;
    frame.data[3] = (index >> 8) & 0x00FF;
    frame.data[4] = (value >> 0) & 0x00FF;
    frame.data[5] = (value >> 8) & 0x00FF;
    frame.data[6] = (value >> 16) & 0x00FF;
    frame.data[7] = (value >> 24) & 0x00FF;
    // ENCODE_INT2CHAR(&value, &frame.data[4]);

    emit sendCANMsg_sig(frame);
}

void Driver::ExeCMD(char const* cmd, float value, short index)
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id =  mCANId;
    frame.can_dlc = 8;
    frame.data[0] = cmd[0];
    frame.data[1] = cmd[1];
    frame.data[2] = (index >> 0) & 0x00FF;
    frame.data[3] = (((index >> 8) & 0x003F) | 0x80);
    u16 f_c[2];
    *(float*)f_c = *(&value);
    frame.data[4] = (f_c[0]>>0&0x00ff);
    frame.data[5] = ((f_c[0]>>8)&0x00ff);
    frame.data[6] = (f_c[1]>>0&0x00ff);
    frame.data[7] = ((f_c[1]>>8)&0x00ff);
    emit sendCANMsg_sig(frame);
}

void Driver::QueryCMD(char const* cmd, short index)
{
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id =  mCANId;
    frame.can_dlc = 4;
    frame.data[0] = cmd[0];
    frame.data[1] = cmd[1];
    frame.data[2] = (index >> 0) & 0x00FF;
    frame.data[3] = (((index >> 8) & 0x00FF) | 0x40);
    emit sendCANMsg_sig(frame);
}
