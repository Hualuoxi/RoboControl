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
