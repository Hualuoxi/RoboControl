#include "Utilities.h"
// float to char* , for sent CAN message

const float pi=3.141592654f;


void Float2Char(unsigned char *buff, float *f)
{
    unsigned short int f_c[2] = {0};
    *((float*)f_c) = *f;
    buff[0] = ((f_c[0] >> 0) & 0x00ff);
    buff[1] = ((f_c[0] >> 8) & 0x00ff);
    buff[2] = ((f_c[1] >> 0) & 0x00ff);
    buff[3] = ((f_c[1] >> 8) & 0x00ff);
}

// char* to float, for received CAN message
void Char2Float(float* f, unsigned char* buff)
{
    short int f_c[2];
    f_c[0] = *(short int*)buff;
    f_c[1] = *(short int*)(buff + 2);
    *f = *(float *)f_c;
}

u16 CRC_CHECK(u8 * Buf, u8 CRC_CNT)
{
    u16 CRC_Temp;
    u8 i, j;
    CRC_Temp = 0xffff;

    for (i = 0; i < CRC_CNT; i++)
    {
        CRC_Temp ^= Buf[i];
        for (j = 0; j < 8; j++)
        {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >> 1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
