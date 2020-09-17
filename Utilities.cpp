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
