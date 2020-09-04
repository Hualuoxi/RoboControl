#ifndef UTILITIES_H
#define UTILITIES_H
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int32_t s32;
typedef short int s16;

typedef signed          char int8_t;
typedef signed short    int int16_t;
typedef volatile signed int vint32_t;
typedef volatile signed char vint8_t;

/********************   PI   *************************/
extern const double pi;
/*****************  Elmo_CAN_ID  *********************/
#define ELMO_CTRL_ID_BASE   0x300
#define ELMO_FBCK_ID_BASE   0x280
#define ELMO_COARSEN_1      1
#define ELMO_COARSEN_2      2
#define ELMO_WALKER_1       3
#define ELMO_WALKER_2       4
#define ELMO_NUM            4
/*****************  Elmo_MODE  *********************/

// convert 4 bytes to signed int
#define Int2Char(charBuff, intValue)		{ *(signed int*)buff = *intValue; }
// signed int to 4 bytes
#define Char2Int(intValue, charBuff)		{ *(signed int *)intValue = *(signed int *)charBuff; }
typedef struct
{
    float pitch;			/*uinit: ° (deg)*/
    float roll;
    float yaw;
}RobotAngle;

typedef struct
{
    float x;			/*uinit:  m */
    float y;
    float z;
}RobotPosition;

typedef struct
{
    float x;			/*uinit: m/s */
    float y;
    float w;
}RobotSpeed;

enum _ELMO_MODE
{
    TORQUE_MODE = 1,
    SPEED_MODE,
    MICRO_STEP_MODE,
    DOUBLE_FEEDBACK_POSTION_MODE,
    POSITON_MODE,
};

struct _CMD_FRAME
{
   unsigned char header[2];
   unsigned char cmd[2]; // command
   char cmd_para[4];     // parameters of the command, [0]: coasen direction; [1]: dir_1, 1 or -1; [2]: dir_2, 1 or -1;
   signed int coasen_sp; // coasening speed
   signed int move_sp;   // moving speed
   signed short turn_pa; // pa for turning angle
   signed short check;
};
typedef _CMD_FRAME CMD_frame;
void Char2Float(float* f, unsigned char* buff);
void Float2Char(unsigned char *buff, float *f);
#endif // UTILITIES_H
