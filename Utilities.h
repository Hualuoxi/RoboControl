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
extern const float pi;
/*****************  Elmo_CAN_ID  *********************/
#define ELMO_CTRL_ID_BASE   0x300
#define ELMO_FBCK_ID_BASE   0x280
#define ELMO_CUTTER_1      1
#define ELMO_CUTTER_2      2
#define ELMO_WHEEL_1       3
#define ELMO_WHEEL_2       4
#define ELMO_TURN_1        5
#define ELMO_TURN_2        6
#define ELMO_NUM           6

#define ELMO_CUTTER_1_TX    (ELMO_CTRL_ID_BASE+ELMO_CUTTER_1)
#define ELMO_CUTTER_2_TX    (ELMO_CTRL_ID_BASE+ELMO_CUTTER_2)
#define ELMO_WHEEL_1_TX     (ELMO_CTRL_ID_BASE+ELMO_WHEEL_1)
#define ELMO_WHEEL_2_TX     (ELMO_CTRL_ID_BASE+ELMO_WHEEL_2)
#define ELMO_TURN_1_TX      (ELMO_CTRL_ID_BASE+ELMO_TURN_1)
#define ELMO_TURN_2_TX      (ELMO_CTRL_ID_BASE+ELMO_TURN_2)

#define ELMO_CUTTER_1_RX    (ELMO_FBCK_ID_BASE+ELMO_CUTTER_1)
#define ELMO_CUTTER_2_RX    (ELMO_FBCK_ID_BASE+ELMO_CUTTER_2)
#define ELMO_WHEEL_1_RX     (ELMO_FBCK_ID_BASE+ELMO_WHEEL_1)
#define ELMO_WHEEL_2_RX     (ELMO_FBCK_ID_BASE+ELMO_WHEEL_2)
#define ELMO_TURN_1_RX      (ELMO_FBCK_ID_BASE+ELMO_TURN_1)
#define ELMO_TURN_2_RX      (ELMO_FBCK_ID_BASE+ELMO_TURN_2)



#define PVTPnt 8

// convert 4 bytes to signed int
#define Int2Char(charBuff, intValue)		{ *(char *)charBuff = *(char *)intValue; }
// signed int to 4 bytes
#define Char2Int(intValue, charBuff)		{ *(signed int *)intValue = *(signed int *)charBuff; }

// convert 4 bytes to signed int
#define s162Char(charBuff, s16Value)		{ *(char *)charBuff = *(char *)s16Value; }
// signed int to 4 bytes
#define Char2s16(intValue, charBuff)		{ *(s16 *)intValue = *(s16 *)charBuff; }

typedef struct
{
    float pitch;			/*uinit: ° (deg)*/
    float roll;
    float yaw;
}RobotAngle;

/*
 * @brief 位移速度加速度冲击结构体
 */
struct XVAS
{
    float x;
    float v;
    float a;
    float s;
};

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Ek[3];
    float ErrSum;
    float OutPut;


    float ThresHold;
    float MaxOut;
}RobotPID;

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


enum MOVESTATUS
{
    STOP = 0,
    RUN,
    TURNRIGHT,
    TURNLEFT,
    ROTATION,
    BIPEDRUN,
    READY,
    WALK,
};


#pragma pack(1)
struct _ROBOT_DATA_FRAME
{
   u8 header[2];
   u8 status[2];
   int16_t data1;
   int16_t data2;
   int16_t data3;
   int16_t data4;
   int16_t data5;
   int16_t data6;
   int16_t data7;
   int16_t data8;
   int16_t data9;
   int16_t data10;
   int16_t check;
};
typedef _ROBOT_DATA_FRAME RobotFadebck_frame;

struct IndependentControlParam
{
    // 目标位置 转折点之后反转
    float PW[6] = { 0 };
    // 相位差
    float PD[6] = { 0 };
    // 运动周期
    float T[6] = { 1000,1000,1000,1000,1000,1000 };
    // 转向点
    float TP[6] = {0};
    // 转向时间
    float TT[6] = { 0 };
    // 步态循环次数
    int StepNum = 1;
    // 步态循环次数
    char MoveStatus = 0;
};
typedef IndependentControlParam ControlParam;


struct PVTPlanningParam
{
    // 目标位置 转折点之后反转
    float PosWant[6] = {2*pi,2*pi,2*pi,2*pi,2*pi,2*pi};
    // 相位差
    float PhaseDiff[6] = {0.0,0.0,0.0,0.0,0.0,0.0 };
    // 运动周期
    float T[6] = { 2000,2000,2000,2000,2000,2000 };
    float Tb[6] = { 2000,2000,2000,2000,2000,2000 };
    float Te[6] = { 2000,2000,2000,2000,2000,2000 };

    float Ts[6] = { 0.6,0.6,0.6,0.6,0.6,0.6 };
    float Tsb[6] = { 0.6,0.6,0.6,0.6,0.6,0.6 };
    float Tse[6] = { 0.4,0.4,0.4,0.4,0.4,0.4 };
    // 步态循环次数
    int StepNum[6] = {1,1,1,1,1,1};

    bool Bg[6]={true,true,true,true,true,true};
    bool StartRun[6]={true,true,true,true,true,true};
    bool ContinueRun[6]={false,false,false,false,false,false};

    int StepAcc[6]={0,0,0,0,0,0};
    int StepDcc[6]={0,0,0,0,0,0};
    bool KeepPD[6]={false,true,true,false,false,true};

    char MoveStatus = 0;

};
typedef PVTPlanningParam PVTPP;

struct _CMD_FRAME
{
   u8 Header[2];
   u8 Cmd[2]; // command
   s16 Data[3];     // parameters of the command, [0]: coasen direction; [1]: dir_1, 1 or -1; [2]: dir_2, 1 or -1;
   s16 Check;
};
typedef _CMD_FRAME CMD_frame;
#pragma pack()


struct PVT_Prama
{
   double pos;
   double vel;
   double tim;
};


void Char2Float(float* f, unsigned char* buff);
void Float2Char(unsigned char *buff, float *f);
#endif // UTILITIES_H
