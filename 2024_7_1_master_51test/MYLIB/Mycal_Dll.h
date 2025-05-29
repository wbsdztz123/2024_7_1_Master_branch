#ifndef MYLIB_H
#define MYLIB_H
#include "main.h"

#define PEAK_NUM_POINTS_MAX 128
#define PI                          3.14159265358979f
#define ang_to_rad  PI/180.0f
#define FILTERED_POINTS 1  debug


typedef signed char int8_t;
typedef unsigned char   uint8_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;
typedef int  int32_t;
typedef unsigned   uint32_t;
typedef float float32_t;

typedef enum
{
    INSTALL_LEFT_BACK = 0,
    INSTALL_RIGHT_BACK,
    INSTALL_LEFT_FRONT,
    INSTALL_RIGHT_FRONT,
    INSTALL_FRONT,
    INSTALL_BACK,
}InstallPositionT;

typedef struct
{
    float range;
    float doppler;
    float azimuth;
    float elevation;
    float snr;
    float power;
    float angle;
}Cal_data;

typedef struct
{
    float32_t Velocity; /* Vehicle Speed, Km/h*/
    float32_t YawRate;       /* Vehicle Yaw Rate, - = clockwise unit deg/s */
    float32_t SteeringAngle;/**/
    float32_t CurveRadius;
    float32_t RoadCurve;
}Message_VehicleMsgS;

typedef enum {
    CALIBRATION_INIT = 0x00,
    CALIBRATION_RUNING,
    DATA_READ_EXIT,
    CALIBRATION_EXIT
} CALIBRATION_MODE;

typedef struct
{  
    char *output_filename;  //筛选出的点云文件名
    char *Input_filename;   //输入原始点云文件名
    uint8_t  Frame_number;      //帧列
    uint8_t  Serial_number;     //序列号列
    uint8_t  Range;             //Range列
    uint8_t  Doppler;           //Doppler列
    uint8_t  Azimuth;           //Azimuth列
    uint8_t  Snr;               //Snr列
    uint8_t  Vel;               //车速列
    uint8_t  Yaw;               //横摆角列
    uint8_t  Steer;             //转向角列
    uint8_t  Cur;               //曲率半径列
    uint8_t  LIST_NUM;          //单帧获取列数
    uint8_t  line_NUM;          //单帧获取行数
    sem_t sem;              //信号量1  控制线程同步
    sem_t sem2;             //信号量2 控制线程同步
    pthread_t Data_reading_thread; //数据读取线程
    pthread_t Calibration_thread; //校准线程
    char *Split_symbol;     //分隔符
    uint32_t point_num;
    Cal_data Cal_data[PEAK_NUM_POINTS_MAX]; //校准数据
    Message_VehicleMsgS Message_VehicleMsg;
}SIMULATION_DATA;



__declspec(dllexport) void Output_file_clearing(char *output_filename);
__declspec(dllexport) void Tag_write(char *output_filename);
__declspec(dllexport) void Calibration_Screening_Angle(char *output_filename);
__declspec(dllexport) void YD_XD_writing(float YD,float XD,char *output_filename);
__declspec(dllexport) void Function_param_init();
__declspec(dllexport) void FILE_Read();
__declspec(dllexport) void Function_param_join();

#endif // MYLIB_H