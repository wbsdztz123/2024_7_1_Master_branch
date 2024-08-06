#ifndef __ADAPTIVE_CALIBRATION_H
#define __ADAPTIVE_CALIBRATION_H
#define GTRACK_NUM_POINTS_MAX 300

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>

#define max(a,b) ((a) > (b) ? (a) : (b))
//初始化
#define CalibrationTime       250       //一次标定的帧数，150代表150帧。
#define Timeframe              10//7//20//20//7//8//7//7         //1帧数中有用的数据，5代表5个数据是有用数据。
#define CalibrationRangeMin   8.0f//15//20//15//3//3    最小标定距离
#define CalibrationRangeMid   30.0f     
#define CalibrationRangeMax   65.0f//60.0f//40.0f//50.0f//40.0f//50//40//50//45//50//25//35//12//35       //最大标定距离35
#define AdaptiveCalibration_OutputB       0      //自适应标定不输出B
//#define AdaptiveCalibration_OutputB       1      //自适应标定输出B

#define Calibration_MaxSteeringAngle    10.0f//5.0f//10//20   
#define Calibration_MaxYawRate          20.0f
#define Calibration_MinVelocity         4.0f
#define Calibration_MaxVelocity         19.5f
#define Calibration_MaxRoadCurve        300.0f
#define Calibration_MinRCs              15.0f//5//30//30//40//45//30
#define Calibration_Ydata_gap                2.0f //1.5f//2.0f//1.0f//0.7f//1.0f//0.7f//1.0f//0.5f//1.0f
#define PI                          3.14159265358979f
#define RadarInstallAngle            0.0f;
#define INSTALL_FRONT               0

typedef signed char int8_t;
typedef unsigned char   uint8_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;
typedef int  int32_t;
typedef unsigned   uint32_t;
typedef float float32_t;

typedef enum {
    CALIBRATION_INIT = 0x00,
    CALIBRATION_RUNING,
    DATA_READ_EXIT,
    CALIBRATION_EXIT
} CALIBRATION_MODE;


typedef struct
{

    float32_t Velocity; /* Vehicle Speed, Km/h*/

    float32_t YawRate;       /* Vehicle Yaw Rate, - = clockwise unit deg/s */

    float32_t SteeringAngle;/**/

    float32_t CurveRadius;
    float32_t RoadCurve;

}Message_VehicleMsgS;


typedef struct
{   float32_t InstallPosition;
    float32_t InstallAngle;
    float32_t FarHorizontalOffsetAngle;
    float32_t FarVerticalOffsetAngle;
    float32_t FarHorizontalAdptiveAngle;
    float32_t FarVerticalAdptiveAngle;

    float32_t TempHorizontalOffsetAngle;
    float32_t TempVerticalOffsetAngle;
    float32_t TempHorizontalAdptiveAngle;
    float32_t TempVerticalAdptiveAngle;

}RadarParaS;




typedef struct TagCalibrationPara               //标定
{
    uint8_t      State;                         //标定状态    0x10, OfflineCalibration ,   0x20  AdaptiveCalibration
    uint8_t      Step;                          //雷达标定的步骤
    uint8_t      Master_Result;                 //主雷达标定结果    0为未标定，1为标定成功，2为标定失败
    uint8_t      Error_Number;

    
    /***************自适应标定参数**********************/
    uint8_t      Adaptive_step;                   //自适应标定步骤
    uint8_t      Start;                         //标定开始标志
    float32_t    SteeringAngle;                 //存储标定开始时的方向盘转角
    float32_t    YawRate;                       //存储标定开始时的YawRate
    float32_t    Velocity;                      //存储标定开始时的车速
    uint8_t      Counter;                       //标定次数
    uint16_t     Frame;                         //标定帧数
    uint8_t      FalseFrame;                    //连续无目标的帧数
    uint16_t     DataNum;                       //每次标定的数据数量
    uint16_t     ChackDataNum;                  //验证需要的数据数量
    float32_t    Xdata[CalibrationTime*Timeframe];    //x是纵向距离
    float32_t    Ydata[CalibrationTime*Timeframe];    //y是横向距离
    float32_t    AveYdata;                            //平均横向距离
    float32_t    Adap_Angle;                      //自适应标定角度
    float32_t    Adap_B;                              //自适应标定角度的截距
    float32_t    Adap_A;                              //拟合直线斜率
    float32_t    Temp_A[7];
    uint8_t      TEMP_PB;               //进度标志
    uint8_t      adaptive_PB;               //实时进度
    /***************标定参数**********************/
    uint8_t      Driving_Profile;//驾驶指导
    uint8_t      errType;
    uint16_t Calibration_Counter;
    uint8_t Calibration_status;
}CalibrationParaS;

/* Includes ------------------------------------------------------------------*/
typedef struct _or_point_cloud_term_type {
    float range;
    float doppler;
    float azimuth;
    float elevation;
    float snr;
    float power;
    float angle;
} or_point_cloud_term_t;

typedef struct _or_point_cloud_format_type {
    uint32_t point_count;
    or_point_cloud_term_t term[GTRACK_NUM_POINTS_MAX];
} or_point_cloud_format_t;

typedef struct
{
    Message_VehicleMsgS Message_VehicleMsgS;
    RadarParaS RadarParaS;
    CalibrationParaS CalibrationParaS;
    or_point_cloud_format_t or_point_cloud_format_t;

}Calibration_Date;









void Adaptive_CalibrationInit(void);
void Adaptive_CalibrationClear(void);
void Adaptive_Calibration(const or_point_cloud_format_t *PeakList);
void Adaptive_CalibrationSaveData(const or_point_cloud_format_t *PeakList);
void Adaptive_CalibrationPolyFit(void);
void R_squareChack(void);
//void Adaptive_CalibrationFinish(void);
void Adaptive_Calibration_Exit(uint8_t* StatusArray);  //退出标定
uint8_t Rang_judge(const or_point_cloud_format_t *PeakList);
uint8_t AdaptiveCalStart(void);
void GetAdaptiveCalStatus(uint8_t * StatusArray);
void Calibration_Progress(uint8_t pace);
uint8_t Body_Posture_Detection(void);
uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList,uint8_t i);

#endif /* __ANGLE_CONFIG_H */