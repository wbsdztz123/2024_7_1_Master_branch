
/****************************************************************************
 *                        File: adaptive_calibration.h                      *
 *                        @sjw20210713                                           *
 ****************************************************************************/

  
#ifndef __ADAPTIVE_CALIBRATION_H
#define __ADAPTIVE_CALIBRATION_H
#define GTRACK_NUM_POINTS_MAX 300
/* Includes ------------------------------------------------------------------*/
//#include "common/bsd_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#define max(a,b) ((a) > (b) ? (a) : (b))



#define CalibrationTime       10//110//120//100//80//57//180//57       //һ�α궨��֡����150����150֡��
#define Timeframe              7//8//7//7         //1֡�������õ����ݣ�5����5���������������ݡ�
#define CalibrationRangeMin   3.0f//15//20//15//3//3    ��С�궨����
#define CalibrationRangeMid   30.0f     
#define CalibrationRangeMax   45.0f//50.0f//50.0f//40.0f//50.0f//40.0f//50//40//50//45//50//25//35//12//35       //���궨����35
#define AdaptiveCalibration_OutputB       0      //����Ӧ�궨�����B
//#define AdaptiveCalibration_OutputB       1      //����Ӧ�궨���B

#define Calibration_MaxSteeringAngle    10.0f//5.0f//10//20   
#define Calibration_MaxYawRate          20.0f
#define Calibration_MinVelocity         4.0f
#define Calibration_MaxVelocity         19.5f
#define Calibration_MaxRoadCurve        300.0f
#define Calibration_MinRCs              10.0f//15.0f//5//30//30//40//45//30
#define Calibration_Ydata_gap                 0.7f//0.7f//2.0f//0.7f//1.0f//0.7f//1.0f//0.5f//1.0f


#define PI                          3.14159265358979f
#define RadarInstallAngle            38



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

typedef struct
{
    /**  @brief   Range, m */
	float range;
    /**  @brief   Azimuth, rad */
	float azimuth;
    /**  @brief   Elevation, rad */
	float elev;
    /**  @brief   Radial velocity, m/s */
	float doppler;

    
} GTRACK_measurement_vector;


typedef struct {
        union {
            /**  @brief   Measurement vector */
            GTRACK_measurement_vector vector;
        };
        /**  @brief   status, is valuable or not */
        uint8_t status;
        /**  @brief   Range detection SNR, linear */
        float snr;
        /**  @brief  angle, radian */
        float angle;
        /**  @brief  Heighth, m */
        float Heighth;
        /**  @brief  isStatic,  is Absolute Static or not */
        bool isStatic;
        /**  @brief  FenceFlag,  is fence or not */
        uint8_t FenceFlag;
        /**  @brief   Cross Range, m */
        float CrossRange;
        /**  @brief   Longtitude Range, m */
        float LongtitudeRange;
        /**  @brief   NearStaticFlag, is Near Static or not */
        bool NearStaticFlag;
        /**  @brief   VerticalVelocity, m/s */
        float VerticalVelocity;
        /**  @brief   signal, DB */
        float signal; // wxq20230419
        /**  @brief   Near Target Index,  */
        uint16_t NearIndex; // wxq20230424

        float rcs;
        
    } GTRACK_measurementPoint;




//void Adaptive_CalibrationInit(void);
void Adaptive_CalibrationClear(void);
void Adaptive_Calibration(uint32_t point_count,GTRACK_measurementPoint *PeakList);
void Adaptive_CalibrationSaveData(uint32_t point_count,GTRACK_measurementPoint *PeakList);
void Adaptive_CalibrationPolyFit(void);
void R_squareChack(void);
void Adaptive_CalibrationFinish(void);
void Adaptive_Calibration_Exit(uint8_t* StatusArray);  //�˳��궨
uint8_t Rang_judge(uint32_t point_count,GTRACK_measurementPoint *PeakList);
uint8_t AdaptiveCalStart(void);
void GetAdaptiveCalStatus(uint8_t * StatusArray);
void timer_fuc_ADPCAL_end(void);
// static void AdaptiveCalTimeoutProc(ClockP_Object *clock, void *argv);
//void AdaptiveCalClockLaunch(uint64_t period);
void Private_Can_CalibrationDebug(uint32_t ID,uint8_t* Data,uint8_t lenth);
void Calibration_Progress(uint8_t pace);
uint8_t Body_Posture_Detection(void);
uint8_t CAL_Target_Filtering(GTRACK_measurementPoint *PeakList,uint8_t i);
void SaveAdaptiveCalStatus_DID_0x4902(uint8_t * StatusArray);
#endif /* __ANGLE_CONFIG_H */

/******************************END OF FILE*************************************/
