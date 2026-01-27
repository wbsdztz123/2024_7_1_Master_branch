
/****************************************************************************
 *                        File: adaptive_calibration.h                      *
 *                        @sjw20210713                                           *
 ****************************************************************************/

  
#ifndef __ADAPTIVE_CALIBRATION_H
#define __ADAPTIVE_CALIBRATION_H

/* Includes ------------------------------------------------------------------*/

#include "calibration_common.h"
///////////////////////////////////////////////////////////////////
#define ADAPTIVE_WORKMODE_CHECK_SUCCESS                      (0)
#define ADAPTIVE_WORKMODE_CHECK_NO_SUCCESS                   (-1)

#define ADAPTIVE_INIT_SUCCESS                                (0)
#define ADAPTIVE_INIT_NO_SUCCESS                             (-1)


/***************选点参数*******************/
#define CalibrationRangeMin   8.0f//15//20//15//3//3    最小标定距离
#define CalibrationRangeMid   30.0f     
#define CalibrationRangeMax   65.0f//60.0f//40.0f//50.0f//40.0f//50//40//50//45//50//25//35//12//35       //最大标定距离35

#define ADAPTIVE_MAX_AZIMUTH  10.0f
#define ADAPTIVE_MIN_AZIMUTH  -55.0f

#define Calibration_MinRCs              15.0f//5//30//30//40//45//30
#define Calibration_Ydata_gap                2.0f //1.5f//2.0f//1.0f//0.7f//1.0f//0.7f//1.0f//0.5f//1.0f
/***************选点参数*******************/


/***********车身姿态参数************************ */
#define Calibration_MaxSteeringAngle    8.0f//10.0f//5.0f//10//20   
#define Calibration_MaxYawRate          0.8f
#define Calibration_MinVelocity         4.0f
#define Calibration_MaxVelocity         19.5f
#define Calibration_MaxRoadCurve        300.0f
#define STEERING_TOLERANCE                                   3.0f // 方向盘转角允许偏差(度)

#define VELOCITY_TOLERANCE_MS                                3.0f // 车速允许偏差(m/s)
/*******************************Body attitude restriction*********************/





#define DataLength_Data_4901          1
#define DataLength_Data_4902          6

#define PROGRESS_STEP                                        14
#define INITIAL_PROGRESS                                     5
#define FINAL_PROGRESS_OFFSET                                8


#define CALIBRATION_MIN_SAMPLES                              10 // 标定最小数据点数
#define CALIB_TIMEFRAME_HALF                                 (CALIBRATION_MIN_SAMPLES / 2 + 1)
#define CALIB_FAIL_FRAME_THRESH                              (ADAPTIVE_FRAME_NUM / 18)
#define ADAPTIVE_FRAME_NUM 180    //自适应标定帧数
#define SINGLE_DATA_AMOUNT  ADAPTIVE_FRAME_NUM * CALIBRATION_MIN_SAMPLES //单次标定的数据总量
#define ADAPTIVE_COUNTER             6     //标定次数
#define ADAPTIVE_RESULT_NUM             7     

#define ADAPTIVE_CAL_TIMEOUT_CYCLE (10 * 60 * 1000 * 1000u) /* 10min */




typedef struct adaptive_calibrationpara //标定
{
    float32_t         xdata[SINGLE_DATA_AMOUNT]; //x是纵向距离
    float32_t         ydata[SINGLE_DATA_AMOUNT]; //y是横向距离
    float32_t         elevdata[SINGLE_DATA_AMOUNT];
    float32_t         rangdata[SINGLE_DATA_AMOUNT];
    float32_t         AveYdata;   //平均横向距离
    float32_t         Adap_Angle; //自适应标定角度
    float32_t         Adap_eleAngle;
    float32_t         Adap_B; //自适应标定角度的截距
    float32_t         Adap_A; //拟合直线斜率
    float32_t         Temp_A[ADAPTIVE_RESULT_NUM];
    float32_t         Temp_ele[ADAPTIVE_RESULT_NUM];
    float32_t         SteeringAngle;                      //存储标定开始时的方向盘转角
    float32_t         YawRate;                            //存储标定开始时的YawRate
    float32_t         Velocity;                           //存储标定开始时的车速
    
    // float32_t         angle_buffer_h[ADAPTIVE_COUNTER];
    // float32_t         angle_buffer_v[ADAPTIVE_COUNTER];

    // float32_t         apat_angle_h;             //水平角
    // float32_t         apat_angle_v;             //俯仰角
    calibration_adaptive_result_kind_t         adapt_result;        //结果

    uint16_t          Frame;                              //标定帧数
    uint16_t          Error_Number;  
    uint16_t          DataNum;                            //每次标定的数据数量
    uint16_t          ChackDataNum;                       //验证需要的数据数量
    uint8_t           Counter;                            //标定次数
    uint8_t           FalseFrame;                         //连续无目标的帧数
    uint8_t           TEMP_PB;           //进度标志
    uint8_t           adaptive_PB;       //实时进度
    uint8_t           driving_profile;  
    uint8_t           Step;          //雷达标定的步骤
    uint8_t           Master_Result;                      //标定结果，0为未标定，1为标定成功，2为标定失败
    uint8_t           Start;                  //标定结果，0为未标定，1为标定成功，2为标定失败

    calibration_adaptive_error_kind_t           errType;

    //ADAPTIVE_WORKMODE adaptive_Workmode; //实时进度

} adaptive_calibrationparas;
typedef enum
{
    CHEACK_MODE = 0x00,
    ADAPTIVE_MODE,
}ADAPTIVE_WORKMODE;

// static ClockP_Object adaptive_cal_clock;

void Adaptive_CalibrationInit(void);
void Adaptive_CalibrationClear(void);
void Adaptive_Calibration(const or_point_cloud_format_t *PeakList);
void Adaptive_CalibrationSaveData(const or_point_cloud_format_t *PeakList);
void Adaptive_CalibrationPolyFit(void);
void R_squareChack(void);
void Adaptive_CalibrationFinish(void);
void Adaptive_Calibration_Exit(uint8_t* StatusArray);  //退出标定
uint8_t Rang_judge(const or_point_cloud_format_t *PeakList);
uint8_t AdaptiveCalStart(void);
void GetAdaptiveCalStatus(uint8_t * StatusArray);
// static void AdaptiveCalTimeoutProc(ClockP_Object *clock, void *argv);
void AdaptiveCalClockLaunch(uint64_t period);
int32_t Private_Can_CalibrtionDebug(uint32_t ID,uint8_t* Data,uint8_t lenth);
void Calibration_Progress(uint8_t pace);
uint8_t Body_Posture_Detection(void);
uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList,uint8_t i);
//void SaveAdaptiveCalStatus_DID_0x4902(uint8_t * StatusArray);
void angle_deviation_detection(const or_point_cloud_format_t *PeakList);

void Installation_Angle_check(const or_point_cloud_format_t *PeakList);
bool Angle_Difference_judg(void);
#endif /* __ANGLE_CONFIG_H */

    /******************************END OF FILE*************************************/
