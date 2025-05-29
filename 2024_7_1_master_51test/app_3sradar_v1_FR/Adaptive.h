
/****************************************************************************
 *                        File: adaptive_calibration.h                      *
 *                        @sjw20210713                                           *
 ****************************************************************************/

  
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
#include "main.h"
#include "calibration_common.h"

#define max(a,b) ((a) > (b) ? (a) : (b))





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
    or_point_cloud_format_t or_point_cloud_format_t;
}Calibration_Date;


///////////////////////////////////////////////////////////////////
#define ADAPTIVE_WORKMODE_CHECK_SUCCESS                      (0)
#define ADAPTIVE_WORKMODE_CHECK_NO_SUCCESS                   (-1)

#define ADAPTIVE_INIT_SUCCESS                                (0)
#define ADAPTIVE_INIT_NO_SUCCESS                             (-1)


// 常量定义

#define KMH_TO_MS(kmh)                                       ((kmh) / 3.6f)
#define SPEED_THRESHOLD_LOW                                  0.15f // 低速阈值系数
#define SPEED_THRESHOLD_HIGH                                 0.15f // 高速阈值系数
#define SPEED_SEGMENT_LOW                                    4.1f  // 低速分界点 (m/s)
#define SPEED_SEGMENT_HIGH                                   8.3f  // 高速分界点 (m/s)
#define AZIMUTH_ANGLE_MIN                                    0.0f // 最大允许方位角 (度)
#define AZIMUTH_ANGLE_MAX                                    55.0f // 最大允许方位角 (度)

#define EPSILON                                    1e-9
/*******************************Body attitude restriction*********************/
#define ADAPTIVE_MAX_STEERING_ANGLE    10.0f//5.0f//10//20 

#define ADAPTIVE_MAX_YAWRATE          0.8f

#define ADAPTIVE_MIN_VELOCITY         4.0f

#define ADAPTIVE_MAX_VELOCITY          19.5f

#define ADAPTIVE_MIN_CURVERADIUS           300.0f



#define STEERING_TOLERANCE                                   3.0f // 方向盘转角允许偏差(度)

#define VELOCITY_TOLERANCE_MS                                3.0f // 车速允许偏差(m/s)
/*******************************Body attitude restriction*********************/


#define RANGE_MIN                                            5.0f  // 有效距离下限 (米)
#define RANGE_MAX                                            25.0f // 有效距离上限 (米)
#define Y_SEGMENT_LOW                                        4.01f // Y轴低区上限
#define Y_SEGMENT_HIGH                                       8.01f // Y轴高区上限

#define YDATA_LOWER_BOUND_CASE1                              0.5f
#define YDATA_UPPER_BOUND_CASE1                              7.0f
#define PROGRESS_STEP                                        14
#define INITIAL_PROGRESS                                     5
#define FINAL_PROGRESS_OFFSET                                8

#define CALIBRATION_MIN_SAMPLES                              10 // 标定最小数据点数

#define ADAPTIVE_MIN_RCS                                    15.0f//5
#define ADAPTIVE_YDATA_GAP                                  2.0f //1.5f
#define X_DISTANCE_MIN                                       8.0f // X轴最小有效距离
#define X_DISTANCE_MAX                                       65.0f // X轴最小有效距离
#define YDATA_GAP_ADJUST                                     0.2f // Y轴间隙调整量
#define CALIB_TIMEFRAME_HALF                                 (CALIBRATION_MIN_SAMPLES / 2 + 1)
#define CALIB_FAIL_FRAME_THRESH                              (ADAPTIVE_FRAME_NUM / 18)

#define DENSE_ZONE_HIGH        2
#define DENSE_ZONE_LOW          1


#define ADAPTIVE_FRAME_NUM 180    //自适应标定帧数
#define SINGLE_DATA_AMOUNT  ADAPTIVE_FRAME_NUM * CALIBRATION_MIN_SAMPLES //单次标定的数据总量
#define ADAPTIVE_COUNTER             7     //标定次数

#define ADAPTIVE_CAL_TIMEOUT_CYCLE (10 * 60 * 1000 * 1000u) /* 10min */

/*calibration_adaptive_error_kind_t*/
typedef enum _calibration_adaptive_error_kind_type{
    CALIBRATION_ADAPTIVE_IS_SUCCESSFUL = 0,
    CALIBRATION_ADAPTIVE_IN_PROGRESS,
    CALIBRATION_ADAPTIVE_IS_FAIL
}calibration_adaptive_error_kind_t;

typedef struct calibration_adaptive_result_content_type{
    calibration_adaptive_error_kind_t last_result;
    float32_t                         adapt_angle_h;
    float32_t                         adapt_angle_v;
}calibration_adaptive_result_content_t;



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
    float32_t         Temp_A[7];
    float32_t         Temp_ele[7];
    float32_t         SteeringAngle;                      //存储标定开始时的方向盘转角
    float32_t         YawRate;                            //存储标定开始时的YawRate
    float32_t         Velocity;                           //存储标定开始时的车速

    float32_t         angle_buffer_h[ADAPTIVE_COUNTER];
    float32_t         angle_buffer_v[ADAPTIVE_COUNTER];
    float32_t         apat_angle_h;
    float32_t         apat_angle_v;
    uint16_t          Frame;                              //标定帧数
    uint16_t          DataNum;                            //每次标定的数据数量
    uint16_t          ChackDataNum;                       //验证需要的数据数量
    uint8_t           Counter;                            //标定次数
    uint8_t           FalseFrame;                         //连续无目标的帧数
    uint8_t           TEMP_PB;           //进度标志
    uint8_t           adaptive_PB;       //实时进度
    uint8_t           driving_profile;  
    uint8_t           errType;
    uint8_t   Step;          //雷达标定的步骤
    //ADAPTIVE_WORKMODE adaptive_Workmode; //实时进度

} adaptive_calibrationparas;

typedef  struct calibtion_adaptive_format_type{
    calibration_adaptive_result_content_t adap_result;  //标定结果
    adaptive_calibrationparas adaptve_calibrationpara;                 //标定参数
    cal_extern_para_t adap_extern_para;       //外部参数
}calib_adapt_format_t;



// 定义单帧最大数据点数量和维度
#define MAX_POINTS 300
#define DIMENSIONS 2
typedef struct {
    double coords[DIMENSIONS]; // 坐标（示例为二维）
    bool   visited;            // 是否被访问过
    int    cluster_id;         // 簇ID（-1表示噪声）
} Point;

typedef struct {
    float x; // 前向距离
    float y; // 横向偏移
    float z; // 高程信息
} Point3D;


typedef enum {
    ADAPTIVE_START = 0X00,
    ADAPTIVE_WORKMODE_CHECK,
    ADAPTIVE_INIT,
    ADAPTIVE_BODY_POSTURE_DETECTION,
    ADAPTIVE_DATUM_SELECTION,
    ADAPTIVE_FINISH,
    ADAPTIVE_END,
} ADAPTIVE_STEP;


//////////////////////////////////////////////////////////////////////
//初始化






typedef enum
{
    CHEACK_MODE = 0x00,
    ADAPTIVE_MODE,
}ADAPTIVE_WORKMODE;


//static ClockP_Object adaptive_cal_clock;
bool offline_start_func();

void Calibration_Progress(uint8_t pace);

void Adaptive_Calibration_Exit(uint8_t* StatusArray);  //退出标定

uint8_t AdaptiveCalStart(void);

void GetAdaptiveCalStatus(uint8_t * StatusArray);
int32_t adaptive_flow_control_func(const or_point_cloud_format_t *PeakList);
//static void AdaptiveCalTimeoutProc(ClockP_Object *clock, void *argv);

//void AdaptiveCalClockLaunch(uint64_t period);



#endif /* __ANGLE_CONFIG_H */

    /******************************END OF FILE*************************************/
