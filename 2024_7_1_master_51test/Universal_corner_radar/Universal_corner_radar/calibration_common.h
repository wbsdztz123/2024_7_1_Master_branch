#ifndef _CALIBRATION_COMMON_H_
#define _CALIBRATION_COMMON_H_
#include <stdint.h>
#include <stdbool.h>    
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <minwindef.h>
#include <unistd.h>
#include "common_api.h"

#define DIRECTLY_BEHIND_IS_NEGATIVE    false//正后方负角度
#define DIRECTLY_BEHIND_IS_POSITIVE    true     //正后方正角度
#define INSTALL_DIRECTION    DIRECTLY_BEHIND_IS_NEGATIVE   //雷达安装方向 1正 -1负

// #define true 1
// #define false 0
//#define NULL 0
#define PI 3.14159265358979323846f
// #define TRUE 1
// #define FALSE 0

#define KMH_TO_MS(kmh) ((kmh) / 3.6)
#define RAD_TO_DEG(rad)                                      ((rad) * 57.2957795f) // 180/PI ≈ 57.2957795
#define DEG_TO_RAD(deg)                                      ((deg) * 0.0174532925f)

#define EPSILON 1e-9
#define MAX_ANGLE_DEVIATION 10.0f //角度偏差最大值
#define MAX_DISTANCE_DEVIATION 0.5f //距离偏差最大值
#define MAX_ANGLE_DEVIATION_AUTH 15.0f //真实下线角度偏差最大值
#define MAX_DISTANCE_DEVIATION_AUTH 1.0f //真实下线距离偏差最大值

typedef enum {
    CALIBRATION_INIT = 0x00,
    CALIBRATION_RUNING,
    DATA_READ_EXIT,
    CALIBRATION_EXIT
} CALIBRATION_MODE;


/* Includes ------------------------------------------------------------------*/
#define RAD_TO_DEG(rad)                                      ((rad) * 57.2957795f) // 180/PI ≈ 57.2957795
#define DEG_TO_RAD(deg)                                      ((deg) * 0.0174532925f)
#define Calibration_Tolerance               5.0f //标定水平容差
#define Calibration_elevTolerance           3.0f //标定垂直容差
#define OfflineCalibration_elevTolerance_authentic 5.5f //真实下线标定垂直容差
#define EPSILON                                    1e-9

#define DataLength_Data_4901          1
#define DataLength_Data_4902          6
/*******************adaptive******************/ 

/*calibration_adaptive_result_kind_t*/
typedef enum _calibration_adaptive_result_kind_type{
    CALIBRATION_ADAPTIVE_IS_SUCCESSFUL = 0,  //成功
    CALIBRATION_ADAPTIVE_IN_PROGRESS,        //进行中
    CALIBRATION_ADAPTIVE_IS_FAIL            //失败
}calibration_adaptive_result_kind_t;

typedef enum _calibration_adaptive_error_kind_type{
    CALIBRATION_ADAPTIVE_IS_NO_FAIL = 0,    //成功
    CALIBRATION_ADAPTIVE_IN_TIMEOUT,          //超时
    CALIBRATION_ADAPTIVE_IS_ANGLE_OVERSHOOT, //角度超差
    CALIBRATION_ADAPTIVE_NUM_WRITE_ERROR //NUM写入错误
}calibration_adaptive_error_kind_t;

typedef struct adaptive_content_type{
    calibration_adaptive_result_kind_t output_result;   //结果
    uint8_t                            output_progress;  //进度
    calibration_adaptive_error_kind_t  output_cause_of_failure;  //失败原因
    float32_t                          output_adapt_angle_h;  ///水平角
    float32_t                          output_adapt_angle_v;  //俯仰角
}adaptive_status_out_t;

typedef struct calibration_adaptive_result_content_type{
    calibration_adaptive_result_kind_t last_result;
    float32_t                         adapt_angle_h;
    float32_t                         adapt_angle_v;
}calibration_adaptive_result_content_t;
/*********************adaptive******************/ 

typedef enum
{
    INSTALL_LEFT_BACK = 0,
    INSTALL_RIGHT_BACK,
    INSTALL_LEFT_FRONT,
    INSTALL_RIGHT_FRONT,
    INSTALL_FRONT,
    INSTALL_BACK,
}InstallPositionT;

/*************common struct**************/
typedef struct cal_external_para_type{
    InstallPositionT  cal_installation;                      //雷达安装位置
    float32_t        cal_install_angle;                            //雷达安装角度
}cal_extern_para_t; 

typedef struct calibration_result_type{
    uint8_t          cal_result;            
    float32_t        cal_horiztal_angle;   //水平角  自适应+下线
    float32_t        cal_pitch_angle;      //俯仰角  自适应+下线                
}cal_result_t; 
/************************************/

/*******************offline******************/ 
typedef enum {
    CALIBRATION_SUCCESS = 0x00,
    CALIBRATION_IN_PROGRESS,
    CALIBRATION_TIMEOUT,
    VERTICAL_DEVIATION_OVERFLOW,
    HORIZONTAL_DEVIATION_OVERFLOW,
    TARGET_LOST,
    WRITE_NVM_ERROR,
    INVALID_ANGLE_DATA,
} OFFINE_ERROR_TYPE;

typedef struct offline_content_type
{
    float32_t horizontal_angle_deviation;
    float32_t vertical_angle_deviation;
    OFFINE_ERROR_TYPE Calibration_Status;
}offline_status_out_t;
/**********offline******************/ 

offline_status_out_t *offline_cal_get_status(void);
const offline_status_out_t* offline_status_out_func(void);
void GetOfflineCalStatus(uint8_t *StatusArray);
float GetCalHorizontalAngle(void);
float GetCalVertiAngle(void);
bool GetDtcCalOutOfRange(void);
bool offline_start_func(void);
void offline_calibrate_func(const or_point_cloud_format_t *PeakList);
int  switching_mode_debug();
void adaptive_calibrate_func(const or_point_cloud_format_t *PeakList);
void GetAdaptiveCalStatus(uint8_t *StatusArray);
bool adaptive_start_func(void);
calibration_adaptive_result_content_t* get_adapt_status(void);
//void SaveLastCalStatus(uint8_t LastStatus);
void SaveAdaptiveCalStatus_DID_0x4902(uint8_t * StatusArray);
void SaveOfflineCalStatus_DID_0x4901(uint8_t * StatusArray);

extern CALIBRATION_MODE CAL_MODE;
extern bool adaptive_start(void);
extern void adaptive_real_time_status_set_func(void);
extern int32_t adaptive_flow_control_func(const or_point_cloud_format_t *PeakList);
extern int32_t offline_flow_control_func(const or_point_cloud_format_t *PeakList);
extern bool offline_start(void);
extern int32_t offline_real_time_status_set_func(void);
extern calibration_adaptive_result_content_t adap_result;
extern offline_status_out_t offline_status_out;
extern cal_extern_para_t calibration_extern_para;
extern adaptive_status_out_t adap_status_out;

#endif /* __ANGLE_CONFIG_H */