#ifndef __REALTIME_CALIBRATION_H
#define __REALTIME_CALIBRATION_H

/* Includes ------------------------------------------------------------------*/

#include "adaptive_calibration.h"

/***************选点参数*******************/
#define CalibrationRangeMin   8.0f//15//20//15//3//3    最小标定距离
#define CalibrationRangeMid   30.0f     
#define CalibrationRangeMax   65.0f//60.0f//40.0f//50.0f//40.0f//50//40//50//45//50//25//35//12//35       //最大标定距离35
#define ADAPTIVE_MAX_AZIMUTH  55.0f
#define ADAPTIVE_MIN_AZIMUTH  0.0f
#define Calibration_MinRCs              15.0f//5//30//30//40//45//30
#define Calibration_Ydata_gap                1.0f //1.5f//2.0f//1.0f//0.7f//1.0f//0.7f//1.0f//0.5f//1.0f
/***************选点参数*******************/
#define CLEAR_ANGLE 0
#define NOT_CLEAR_ANGLE 1
 
/***********车身姿态参数************************ */
#define Calibration_MaxSteeringAngle    10.0f//5.0f//10//20   
#define Calibration_MaxYawRate          0.8f
#define Calibration_MinVelocity         4.0f
#define Calibration_MaxVelocity         19.5f
#define Calibration_MaxRoadCurve        300.0f
#define STEERING_TOLERANCE                                   3.0f // 方向盘转角允许偏差(度)
#define VELOCITY_TOLERANCE_MS                                3.0f // 车速允许偏差(m/s)
/************************************************/

void realtime_calibration_func(const or_point_cloud_format_t *PeakList);
#endif