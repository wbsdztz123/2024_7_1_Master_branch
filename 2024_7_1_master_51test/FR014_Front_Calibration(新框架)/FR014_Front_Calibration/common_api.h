#ifndef COMMON_API_H
#define COMMON_API_H

#define int8_t char
#define uint64_t unsigned long long
#define float32_t float
#define uint8_t unsigned char
#define uint16_t unsigned short
#define int16_t short
#define uint32_t unsigned int
#define int32_t int
#define GTRACK_NUM_POINTS_MAX 300

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
    //CalibrationParaS CalibrationParaS;
    or_point_cloud_format_t or_point_cloud_format_t;

}Calibration_Date;






extern Calibration_Date Calibration_Message;
extern Message_VehicleMsgS Message_VehicleMsg;
extern RadarParaS RadarPara;
#endif // COMMON_API_H