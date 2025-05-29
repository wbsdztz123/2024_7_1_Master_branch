#ifndef _CALIBRATION_COMMON_H_
#define _CALIBRATION_COMMON_H_


extern int _3sradar_tui_print(const char *format, ...);

#define RAD_TO_DEG(rad)                                      ((rad) * 57.2957795f) // 180/PI ≈ 57.2957795
#define DEG_TO_RAD(deg)                                      ((deg) * 0.0174532925f)
#define Calibration_Tolerance               5.0f //标定水平容差
#define Calibration_elevTolerance           3.0f //标定垂直容差
#define OfflineCalibration_elevTolerance_authentic 5.5f //真实下线标定垂直容差

typedef signed char int8_t;
typedef unsigned char   uint8_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;
typedef int  int32_t;
typedef unsigned   uint32_t;
typedef float float32_t;


typedef enum
{
    INSTALL_FRONT = 0,
    INSTALL_BACK,
    INSTALL_LEFT_FRONT,
    INSTALL_RIGHT_FRONT,
    INSTALL_LEFT_BACK ,
    INSTALL_RIGHT_BACK,
}InstallPositionT;

typedef struct cal_external_para_type{
    InstallPositionT  cal_installation;                      //雷达安装位置
    float32_t        cal_install_angle;                            //雷达安装角度
}cal_extern_para_t; 


typedef struct calibration_result_type{
    uint8_t          cal_result;            
    float32_t        cal_horiztal_angle;   //水平角  自适应+下线
    float32_t        cal_pitch_angle;      //俯仰角  自适应+下线                
}cal_result_t; 




#endif /* __ANGLE_CONFIG_H */