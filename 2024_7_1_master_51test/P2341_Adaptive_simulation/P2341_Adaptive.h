#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#define GTRACK_NUM_POINTS_MAX 128

#define CalibrationTime       180
#define Timeframe             5
#define CalibrationRangeMin   3
#define CalibrationRangeMid   21
#define CalibrationRangeMax   40//50//80//60//40//100
#define AdaptiveCalibration_OutputB       0
//#define AdaptiveCalibration_OutputB       1

#define Calibration_MaxSteeringAngle    20
#define Calibration_MaxYawRate          20
#define Calibration_MinVelocity         5
#define Calibration_MaxVelocity         12.5
#define Calibration_MaxRoadCurve        300
#define Calibration_MinRCs              10//30//40//45//30


#define RadarInstallAngle            35

#define max(a,b) ((a) > (b) ? (a) : (b))
#define PI                          3.14159265358979f

typedef signed char int8_t;
typedef unsigned char   uint8_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;
typedef int  int32_t;
typedef unsigned   uint32_t;
typedef float float32_t;

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
    uint8_t      Flow_Receive;                  //连续接收
    uint16_t     Times;                         //标定计时
    uint8_t      Master_Calibration;            //主雷达标定
    uint8_t      Master_Result;                 //主雷达标定结果    0为未标定，1为标定成功，2为标定失败
    uint8_t      Slave_Calibration;             //从雷达标定
    uint8_t      Slave_Result;                  //从雷达标定结果
    uint8_t      MasterSlave_OldState;          //主从雷达之前的标定结果
    uint8_t      MasterSlave_faultState;          //主从雷达之前的标定结果
    uint16_t     Slave_Times;                   //从雷达更新时间
    uint8_t      OnState;                       //从雷达标定的状态
    uint8_t      Slave_AngleValue[2];           //从雷达标定的角度信息
    
    /***************下线标定参数**********************/
    uint8_t      Num;                           //下线标定的步骤
    float32_t    Angle_OffsetSum;                  //下线标定角度偏差总和
    float32_t    Angle_elevOffsetSum;              //下线垂直标定角度偏差总和
    float32_t    Angle_OffsetAdv;                  //平均下线标定角度偏差
    float32_t    Angle_elevOffsetAdv;              //平均下线垂直标定角度偏差
    float32_t    ChackOffset;                      //标定后水平角度偏差检验
    float32_t    ChackelevOffset;                  //标定后垂直角度偏差检验
    uint16_t     Error_Number;                  //下线标定错误计数
    uint8_t      Recalibration_Number;          //下线标定重新校准计数
    float32_t    Angle_Value;                   //下线标定需要标到的水平角度
    float32_t    Angle_elev;                    //下线标定需要标到的垂直角度

    /***************下线标定参数**********************/
    
    /***************自适应标定参数**********************/
    uint8_t     Adaptive_step;                   //自适应标定步骤
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
    uint8_t      TEMP_PB;               //进度标志
    uint8_t      adaptive_PB;               //实时进度
    /***************标定参数**********************/
    float32_t    OfflineCalibration_Range; //目标径向距离
    float32_t    OfflineCalibration_CrossRange; //目标横向距离
    float32_t    OfflineCalibration_LongtitudeRange; //目标纵向距离
    uint8_t      Driving_Profile;//驾驶指导
    uint8_t      errType;
    uint16_t Calibration_Counter;
    uint8_t Calibration_status;
}CalibrationParaS;


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

        uint16_t point_count;
        
    } GTRACK_measurementPoint;


void Adaptive_CalibrationInit(void);
void Adaptive_CalibrationClear(void);
void Adaptive_Calibration(uint32_t gNumPoints, GTRACK_measurementPoint *PeakList);
void Adaptive_CalibrationSaveData(uint32_t gNumPoints, GTRACK_measurementPoint *PeakList);
void Adaptive_CalibrationPolyFit(void);
void Adaptive_CalibrationFinish(void);

void Adaptive_Calibration_Proc(void);
void Adaptive_CalibrationSlave_Exit(uint16_t Exit_Times);