#include "realtime_calibration.h"
#include <stdarg.h>

extern adaptive_calibrationparas CalibrationPara;
extern void YD_XD_writing(float YD,float XD);


static void realtime_calibration_clean(uint8_t control_flag);
static uint8_t realtime_body_posture_detection();
static void realtime_Calibration_data_collection(const or_point_cloud_format_t *PeakList);
static void adaptive_calibration_result_collection();
static float32_t get_adaptive_calibration_this_result();
static bool realtime_calibration_run_or_end();//true:runing false:end
void Adaptive_CalibrationPolyFit(void);
uint8_t Rang_judge(const or_point_cloud_format_t *PeakList);

//uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList, uint8_t i);

/*
uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList, uint8_t i)
{
    uint8_t Calibration_flag = 0;
    float32_t temp_speed_gap = 0xff;
    
    if (PeakList->term[i].range > CalibrationRangeMin 
        && PeakList->term[i].range < CalibrationRangeMax
        && (PeakList->term[i].azimuth * 180 / PI) > ADAPTIVE_MIN_AZIMUTH
        && (PeakList->term[i].azimuth * 180 / PI) < ADAPTIVE_MAX_AZIMUTH
        && PeakList->term[i].snr >= Calibration_MinRCs)
    {
        bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_LEFT_FRONT || RadarPara.InstallPosition == INSTALL_RIGHT_FRONT);
        bool isDopplerNegative = PeakList->term[i].doppler < 0.0f;

        if ((isLeftOrRightFront && isDopplerNegative) || (!isLeftOrRightFront && !isDopplerNegative))
        {
            float32_t cosValue = cos((RadarPara.InstallAngle - (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
            float32_t speed = Message_VehicleMsg.Velocity / 3.6f;
            float32_t threshold = 0.15f;

            if (speed >= 4.1f && speed < 8.3f)
            {
                threshold = 0.1f;
            }
            else if (speed >= 8.3f)
            {
                threshold = 0.08f;
            }

            if (isLeftOrRightFront)
            {
                temp_speed_gap = fabs(PeakList->term[i].doppler / cosValue + speed);
            }
            else
            {
                temp_speed_gap = fabs(PeakList->term[i].doppler / cosValue - speed);
            }

            Calibration_flag = (temp_speed_gap < speed * threshold);
        }
    }

    return Calibration_flag;
}

*/







uint8_t Rang_judge(const or_point_cloud_format_t *PeakList)
{
    #define TARGET_MIN_RANGE 5.0f
    #define TARGET_MAX_RANGE 25.0f
    uint32_t i = 0;
    uint8_t result = 0;
    uint16_t rang0_2 = 0, rang2_4 = 0, rang4_7 = 0;
    float32_t azimuth_deg = 0.0f;
    for (i = 0; i < PeakList->point_count; i++)
    {
        azimuth_deg = RAD_TO_DEG(PeakList->term[i].azimuth);

        result = CAL_Target_Filtering(PeakList, i); 
        if((PeakList->term[i].range > TARGET_MIN_RANGE) && (PeakList->term[i].range < TARGET_MAX_RANGE))
        {
        if (result)
        {
            float32_t temp_Ydata = PeakList->term[i].range * sin(((PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
            if (temp_Ydata < 2.01f) {
                rang0_2++;
            } else if (temp_Ydata < 4.01f) {
                rang2_4++;
            } else {
                rang4_7++;
            }
        }
    }
    }
    if (rang0_2 > rang2_4 && rang0_2 > rang4_7)
    {
        return 1;
    }
    else if (rang2_4 > rang0_2 && rang2_4 > rang4_7)
    {
        return 2;
    }
    else if (rang4_7 > rang2_4 && rang4_7 > rang0_2)
    {
        return 3;
    }
    
    return 0;
}


void Adaptive_CalibrationPolyFit(void)
{
    float32_t sum_x2 = 0;
    float32_t sum_y  = 0;
    float32_t sum_x  = 0;
    float32_t sum_xy = 0;
    uint32_t  i      = 0;
    float32_t a;
    float32_t b;

    for (i = 0; i < CalibrationPara.DataNum; i++) 
    {
        float32_t x = CalibrationPara.xdata[i];
        float32_t y = CalibrationPara.ydata[i];

                                    // if((CalibrationPara.AveYdata > 2.5f)&&(CalibrationPara.cal_col_num < 1))
                                    // {
                                    //     YD_XD_writing(y,x);
                                    // }

        sum_x2 += x * x;
        sum_y += y;
        sum_x += x;
        sum_xy += x * y;
    }

    float32_t denominator = CalibrationPara.DataNum * sum_x2 - sum_x * sum_x;

    a = (CalibrationPara.DataNum * sum_xy - sum_x * sum_y) / denominator;
    b = (sum_x2 * sum_y - sum_x * sum_xy) / denominator;

    CalibrationPara.Adap_B     = b;
    CalibrationPara.Adap_A     = a;
    CalibrationPara.Adap_Angle = atan(a) * 180 / PI;

#ifdef _CALIBRATION_DEBUG_

        int32_t  tmp_data;
        uint8_t data[8] = {0};   
    
        tmp_data = (int32_t)((CalibrationPara.Adap_Angle) * 10);
        data[0] = (uint8_t)((tmp_data >> 16) & 0xFF);
        data[1] = (uint8_t)((tmp_data >> 8) & 0xFF);
        data[2] = (uint8_t)(tmp_data & 0xFF);

        tmp_data = (int32_t)((CalibrationPara.Adap_B) * 10);
        data[3] = (uint8_t)((tmp_data >> 16) & 0xFF);
        data[4] = (uint8_t)((tmp_data >> 8) & 0xFF);
        data[5] = (uint8_t)(tmp_data & 0xFF);
        data[6] = 0xFF;
        Private_Can_CalibrtionDebug(0x719,data,7);
    /*******debug*****/
#endif
}

void realtime_calibration_init(void)
{
    memset(&CalibrationPara, 0, sizeof(adaptive_calibrationparas));
}

static void realtime_calibration_clean(uint8_t control_flag)
{
    uint16_t i;
    CalibrationPara.Start         = 0;
    CalibrationPara.SteeringAngle = 0;
    CalibrationPara.YawRate       = 0;
    CalibrationPara.Velocity      = 0;
    CalibrationPara.Frame         = 0;
    CalibrationPara.FalseFrame    = 0;
    CalibrationPara.DataNum       = 0;
    CalibrationPara.AveYdata      = 0;
    CalibrationPara.Adap_A        = 0;
    CalibrationPara.Adap_B        = 0;


    for (i = 0; i < SINGLE_DATA_AMOUNT; i++) {
        CalibrationPara.xdata[i] = 0;
        CalibrationPara.ydata[i] = 0;
    }
    CalibrationPara.Adap_Angle      = 0;
    CalibrationPara.Adap_eleAngle   = 0;

    //CalibrationPara.Counter = 0;

    // for (i = 0; i < ADAPTIVE_RESULT_NUM;i++) {
    //     CalibrationPara.Temp_A[i] = 0;
    //     CalibrationPara.Temp_ele[i] = 0;
    // }
    //清除本行内容 
if(!control_flag)
{
        for(int i = 0; i < COL; i++)
        {
            CalibrationPara.realtime_cal_res[CalibrationPara.cal_row_num*COL + i] = 0;
        }
        CalibrationPara.cal_col_num =  0;   //从新填充本次标定角度
}

    return;
}

static uint8_t realtime_body_posture_detection()
{

#define REALTIME_CAL_MAX_steeringangle        7.0f
#define REALTIME_CAL_MAX_YawRate         0.5f

#define REALTIME_CAL_MIN_VELOCITY         4.0f
#define REALTIME_CAL_MAX_VELOCITY         28.0f
#define Calibration_MaxRoadCurve        300.0f

    uint8_t result = 0;
    float velocity = KMH_TO_MS(Message_VehicleMsg.Velocity);
    float steeringAngle = fabs(Message_VehicleMsg.SteeringAngle);
    float curveRadius = fabs(Message_VehicleMsg.CurveRadius);
    float yawRate = fabs(Message_VehicleMsg.YawRate);

    result = (velocity > REALTIME_CAL_MIN_VELOCITY ) && (velocity < REALTIME_CAL_MAX_VELOCITY) 
             && (steeringAngle < REALTIME_CAL_MAX_steeringangle) 
             && (curveRadius > Calibration_MaxRoadCurve) 
             && (yawRate < REALTIME_CAL_MAX_YawRate);

    return result;

}
// 数据保存
static void realtime_Calibration_data_collection(const or_point_cloud_format_t *PeakList)
{
    uint8_t  flag;
    uint8_t  Calibration_flag;
    uint32_t i;
    uint32_t  Start_num = 0;
    float32_t temp_speed_gap = 0xff;
    float32_t temp_Ydata;
    uint8_t   rang_flag = 0;
    flag         = realtime_body_posture_detection(); //车身姿态检测

    if (flag) // 车辆处于可以标定的状态，进行标定
    {
            CalibrationPara.Error_Number = 0; // 车速、档位、转弯正确，则时间清零。
            if (CalibrationPara.Start == 0) // 没有开始标定
            {
                CalibrationPara.AveYdata = 0;
                rang_flag                = Rang_judge(PeakList); // 判断栅栏距离

                float lower_bound, upper_bound;
                switch (rang_flag) {
                case 2:
                    lower_bound = 2.0f;
                    upper_bound = 4.2f;
                    break;
                case 1:
                    lower_bound = 0.5f;
                    upper_bound = 2.2f;
                    break;
                default:
                    lower_bound = 3.9f;
                    upper_bound = 7.0f;
                    break;
                }

                for (i = 0; i < PeakList->point_count; i++) // 读取车辆旁边障碍物的位置，存储数据
                {
                    Calibration_flag = CAL_Target_Filtering(PeakList, i);
                    if (Calibration_flag && rang_flag) {
                        float azimuth = PeakList->term[i].azimuth * 180 / PI;
                        temp_Ydata    = PeakList->term[i].range * sin((azimuth) * PI / 180);
                        if (temp_Ydata > lower_bound && temp_Ydata < upper_bound)
                        {
                            Start_num++;
                            CalibrationPara.AveYdata += temp_Ydata;
                        }
                    }
                }
                if (Start_num >= CALIBRATION_MIN_SAMPLES) {
                    CalibrationPara.AveYdata = CalibrationPara.AveYdata / Start_num;
                    if ((CalibrationPara.AveYdata > 0.5f) && (CalibrationPara.AveYdata < 7.0f)) //3.0-4.0-4.5-5.0-6.0-8.0
                    {
                        
                        CalibrationPara.Start = 1;
                        CalibrationPara.SteeringAngle = Message_VehicleMsg.SteeringAngle; // 存储标定开始的方向盘转角
                        CalibrationPara.Velocity = KMH_TO_MS(Message_VehicleMsg.Velocity); // 存储标定开始的车速
                    } else {
                        realtime_calibration_clean(CLEAR_ANGLE); // 标定放弃
                    }
                } else {
                    realtime_calibration_clean(CLEAR_ANGLE); // 标定放弃
                }
            
            } else // 已经开始标定
            {
                if ((fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle) < 3.0f) //5//6//10// 方向盘转角偏差小于10才行。
                    && (fabs((KMH_TO_MS(Message_VehicleMsg.Velocity)) - CalibrationPara.Velocity) < 3.0f)) // 3
                {
                    Start_num = 0;
                    for (i = 0; i < PeakList->point_count; i++) {
                        Calibration_flag = CAL_Target_Filtering(PeakList, i);

                        if (Calibration_flag) {
                            float azimuth_deg = PeakList->term[i].azimuth * 180 / PI;
                            float X = PeakList->term[i].range * cos((azimuth_deg) * PI / 180);
                            float Y = PeakList->term[i].range * sin((azimuth_deg) * PI / 180);

                            if (CalibrationPara.AveYdata < 2.2f) {
                                if (CalibrationPara.DataNum < SINGLE_DATA_AMOUNT
                                    && Y > (CalibrationPara.AveYdata - (Calibration_Ydata_gap - 0.2))
                                    && Y < (CalibrationPara.AveYdata + (Calibration_Ydata_gap - 0.2)) && X > 8.0f) {
                                    CalibrationPara.xdata[CalibrationPara.DataNum] = X;
                                    CalibrationPara.ydata[CalibrationPara.DataNum] = Y;
                                    CalibrationPara.DataNum++;
                                    Start_num++;
                                }
                            } else {
                                if (CalibrationPara.DataNum < SINGLE_DATA_AMOUNT
                                    && Y > (CalibrationPara.AveYdata - Calibration_Ydata_gap)
                                    && Y < (CalibrationPara.AveYdata + Calibration_Ydata_gap) && X > 8.0f) {
                                    CalibrationPara.xdata[CalibrationPara.DataNum] = X;
                                    CalibrationPara.ydata[CalibrationPara.DataNum] = Y;
                                    CalibrationPara.DataNum++;
                                    Start_num++;

                                    
                                }
                            }
                        }
                    }


                    CalibrationPara.Frame++; // 帧数+1
                    if (Start_num <= (CALIBRATION_MIN_SAMPLES / 2 + 1)) {
                            CalibrationPara.FalseFrame++; // 连续无目标的帧数
                        if (CalibrationPara.FalseFrame > (ADAPTIVE_FRAME_NUM / 18)) //10
                        {
                            realtime_calibration_clean(CLEAR_ANGLE);       // 标定放弃
                        }
                    } else {
                        CalibrationPara.FalseFrame = 0;
                    }
                } else {
                    realtime_calibration_clean(CLEAR_ANGLE); // 标定放弃
                }
            }
    } else {
        CalibrationPara.Error_Number++; // 车速、档位、转弯半径不对的时候，不进行标定
        realtime_calibration_clean(CLEAR_ANGLE);    // 标定放弃
    }
}

static float32_t get_adaptive_calibration_this_result(void)
{

    float32_t median_angle_H = 0;
    for(int i = 0; i < (CalibrationPara.cal_col_num) ; i++)
    {
        for(int j = 0;j < (CalibrationPara.cal_col_num - i - 1); j++)
        {
            
            if(CalibrationPara.Temp_A[j] > CalibrationPara.Temp_A[j + 1])
            {
                median_angle_H = CalibrationPara.Temp_A[j];
                CalibrationPara.Temp_A[j] = CalibrationPara.Temp_A[j + 1];
                CalibrationPara.Temp_A[j + 1] = median_angle_H;
            }
        }
    }
     median_angle_H = CalibrationPara.Temp_A[3];
    return median_angle_H;
}

static void adaptive_calibration_result_collection(void)
{

    //printf("22222222CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num = %d\n",CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num);
    float32_t TmpLinearAngle = 0;
    float32_t TmpLineareleAngle = 0;

    if ((CalibrationPara.Adap_B > 0.5f) && (CalibrationPara.Adap_B < 7.0f))
    {
            TmpLinearAngle = CalibrationPara.Adap_Angle;
            TmpLineareleAngle = CalibrationPara.Adap_eleAngle;

            CalibrationPara.Temp_A[CalibrationPara.cal_col_num] = TmpLinearAngle;

            CalibrationPara.realtime_cal_res[CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num] = TmpLinearAngle;

            CalibrationPara.cal_index = CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num; //当前标定次数
           // 
            printf("CalibrationPara.realtime_cal_res[%d] = %f\n",CalibrationPara.cal_index,CalibrationPara.realtime_cal_res[CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num]);
            //printf("CalibrationPara.cal_index = %d\n",CalibrationPara.cal_index);
            CalibrationPara.cal_col_num += 1;
            
    //printf("33333333CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num = %d\n",CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num);
            if(CalibrationPara.cal_col_num > ADAPTIVE_COUNTER)
            {
                if(CalibrationPara.Adap_Angle > 1.0f)
                {
                    printf("Calibration.yd = %f\n",CalibrationPara.Adap_B);
                }
                
                CalibrationPara.real_cal_res_median[CalibrationPara.cal_row_num] = get_adaptive_calibration_this_result();
               // printf("CalibrationPara.real_cal_res_median[%d] = %f\n",CalibrationPara.cal_row_num,CalibrationPara.real_cal_res_median[CalibrationPara.cal_row_num]);
                CalibrationPara.cal_col_num = 0;

                CalibrationPara.cal_row_num += 1;

            } else {
                realtime_calibration_clean(NOT_CLEAR_ANGLE);

            }
    }
    else
    {
        realtime_calibration_clean(CLEAR_ANGLE);
    }
}

static bool realtime_calibration_run_or_end(void)
{
    return (CalibrationPara.cal_row_num < ROW);
}

void realtime_calibration_func(const or_point_cloud_format_t *PeakList)
{
    if (realtime_calibration_run_or_end()) // 标定未结束
    {
        realtime_Calibration_data_collection(PeakList); // 保存数据
        if (CalibrationPara.FalseFrame > (ADAPTIVE_FRAME_NUM / 10))
        {
            realtime_calibration_clean(CLEAR_ANGLE); // 标定放弃
        }
        else if (CalibrationPara.Frame >= ADAPTIVE_FRAME_NUM || CalibrationPara.DataNum >= SINGLE_DATA_AMOUNT)
        {
            if (CalibrationPara.DataNum >= (SINGLE_DATA_AMOUNT * 3 / 5))
            {
                Adaptive_CalibrationPolyFit(); // 线性回归拟合
                adaptive_calibration_result_collection();
            }
            else
            {
                realtime_calibration_clean(CLEAR_ANGLE); // 标定放弃
            }
        }
    }else
    {
        printf("**************************************************\n");
        for (int row = 0; row < ROW; row++) {
            for (int col = 0; col < COL; col++) {
                printf("%f  ", CalibrationPara.realtime_cal_res[row * COL + col]);
            }
            printf("\n");
        }
        printf("**************************************************\n");
        for (int i = 0; i < ROW; i++)
        {
            printf("CalibrationPara.real_cal_res_median[%d] = %f\n",i,CalibrationPara.real_cal_res_median[i]);
            /* code */
        }
        printf("**************************************************\n");
        CAL_MODE = CALIBRATION_EXIT;
        //已经标定了10次，进行后续处理
    }

   //printf("111111CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num = %d\n",CalibrationPara.cal_row_num*COL + CalibrationPara.cal_col_num);
}