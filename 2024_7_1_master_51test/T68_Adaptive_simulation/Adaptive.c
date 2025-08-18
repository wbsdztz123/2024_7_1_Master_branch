
/****************************************************************************
 *                        File: adaptive_calibration.c                      *
 *                        @sjw20210713                                           *
 ****************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "Adaptive.h"

CalibrationParaS CalibrationPara = {0};
Message_VehicleMsgS Message_VehicleMsg = {0};
RadarParaS RadarPara = {0};
extern CALIBRATION_MODE CAL_MODE;
/* 函数名: AdaptiveCalStart()
 * 描述：售后校准开始
 * 返回值：0x00=标定例程成功开启
 *        0x01=标定正在运行中
 *        0x02=标定失败
 */
uint8_t AdaptiveCalStart(void)
{
    #define CALIBRATION_ROUTINE_STARTS 0x00
    #define CALIBRATION_ROUTINE_RUNNING 0x01
    #define CALIBRATION_ROUTINE_FAIL 0x02

    uint8_t result = CALIBRATION_ROUTINE_FAIL;
    uint16_t i;
  //  const radar_work_mode_t radar_data = WORKIN_MODE_ADAPTIVE_CALIBRATION;
    //CalibrationPara.adaptive_Workmode = ADAPTIVE_MODE;
    CalibrationPara.Start = 0;
    CalibrationPara.Step = 1;
  //  CalibrationPara.Master_Result = UNCALIBRATED;
    //CalibrationPara.Error_Number = 0;
    //CalibrationPara.Effective_point = 0;
    CalibrationPara.SteeringAngle = 0;
    CalibrationPara.YawRate = 0;
    CalibrationPara.Velocity = 0;
    CalibrationPara.Counter = 0;
    CalibrationPara.Frame = 0;
    CalibrationPara.FalseFrame = 0;
    CalibrationPara.DataNum = 0;
    CalibrationPara.AveYdata = 0;
    CalibrationPara.Adap_A = 0;
    CalibrationPara.Adap_B = 0;
    CalibrationPara.errType =0;
    CalibrationPara.driving_profile = 0;//驾驶指导初始化
    
    CalibrationPara.adaptive_PB = 0; //实时进度
    CalibrationPara.TEMP_PB = 0;  //固定进度
    //CalibrationPara.Adaptive_step = 0; // zjn test
    // if(_atomic_load(RadarPara.WorkMode, radar_work_mode_t) == WORKIN_MODE_NONTRI_CALIBRATION)
    // {
    //    // 
    // }else{
    //     AdaptiveCalClockLaunch(ADAPTIVE_CAL_TIMEOUT_CYCLE);
    // }
    for (i = 0; i < SINGLE_DATA_AMOUNT; i++)
    {
        CalibrationPara.xdata[i] = 0;
        CalibrationPara.ydata[i] = 0;
        CalibrationPara.rangdata[i] = 0;
        CalibrationPara.elevdata[i] = 0;
    }
    CalibrationPara.Adap_Angle = 0;
    CalibrationPara.Adap_eleAngle = 0;

    // if (!switching_mode_debug(WORKIN_MODE_ADAPTIVE_CALIBRATION)) {
    //     result = CALIBRATION_ROUTINE_STARTS;
    // }

    return result;
}

/* 函数名：AdaptiveCalTimeoutProc()
 * 描述：售后校准超时回调函数
 * 输入：clock: clock object
 *       argv: 传入参数
 * 返回值：NA
 */
// static void AdaptiveCalTimeoutProc(ClockP_Object *clock, void *argv)
// {
//     //_3sradar_notify("AdaptiveCalTimeoutProc++\r\n");
//     uint8_t StatusArray[6] = {0};
//     CalibrationPara.errType = CALIBRATION_ADAPTIVE_IN_TIMEOUT;
//     CalibrationPara.Master_Result = CALIBRATION_IS_FAILED;
    
//      StatusArray[0] = CALIBRATION_NOT_COMPLETED;
//      StatusArray[1] = CALIBRATION_ADAPTIVE_IN_TIMEOUT;
//      StatusArray[2] = (((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) >> 8) & 0xFF;
//      StatusArray[3] = ((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) & 0xFF;
//      StatusArray[4] = (((int16_t)(RadarPara.FarVerticalOffsetAngle * 100)) >> 8) & 0xFF;
//      StatusArray[5] =((int16_t)(RadarPara.FarVerticalOffsetAngle * 100)) & 0xFF;
//      Adaptive_Calibration_Exit(StatusArray);
//      //SetDtcMissCal_0x9ED554(TRUE);
     
// }


/* 函数名：AdaptiveCalClockLaunch()
 * 描述：启动售后校准超时clock
 * 输入：period: 超时时长(usec)-
 * 返回值：NA
 * 备注：停止并销毁clock示例代码如下，
 *      ClockP_stop(adaptive_cal_clock);
 *      ClockP_destruct(adaptive_cal_clock);
 */
// void AdaptiveCalClockLaunch(uint64_t period)
// {
//     ClockP_Params clock_params;

//     ClockP_Params_init(&clock_params);
//     clock_params.timeout = ClockP_usecToTicks(period);
//     clock_params.start = 1;
//     clock_params.callback = &AdaptiveCalTimeoutProc;
//     clock_params.args = NULL;

//     ClockP_construct(&adaptive_cal_clock, &clock_params);
// }
/* 函数名: GetAdaptiveCalStatus()
 * 描述：获取售后校准状态
 * 返回值：StatusArray
 *        byte 0: 标定结果 (0x00=标定成功, 0x01=标定进行中, 0x02=标定失败, 0x03=用户终止, 0x04~0xFE=预留, 0xFF=标定未开始)
 *        byte 1: 标定进度 (范围 0~100)
 *        byte 2: 标定失败原因 (0x00=标定成功, 0x01=标定超时, 0x02=角度偏差过大, 0x03=NVM写入错误, 
 *                             0x04=用户终止, 0x05=SDA 校准失败, 0x06~0xFE=预留, 0xFF=标定未开始)
 *        byte 3~4: 水平角度误差值 (Resolution=0.01, Offset=0) 
 *                  Data[3] = (HorizontalAngle >> 8) & 0xFF;
 *                  Data[4] = HorizontalAngle & 0xFF;
 *        byte 5~6: 垂直角度误差值 (Resolution=0.01, Offset=0)
 *                  Data[5] = (VerticalAngle >> 8) & 0xFF;
 *                  Data[6] = VerticalAngle & 0xFF;
 *        byte 7: 保留
 */
// void GetAdaptiveCalStatus(uint8_t * StatusArray)
// {
//     int16_t TempFarHorizontalAdptiveAngle, TempFarVerticalAdptiveAngle;
    
//     if (CalibrationPara.Master_Result == CALIBRATION_IS_SUCCESS || CalibrationPara.errType == CALIBRATION_ADAPTIVE_IS_ANGLE_OVERSHOOT) {
//         TempFarHorizontalAdptiveAngle = (int16_t)((RadarPara.FarHorizontalAdptiveAngle + RadarPara.FarHorizontalOffsetAngle) * 100);
//         TempFarVerticalAdptiveAngle = (int16_t)((RadarPara.FarVerticalAdptiveAngle + RadarPara.FarVerticalOffsetAngle) * 100);
//     } else {
//         TempFarHorizontalAdptiveAngle = 0;
//         TempFarVerticalAdptiveAngle   = 0;
//     }



//     if(!CalibrationPara.Master_Result){
//        *StatusArray  = CALIBRATION_ADAPTIVE_IN_PROGRESS;
      
//     }else if(CalibrationPara.Master_Result == CALIBRATION_IS_SUCCESS){
//        *StatusArray  = CALIBRATION_ADAPTIVE_IS_SUCCESSFUL; 
//     }else if(CalibrationPara.Master_Result == CALIBRATION_IS_FAILED)
//     {
//         *StatusArray  = CALIBRATION_ADAPTIVE_IS_FAIL;
//     }
    
//     *(StatusArray + 1) = CalibrationPara.adaptive_PB; 
//     *(StatusArray + 2) = CalibrationPara.errType;

//     *(StatusArray + 3) = (uint8_t)((TempFarHorizontalAdptiveAngle >> 8) & 0xFF);
//     *(StatusArray + 4) = (uint8_t)(TempFarHorizontalAdptiveAngle & 0xFF);
//     *(StatusArray + 5) = (uint8_t)((TempFarVerticalAdptiveAngle >> 8) & 0xFF);
//     *(StatusArray + 6) = (uint8_t)(TempFarVerticalAdptiveAngle & 0xFF);
//     *(StatusArray + 7) = 0x00;

// }


void Adaptive_CalibrationClear(void)
{
        uint16_t i;
        CalibrationPara.Start = 0;
        CalibrationPara.SteeringAngle = 0;
        CalibrationPara.YawRate = 0;
        CalibrationPara.Velocity = 0;
        CalibrationPara.Frame = 0;
        CalibrationPara.FalseFrame = 0;
        CalibrationPara.DataNum = 0;
        CalibrationPara.AveYdata = 0;
        CalibrationPara.Adap_A = 0;
        CalibrationPara.Adap_B = 0;

        CalibrationPara.TEMP_PB = 0;  //标定进度清零

        for (i = 0; i < SINGLE_DATA_AMOUNT; i++)
        {
            CalibrationPara.xdata[i] = 0;
            CalibrationPara.ydata[i] = 0;
            CalibrationPara.rangdata[i] = 0;
            CalibrationPara.elevdata[i]  = 0;
        }

        // if(CalibrationPara.adaptive_Workmode == CHEACK_MODE)
        // {
        //     CalibrationPara.Counter = 0;
        // }
        CalibrationPara.Adap_Angle = 0;
        CalibrationPara.Adap_eleAngle = 0;
        CalibrationPara.errType = 0;
        CalibrationPara.driving_profile = 0;
}


void Adaptive_Calibration(const or_point_cloud_format_t *PeakList)
{
    uint8_t tempProgress;
    
    Adaptive_CalibrationSaveData(PeakList); // 保存数据

    if (CalibrationPara.FalseFrame > (ADAPTIVE_FRAME_NUM / 10))
    {
        //CalibrationPara.Adaptive_step = 7; //zjn test

        Adaptive_CalibrationClear(); // 标定放弃
        CalibrationPara.driving_profile = 0x10; // 目标不充分
    }
    else if (CalibrationPara.Frame >= ADAPTIVE_FRAME_NUM || CalibrationPara.DataNum >= SINGLE_DATA_AMOUNT)
    {
        //CalibrationPara.Adaptive_step = 8; //zjn test
       // tempProgress = CalibrationPara.Counter * PROGRESS_STEP + 10;
        //CalibrationPara.Adaptive_Check_step = 3;
        Calibration_Progress(tempProgress);

        if (CalibrationPara.DataNum >= (SINGLE_DATA_AMOUNT * 3 / 5))
        {
            //CalibrationPara.Adaptive_step = 9; //zjn test

            Adaptive_CalibrationPolyFit(); // 线性回归拟合
            Adaptive_CalibrationFinish();
        }
        else
        {
            //CalibrationPara.Adaptive_step = 10; //zjn test

            Adaptive_CalibrationClear(); // 标定放弃
            CalibrationPara.driving_profile = 0x10; // 目标不充分
        }
    }
}


// 数据保存
void Adaptive_CalibrationSaveData(const or_point_cloud_format_t *PeakList)
{
    uint8_t  flag;
    uint8_t  Calibration_flag;
    uint32_t i;
    uint32_t  Start_num = 0;
    //float32_t tmpCalibrationRange;
    float32_t temp_speed_gap = 0xff;
    float32_t temp_Ydata;
    uint8_t   rang_flag = 0;
    uint8_t   tempProgress;
    flag         = Body_Posture_Detection(); //车身姿态检测
    // tempProgress = CalibrationPara.Counter * PROGRESS_STEP;
    // tempProgress += 2;
    // Calibration_Progress(tempProgress);


#ifdef _CALIBRATION_DEBUG_

    int32_t tmp_data;
    uint8_t data[8] = {0};

    tmp_data = (int32_t)((Message_VehicleMsg.CurveRadius) * 10);
    data[0]  = (uint8_t)((tmp_data >> 16) & 0xFF);
    data[1]  = (uint8_t)((tmp_data >> 8) & 0xFF);
    data[2]  = (uint8_t)(tmp_data & 0xFF);

    tmp_data = (int32_t)((Message_VehicleMsg.SteeringAngle) * 10);
    data[3]  = (uint8_t)((tmp_data >> 16) & 0xFF);
    data[4]  = (uint8_t)((tmp_data >> 8) & 0xFF);
    data[5]  = (uint8_t)(tmp_data & 0xFF);
    data[6]  = flag;
    Private_Can_CalibrtionDebug(0x710, data, 7);

#endif

    if (flag) // 车辆处于可以标定的状态，进行标定
    {
        CalibrationPara.Error_Number = 0; // 车速、档位、转弯正确，则时间清零。
        //CalibrationPara.Adaptive_Check_step = 1;

        if (CalibrationPara.Start == 0) // 没有开始标定
        {
            CalibrationPara.AveYdata = 0;
            rang_flag                = Rang_judge(PeakList); // 判断栅栏距离

             float lower_bound, upper_bound;
            switch (rang_flag) {
            case 2:
                lower_bound = 4.01f;
                upper_bound = 8.0f;
                break;
            case 1:
                lower_bound = 0.0f;
                upper_bound = 4.01f;
                break;
            default:
                lower_bound = 0.0f;
                upper_bound = 15.0f;
                Adaptive_CalibrationClear(); // 标定放弃
                break;
            }

            for (i = 0; i < PeakList->point_count; i++) // 读取车辆旁边障碍物的位置，存储数据
            {
                Calibration_flag = CAL_Target_Filtering(PeakList, i);

                if (Calibration_flag && rang_flag) {

                    float azimuth = PeakList->term[i].azimuth * 180 / PI;
                    temp_Ydata    = PeakList->term[i].range * sin((0 + azimuth) * PI / 180);

                    if (temp_Ydata > lower_bound && temp_Ydata < upper_bound) 
                    {
                        Start_num++;
                        CalibrationPara.AveYdata += temp_Ydata;
                    }
                }
            }


            // tempProgress = CalibrationPara.Counter * PROGRESS_STEP;
            // tempProgress += 5;
            // Calibration_Progress(tempProgress);

            if (Start_num >= CALIBRATION_MIN_SAMPLES) {
                CalibrationPara.AveYdata = CalibrationPara.AveYdata / Start_num;
                if ((CalibrationPara.AveYdata > 0.5f) && (CalibrationPara.AveYdata < 7.0f)) //3.0-4.0-4.5-5.0-6.0-8.0
                {
                    CalibrationPara.Start = 1;
                    //CalibrationPara.Adaptive_Check_step = 2;
                    CalibrationPara.SteeringAngle = Message_VehicleMsg.SteeringAngle; // 存储标定开始的方向盘转角
                    CalibrationPara.Velocity = KMH_TO_MS(Message_VehicleMsg.Velocity); // 存储标定开始的车速


#ifdef _CALIBRATION_DEBUG_

                    tmp_data = (int32_t)((CalibrationPara.SteeringAngle) * 10);
                    data[0]  = (uint8_t)((tmp_data >> 16) & 0xFF);
                    data[1]  = (uint8_t)((tmp_data >> 8) & 0xFF);
                    data[2]  = (uint8_t)(tmp_data & 0xFF);

                    tmp_data = (int32_t)((CalibrationPara.AveYdata) * 10);
                    data[3]  = (uint8_t)((tmp_data >> 16) & 0xFF);
                    data[4]  = (uint8_t)((tmp_data >> 8) & 0xFF);
                    data[5]  = (uint8_t)(tmp_data & 0xFF);
                    data[6]  = 0xFF;
                    Private_Can_CalibrtionDebug(0x711, data, 7);

#endif
                    // tempProgress = CalibrationPara.Counter * PROGRESS_STEP;
                    // tempProgress += 8;
                    // Calibration_Progress(tempProgress);

                    //CalibrationPara.Adaptive_step = 1; //zjn test
                } else {
                    //CalibrationPara.Adaptive_step = 2; //zjn test
                    //CalibrationPara.Start = 2;      //单帧点符合但是栅栏横向距离不符合
                    Adaptive_CalibrationClear(); // 标定放弃
                    //CalibrationPara.errType = 0x06;
                    CalibrationPara.driving_profile = 0x10; //目标不充分
#ifdef _CALIBRATION_DEBUG_

                    data[0] = 0xFF;
                    data[1] = 0xFF;
                    data[2] = 0xFF;
                    data[3] = 0xFF;
                    data[4] = 0xFF;
                    data[5] = 0xFF;
                    data[6] = 0xFF;
                    Private_Can_CalibrtionDebug(0x712, data, 7);

#endif
                }
            } else {
                //CalibrationPara.Adaptive_step = 3; //zjn test
                //CalibrationPara.Start = 2;     //单帧点不符合
                Adaptive_CalibrationClear(); // 标定放弃
                //CalibrationPara.errType = 0x06;
                CalibrationPara.driving_profile = 0x10; //目标不充分
#ifdef _CALIBRATION_DEBUG_

                data[0] = 0xFF;
                data[1] = 0xFF;
                data[2] = 0xFF;
                data[3] = 0xFF;
                data[4] = 0xFF;
                data[5] = 0xFF;
                data[6] = 0xFF;
                Private_Can_CalibrtionDebug(0x713, data, 7);
                /*******debug*****/
#endif
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
                        float X = PeakList->term[i].range * cos((0 + azimuth_deg) * PI / 180);
                        float Y = PeakList->term[i].range * sin((0 + azimuth_deg) * PI / 180);
                        float ele = PeakList->term[i].elevation * (180 / PI);
                        float Radial_distance = PeakList->term[i].range;
                        if (CalibrationPara.AveYdata < 2.2f)
                        {
                            if (CalibrationPara.DataNum < SINGLE_DATA_AMOUNT
                                && Y > (CalibrationPara.AveYdata - (Calibration_Ydata_gap - 0.2))
                                && Y < (CalibrationPara.AveYdata + (Calibration_Ydata_gap - 0.2)) && X > 8.0f) {
                                CalibrationPara.xdata[CalibrationPara.DataNum] = X;
                                CalibrationPara.ydata[CalibrationPara.DataNum] = Y;
                                CalibrationPara.elevdata[CalibrationPara.DataNum] = ele;
                                CalibrationPara.rangdata[CalibrationPara.DataNum] = Radial_distance;
                                CalibrationPara.DataNum++;
                                Start_num++;
                            }
                        }
                        else
                        {
                            if (CalibrationPara.DataNum < SINGLE_DATA_AMOUNT
                                && Y > (CalibrationPara.AveYdata - Calibration_Ydata_gap)
                                && Y < (CalibrationPara.AveYdata + Calibration_Ydata_gap) && X > 8.0f) 
                            {
                                CalibrationPara.xdata[CalibrationPara.DataNum] = X;
                                CalibrationPara.ydata[CalibrationPara.DataNum] = Y;
                                CalibrationPara.elevdata[CalibrationPara.DataNum]  = ele;
                                CalibrationPara.rangdata[CalibrationPara.DataNum] = Radial_distance;
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
                        //CalibrationPara.Adaptive_step = 5; //zjn test
                        Adaptive_CalibrationClear();       // 标定放弃
                        //CalibrationPara.errType = 0x06;
                        CalibrationPara.driving_profile = 0x10; //目标不充分
                    }
#ifdef _CALIBRATION_DEBUG_

                    data[0] = 0xFF;
                    data[1] = 0xFF;
                    data[2] = 0xFF;
                    data[3] = 0xFF;
                    data[4] = 0xFF;
                    data[5] = 0xFF;
                    data[6] = 0xFF;
                    Private_Can_CalibrtionDebug(0x715, data, 7);

#endif
                } else {
                    CalibrationPara.FalseFrame = 0;
#ifdef _CALIBRATION_DEBUG_

                    data[0] = 0xFF;
                    data[1] = 0xFF;
                    data[2] = 0xFF;
                    data[3] = 0xFF;
                    data[4] = 0xFF;
                    data[5] = 0xFF;
                    data[6] = 0xFF;
                    Private_Can_CalibrtionDebug(0x716, data, 7);

#endif
                }
            } else {
                Adaptive_CalibrationClear(); // 标定放弃

                //CalibrationPara.Adaptive_step = 6;                                                  //zjn test
                if (fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle) > 10.0f) //横摆角速度过大
                {
                    CalibrationPara.driving_profile = 0x04;
                } else if (fabs((KMH_TO_MS(Message_VehicleMsg.Velocity)) - CalibrationPara.Velocity) > 3.0f) //纵向加速度过大
                {
                    CalibrationPara.driving_profile = 0x08;
                }

                //CalibrationPara.errType = 0x06;
#ifdef _CALIBRATION_DEBUG_

                data[0] = 0xFF;
                data[1] = 0xFF;
                data[2] = 0xFF;
                data[3] = 0xFF;
                data[4] = 0xFF;
                data[5] = 0xFF;
                data[6] = 0xFF;
                Private_Can_CalibrtionDebug(0x713, data, 7);
#endif
            }
        }
    } else {
        CalibrationPara.Error_Number++; // 车速、档位、转弯半径不对的时候，不进行标定
                  Adaptive_CalibrationClear();    // 标定放弃
                                        // if (CalibrationPara.Error_Number > 500) // 30s时间
                                        // {
                                        //     if((Message_VehicleMsg.Velocity/3.6f) < Calibration_MinVelocity) //车速过低
                                        //     {
                                        //         CalibrationPara.driving_profile = 0x01;
                                        //     }
                                        //     else if((Message_VehicleMsg.Velocity/3.6f) > Calibration_MaxVelocity)//车速过高
                                        //     {
                                        //         CalibrationPara.driving_profile = 0x02;
                                        //     }
                                        //     else if(fabs(Message_VehicleMsg.SteeringAngle) > Calibration_MaxSteeringAngle)//横摆角速度过大
                                        //     {
                                        //         CalibrationPara.driving_profile = 0x04;
                                        //     }
                                        //     CalibrationPara.TEMP_PB = 0;   //失败进度清0
                                        //     //CalibrationPara.errType = 0x06;  //车辆信号不满足
                                        //}
#ifdef _CALIBRATION_DEBUG_

        data[0] = 0xFF;
        data[1] = 0xFF;
        data[2] = 0xFF;
        data[3] = 0xFF;
        data[4] = 0xFF;
        data[5] = 0xFF;
        data[6] = 0xFF;
        Private_Can_CalibrtionDebug(0x718, data, 7);

#endif
    }
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
//水平
    for (i = 0; i < CalibrationPara.DataNum; i++) 
    {
        float32_t x = CalibrationPara.xdata[i];
        float32_t y = CalibrationPara.ydata[i];

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
//垂直
    float32_t sum_ele =0;
    int16_t count = 0;
    for (int j = 0; j < CalibrationPara.DataNum; j++) {
        if (CalibrationPara.rangdata[j] >= 50 && CalibrationPara.rangdata[j] <= 60) {
            sum_ele += CalibrationPara.elevdata[j];
            count++;
        }
    }
    if (count != 0)
    {
        float32_t averagePitch        = sum_ele / count;
        CalibrationPara.Adap_eleAngle = averagePitch;
    }


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


/*
void Adaptive_CalibrationFinish(void)
{
    #define ADAPTIVE_PROGRESS_FINISH  100
    uint8_t tempProgress;
    /*debug*/
    int8_t   errcode = 0;
    uint8_t StatusArray[9] = {0};
    /*debug*/
    float32_t TmpLinearAngle = 0;
    float32_t TmpLineareleAngle = 0;
    if ((CalibrationPara.Adap_B > 0.5f) && (CalibrationPara.Adap_B < 7.0f)) //(b > 1.3f)  &&  (b < 2.8f))  3.5-4.0-4.5-5.0-6.0
    {
        if (CalibrationPara.Step == 1)
        {
            TmpLinearAngle = CalibrationPara.Adap_Angle;
            TmpLineareleAngle = CalibrationPara.Adap_eleAngle;

            tempProgress = CalibrationPara.Counter * PROGRESS_STEP;
            tempProgress += PROGRESS_STEP;
            Calibration_Progress(tempProgress);
        }

        // if (fabs(TmpLinearAngle) < 7)
        // {
            CalibrationPara.Temp_A[CalibrationPara.Counter] = TmpLinearAngle;
            CalibrationPara.Temp_ele[CalibrationPara.Counter] = TmpLineareleAngle;
            CalibrationPara.Counter++;
             if (CalibrationPara.Counter > ADAPTIVE_COUNTER)
            {
                for(int i = 0; i < (CalibrationPara.Counter ) ; i++)
                {
                    for(int j = 0;j < (CalibrationPara.Counter - i - 1); j++)
                    {
                        if(CalibrationPara.Temp_A[j] > CalibrationPara.Temp_A[j + 1])
                        {
                            TmpLinearAngle = CalibrationPara.Temp_A[j];
                            CalibrationPara.Temp_A[j] = CalibrationPara.Temp_A[j + 1];
                            CalibrationPara.Temp_A[j + 1] = TmpLinearAngle;
                        }
                    }
                }
                for(int i = 0; i < (CalibrationPara.Counter ) ; i++)
                {
                    for(int j = 0;j < (CalibrationPara.Counter - i - 1); j++)
                    {
                        if(CalibrationPara.Temp_ele[j] > CalibrationPara.Temp_ele[j + 1])
                        {
                            TmpLineareleAngle = CalibrationPara.Temp_ele[j];
                            CalibrationPara.Temp_ele[j] = CalibrationPara.Temp_ele[j + 1];
                            CalibrationPara.Temp_ele[j + 1] = TmpLineareleAngle;
                        }
                    }
                }
    // if(CHEACK_MODE == CalibrationPara.adaptive_Workmode)
    // {
    //     TmpLinearAngle = -CalibrationPara.Temp_A[3];
    //     TmpLineareleAngle = RadarPara.RadarSelfDeviation + 0.5f - CalibrationPara.Temp_ele[3];
    //     CalibrationPara.Adaptive_step = 0x02;    //检验完成
    //     CalibrationPara.Adaptive_Check_Angle = TmpLinearAngle;
    //     CalibrationPara.Adaptive_Check_eleAngle = TmpLineareleAngle;

    // }else if(ADAPTIVE_MODE == CalibrationPara.adaptive_Workmode)
    // {
                    if (CalibrationPara.Step == 1)
                    {
                    TmpLineareleAngle = -CalibrationPara.Temp_A[3];
                        //TmpLineareleAngle = RadarPara.RadarSelfDeviation + 0.5f  - CalibrationPara.Temp_ele[3];  先不加偏移
                        
                        if (RadarPara.InstallPosition == INSTALL_LEFT_BACK) {
                            TmpLinearAngle += 0.0f;//0.80f;
                        }else if(RadarPara.InstallPosition == INSTALL_RIGHT_BACK)
                        {
                            TmpLinearAngle += 0.0f;//1.52f;
                        }
                        RadarPara.FarHorizontalAdptiveAngle = TmpLinearAngle - RadarPara.FarHorizontalOffsetAngle;
                        //RadarPara.FarVerticalAdptiveAngle = RadarPara.FarVerticalOffsetAngle;

                        if ((TmpLineareleAngle < (Calibration_elevTolerance_authentic - EPSILON))
                            && TmpLineareleAngle > (CALIBRATION_ELEVTOLERANCE + EPSILON)) {
                            TmpLineareleAngle = 2.6f + 0.3f * rand() / RAND_MAX * 1.0f; //随机数[2.6,2.9]
                        } else if (TmpLineareleAngle > (-(Calibration_elevTolerance_authentic) + EPSILON)
                                   && TmpLineareleAngle < (-(CALIBRATION_ELEVTOLERANCE)-EPSILON)) {
                                TmpLineareleAngle = -2.9f + 0.3f * rand() / RAND_MAX * 1.0f; //随机数[-2.9,-2.6]
                        } else {
                           //do nothing
                        }
                        RadarPara.FarVerticalAdptiveAngle    = TmpLineareleAngle - RadarPara.FarVerticalOffsetAngle;
                        RadarPara.TempHorizontalAdptiveAngle = RadarPara.FarHorizontalAdptiveAngle;
                        RadarPara.TempVerticalAdptiveAngle   = RadarPara.FarVerticalAdptiveAngle;
                    }

                    if (CalibrationPara.Step == 1)
                    {
                        tempProgress = ADAPTIVE_PROGRESS_FINISH;
                        Calibration_Progress(tempProgress);
                        CalibrationPara.Step = 3;          // 标定完成
                        
                        CalibrationPara.driving_profile = 0x00;
                        StatusArray[0] = 0x00;
                        StatusArray[1] = 0x00;
                        StatusArray[2] = (((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) >> 8) & 0xFF;
                        StatusArray[3] = ((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) & 0xFF;
                        StatusArray[4] = (((int16_t)(RadarPara.FarVerticalAdptiveAngle * 100)) >> 8) & 0xFF;
                        StatusArray[5] =((int16_t)(RadarPara.FarVerticalAdptiveAngle * 100)) & 0xFF;

                        //Adaptive_Calibration_Exit(StatusArray);
                        
                        if((fabs(TmpLinearAngle) >= (CALIBRATION_TOLERANCE + EPSILON ))||(fabs(TmpLineareleAngle) >= (CALIBRATION_ELEVTOLERANCE + EPSILON)))
                        {
                            StatusArray[0] = 0x00;
                            StatusArray[1] = 0x02;
                            
                            //SetDtcCalOutOfRange_0x9ED546(TRUE);
                            //SetDtcMissCal_0x9ED554(TRUE);
                            CalibrationPara.Master_Result = CALIBRATION_IS_FAILED; // 雷达标定结果为失败
                            CalibrationPara.errType = 0x02;
                        }else{
                            StatusArray[0] = CALIBRATION_COMPLETED;
                            CalibrationPara.Master_Result = CALIBRATION_IS_SUCCESS; // 雷达标定结果为成功
                            //SetDtcCalOutOfRange_0x9ED546(FALSE);
                            //SetDtcCalOutOfRange_0x9ED955(FALSE);
                            //SetDtcMissCal_0x9ED554(FALSE);
                        }
                        Adaptive_Calibration_Exit(StatusArray);
                        Config_WriteAllConfig();
    //}



                }
            } else {
                Adaptive_CalibrationClear();
            }

            // }
            // else
            // {
            //     Adaptive_CalibrationClear(); // 标定放弃
            //     //CalibrationPara.errType = 0x02;//标定角度偏差过大
            // }
    }
    else
    {
        Adaptive_CalibrationClear();
    }
}
*/


uint8_t Rang_judge(const or_point_cloud_format_t *PeakList)
{
    #define TARGET_MIN_RANGE 5.0f
    #define TARGET_MAX_RANGE 25.0f
    uint32_t i = 0;
    uint8_t result = 0;
    uint16_t rang0_4 = 0, rang4_8 = 0, rang8_ = 0;
    float32_t azimuth_deg = 0.0f;

    for (i = 0; i < PeakList->point_count; i++)
    {
        azimuth_deg = RAD_TO_DEG(PeakList->term[i].azimuth);

        result = CAL_Target_Filtering(PeakList, i); 
        //float temp_X = PeakList->term[i].range * cos((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
        if((PeakList->term[i].range > TARGET_MIN_RANGE) && (PeakList->term[i].range < TARGET_MAX_RANGE))
        {
            if ((result))
            {
                float32_t temp_Ydata = PeakList->term[i].range * sin(DEG_TO_RAD((0 + azimuth_deg)));
                if (temp_Ydata > 0 && temp_Ydata < 4.01f) {
                    rang0_4++;
                } else if (temp_Ydata > 4.01f && temp_Ydata < 8.01f) {
                    rang4_8++;
                } 
            }
        }
    }
        if (rang0_4 > rang4_8)
        {
            return 1;
        }
        else
        {
            return 2;
        }
    return 0;
}



void Adaptive_Calibration_Exit(uint8_t* StatusArray)  //退出标定
{
    //uint8_t last_cal_st = 0; // 0 is fail; 1 is sucess.
    ClockP_stop(&adaptive_cal_clock);
#if 0
    ClockP_destruct(&adaptive_cal_clock);
#endif
    radar_work_mode_t radar_data = WORKIN_TRACK_MODE;
    SaveAdaptiveCalStatus_DID_0x4902(StatusArray);

    // if (StatusArray[0] == 0x00)
    //     last_cal_st = 0x1;  // Calibration is OK
    // else 
    //     last_cal_st = 0x0;  // Calibration is fail
    SaveLastCalStatus(CalibrationPara.Master_Result);

    RadarPara.FarHorizontalAdptiveAngle = RadarPara.TempHorizontalAdptiveAngle;
    RadarPara.FarVerticalAdptiveAngle = RadarPara.TempVerticalAdptiveAngle;
    _atomic_store(RadarPara.WorkMode, radar_data);
}

uint8_t Body_Posture_Detection(void)
{
    uint8_t result = 0;
    float velocity = KMH_TO_MS(Message_VehicleMsg.Velocity);
    float steeringAngle = fabs(Message_VehicleMsg.SteeringAngle);
    float curveRadius = fabs(Message_VehicleMsg.CurveRadius);
    float yawRate = fabs(Message_VehicleMsg.YawRate);

    result = (velocity > Calibration_MinVelocity ) && (velocity < Calibration_MaxVelocity) 
             && (steeringAngle < Calibration_MaxSteeringAngle) 
             && (curveRadius > Calibration_MaxRoadCurve) 
             && (yawRate < Calibration_MaxYawRate);

    return result;
}



void Calibration_Progress(uint8_t pace)
{
    CalibrationPara.TEMP_PB = max(pace, CalibrationPara.TEMP_PB);
    CalibrationPara.adaptive_PB = max(CalibrationPara.TEMP_PB, CalibrationPara.adaptive_PB);
}


// int32_t Private_Can_CalibrationDebug(uint32_t ID, uint8_t* Data, uint8_t length)
// {
//     uint32_t tx_id = ID;
//     MCAN_TxBufElement tx_msg;
//     int8_t errcode = 0;
    
//     MCAN_initTxBufElement(&tx_msg);
    
//     if (length < 9) {
//         tx_msg.dlc = MCAN_DATA_SIZE_8BYTES;
//     } else if (length < 17) {
//         tx_msg.dlc = MCAN_DATA_SIZE_16BYTES;
//     } else if (length < 25) {
//         tx_msg.dlc = MCAN_DATA_SIZE_24BYTES;
//     } else if (length < 33) {
//         tx_msg.dlc = MCAN_DATA_SIZE_32BYTES;
//     } else if (length < 49) {
//         tx_msg.dlc = MCAN_DATA_SIZE_48BYTES;
//     } else if (length < 65) {
//         tx_msg.dlc = MCAN_DATA_SIZE_64BYTES;
//     } else {
//         return errcode;
//     }
    
//     tx_msg.fdf = TRUE;
//     tx_msg.xtd = FALSE;
    
//     for (uint8_t i = 0; i < length; i++) {
//         tx_msg.data[i] = Data[i];
//     }
    
//     tx_msg.id = (((tx_id) & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
    
//     errcode = send_message_via_private_can(&tx_msg);
    
//     return errcode;
// }


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
        //bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_LEFT_FRONT || RadarPara.InstallPosition == INSTALL_RIGHT_FRONT);
        bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_FRONT);
        bool isDopplerNegative = PeakList->term[i].doppler < 0.0f;

        if ((isLeftOrRightFront && isDopplerNegative) || (!isLeftOrRightFront && !isDopplerNegative))
        {
            float32_t cosValue = cos((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
            float32_t speed = KMH_TO_MS(Message_VehicleMsg.Velocity);
            float32_t threshold = 0.15f;

            if (speed >= 4.1f && speed < 8.3f)
            {
                threshold = 0.15f;//0.12//0.13
            }
            else if (speed >= 8.3f)//0.1//.12
            {
                threshold = 0.15f;
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

// void Pitch_angle_deviation_detection(const or_point_cloud_format_t *PeakList)
// {
//     uint32_t i;
//     if (PeakList->point_count >= GTRACK_NUM_POINTS_MAX) {
//         return;
//     }
//     if (CalibrationPara.pitch_angle_num >= 1000) {
//         {
//             if ((CalibrationPara.pitch_angle_cnt / CalibrationPara.pitch_angle_num) > 0.8f) {
//                 CalibrationPara.No_pitch_deviation_cnt++;
//             }else{
//                 CalibrationPara.Large_pitch_deviation_cnt++;
//             }
//             CalibrationPara.pitch_angle_cnt = 0;
//             CalibrationPara.pitch_angle_num = 0;
//         }
//     }

//     if (CalibrationPara.No_pitch_deviation_cnt) {
//         CalibrationPara.pitch_angle_flag = 0;
//         CalibrationPara.No_pitch_deviation_cnt = 0;
//         CalibrationPara.Large_pitch_deviation_cnt = 0;
//     } else if (CalibrationPara.No_pitch_deviation_cnt > 9) {
//         CalibrationPara.pitch_angle_flag = 1;
//         CalibrationPara.No_pitch_deviation_cnt    = 0;
//         CalibrationPara.Large_pitch_deviation_cnt = 0;
//     }else{
        
//     }

//     if (((fabs(gModelParamK) > EPSILON) && (Message_VehicleMsg.Velocity > 20.0f) && (Message_VehicleMsg.Velocity < 50.0f)) 
//           || (Message_VehicleMsg.Velocity > 80.0f))
//     {
//         CalibrationPara.pitch_angle_num++;
//         for (i = 0; i < PeakList->point_count;i++) 
//         {
//             if (PeakList->term[i].range >= 100.0f) 
//             {
//                 CalibrationPara.pitch_angle_cnt++;
//                 return;
//             }
//         }
//     }
// }


// bool Angle_Difference_judg(void)
// {
//     bool      result       = TRUE;
//     float32_t tmp_angle    = GetCalHorizontalAngle();
//     float32_t tmp_eleangle = GetCalVertiAngle();
//     if (fabs(CalibrationPara.Adaptive_Check_Angle - tmp_angle) > 3.0f) {
//         CalibrationPara.CheckH_count++;
//     }

//     if (fabs(CalibrationPara.Adaptive_Check_eleAngle - tmp_eleangle) > 5.0f) {
//         CalibrationPara.CheckV_count++;
//     }

//     if (3 < (CalibrationPara.CheckV_count + CalibrationPara.CheckH_count)) 
//     {
//         CalibrationPara.Excessive_horizontalAngle_flag = 0x01; //角度偏差过大
//         return FALSE;
//     }
//     else {
//         CalibrationPara.CheckH_count                    = 0;
//         CalibrationPara.CheckV_count                    = 0;
//         CalibrationPara.Excessive_horizontalAngle_flag = 0x00; //正常
//         return TRUE;
//     }
// }


// /* 函数名: Installation_Angle_check_start()
//  * 描述：
//  * 返回值：
//  *        
//  *        
//  */
// void Installation_Angle_check(const or_point_cloud_format_t *PeakList)
// {
//     if (Radar_SensorState_List.OperationMode)
//     {
//         if ((0x00 == CalibrationPara.Adaptive_step)) //上电初始化为0x00
//         {
//             int      result = 0;
//             uint16_t i;
//             CalibrationPara.adaptive_Workmode = CHEACK_MODE;
//             CalibrationPara.Start             = 0;
//             CalibrationPara.Step              = 1;
//             //CalibrationPara.Master_Result     = 0;
//             CalibrationPara.Error_Number      = 0;
//             CalibrationPara.Effective_point   = 0;
//             CalibrationPara.SteeringAngle     = 0;
//             CalibrationPara.YawRate           = 0;
//             CalibrationPara.Velocity          = 0;
//             CalibrationPara.Counter           = 0;
//             CalibrationPara.Frame             = 0;
//             CalibrationPara.FalseFrame        = 0;
//             CalibrationPara.DataNum           = 0;
//             CalibrationPara.AveYdata          = 0;
//             CalibrationPara.Adap_A            = 0;
//             CalibrationPara.Adap_B            = 0;
//             //CalibrationPara.errType           = 0;
//             CalibrationPara.driving_profile   = 0; //驾驶指导初始化

//             // CalibrationPara.adaptive_PB = 0; //实时进度
//             // CalibrationPara.TEMP_PB     = 0; //固定进度

//             for (i = 0; i < CalibrationTime * Timeframe; i++) {
//                 CalibrationPara.xdata[i]    = 0;
//                 CalibrationPara.ydata[i]    = 0;
//                 CalibrationPara.rangdata[i] = 0;
//                 CalibrationPara.elevdata[i]  = 0;
//             }
//             CalibrationPara.Adap_Angle = 0;

//             CalibrationPara.Adaptive_Check_Angle    = 0;
//             CalibrationPara.Adaptive_Check_eleAngle = 0;
//             CalibrationPara.Adaptive_step           = 0x01; //标定参数初始化完成

//         } else if (0x01 == CalibrationPara.Adaptive_step) {
//             Adaptive_Calibration(PeakList);
//         } else if (0x02 == CalibrationPara.Adaptive_step) {
//             if (FALSE == Angle_Difference_judg()) {
//             }
//             CalibrationPara.Adaptive_step = 0x00;
//         }
//     }else{
//         Adaptive_CalibrationClear();
//         CalibrationPara.Adaptive_step = 0x00;
//     }

//     return ;
// }


    /******************************END OF FILE*************************************/
