
/****************************************************************************
 *                        File: adaptive_calibration.c                      *
 *                        @sjw20210713                                           *
 ****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "QX_Adaptive.h"
CalibrationParaS CalibrationPara = {0};
Message_VehicleMsgS Message_VehicleMsg = {0};
RadarParaS RadarPara = {0};
GTRACK_measurementPoint Peakpoint[GTRACK_NUM_POINTS_MAX] = {0};
extern CALIBRATION_MODE CAL_MODE;

/* 函数名: AdaptiveCalStart()
 * 描述：售后校准开始
 * 返回值：0x00=标定例程成功开启
 *        0x01=标定正在运行中
 *        0x02=标定失败
 */
uint8_t AdaptiveCalStart(void)
{
    int result = 0;
    uint16_t i;
    //int32_t radar_data = WORKIN_MODE_ADAPTIVE_CALIBRATION;

   RadarPara.InstallAngle = RadarInstallAngle;
    CalibrationPara.Start = 0;
    CalibrationPara.Step = 1;
    CalibrationPara.Master_Result = 0;
    CalibrationPara.Error_Number = 0;
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
    CalibrationPara.Driving_Profile = 0;//驾驶指导初始化
    
    CalibrationPara.adaptive_PB = 0; //实时进度
    CalibrationPara.TEMP_PB = 0;  //固定进度
    CalibrationPara.Adaptive_step = 0; // zjn test
   
    //Timer_Start(TIMER_ID_ONESHOT_ADPCAL_600S, TIMER_CNT_600s, TIMER_KIND_ONESHOT, timer_fuc_ADPCAL_end);//定时10分钟
    for (i = 0; i < CalibrationTime * Timeframe; i++)
    {
        CalibrationPara.Xdata[i] = 0;
        CalibrationPara.Ydata[i] = 0;
    }
    CalibrationPara.Adap_Angle = 0;

    if (CalibrationPara.Step == 1)
    {
        RadarPara.TempHorizontalAdptiveAngle = RadarPara.FarHorizontalAdptiveAngle;
        RadarPara.TempVerticalAdptiveAngle =  RadarPara.FarVerticalAdptiveAngle;
        RadarPara.FarHorizontalAdptiveAngle = 0; // 远区自适应水平角度清零
        RadarPara.FarVerticalAdptiveAngle = 0;   // 远区自适应垂直角度清零
    }
    // RadarPara.WorkMode = 11;
    // if(RadarPara.WorkMode != 11){
    //     result = 0x02;
    // }
    CAL_MODE  = CALIBRATION_RUNING;
    return result;
}

/* 函数名：AdaptiveCalTimeoutProc()
 * 描述：售后校准超时回调函数
 * 输入：clock: clock object
 *       argv: 传入参数
 * 返回值：NA
 */
void timer_fuc_ADPCAL_end(void)
{
    CalibrationPara.errType = 0X01;
    CalibrationPara.Master_Result = 2;
    // if (Comm_Control.PowerSupply_Control == 2)
    // {
    //     // EMBARC_PRINTF("Can Sleep\r\n");
    //     // Can_Sleep();
    //     Comm_Control.PowerSupply_Control = 0;
    // }
    //超时失败置位
    //Adaptive_Calibration_Exit(StatusArray);
}
// static void AdaptiveCalTimeoutProc(ClockP_Object *clock, void *argv)
// {
//     //_3sradar_tui_print("AdaptiveCalTimeoutProc++\r\n");
//     uint8_t StatusArray[6] = {0};
//     CalibrationPara.errType = 0X01;
//     CalibrationPara.Master_Result = 2;
    
//      StatusArray[0] = 0x02;
//      StatusArray[1] = 0x01;
//      StatusArray[2] = (((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) >> 8) & 0xFF;
//      StatusArray[3] = ((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) & 0xFF;
//      StatusArray[4] = (((int16_t)(RadarPara.FarVerticalOffsetAngle * 100)) >> 8) & 0xFF;
//      StatusArray[5] =((int16_t)(RadarPara.FarVerticalOffsetAngle * 100)) & 0xFF;
//      Adaptive_Calibration_Exit(StatusArray);
//     // SetDtcMissCal_0x9ED554(TRUE);
     
// }


/* 函数名：AdaptiveCalClockLaunch()
 * 描述：启动售后校准超时clock
 * 输入：period: 超时时长(usec)-
 * 返回值：NA
 * 备注：停止并销毁clock示例代码如下，
 *      ClockP_stop(adaptive_cal_clock);
 *      ClockP_destruct(adaptive_cal_clock);
 * 
 */
// void AdaptiveCalClockLaunch(uint64_t period)
// {
//     // ClockP_Params clock_params;

//     // ClockP_Params_init(&clock_params);
//     // clock_params.timeout = ClockP_usecToTicks(period);
//     // clock_params.start = 1;
//     // clock_params.callback = &AdaptiveCalTimeoutProc;
//     // clock_params.args = NULL;

//     // ClockP_construct(&adaptive_cal_clock, &clock_params);
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
void GetAdaptiveCalStatus(uint8_t * StatusArray)
{
    // int16_t TempFarHorizontalAdptiveAngle, TempFarVerticalAdptiveAngle;
    
    // if (CalibrationPara.Master_Result == 1 || CalibrationPara.errType == 0x02) {
    //     TempFarHorizontalAdptiveAngle = (int16_t)((RadarPara.FarHorizontalAdptiveAngle + RadarPara.FarHorizontalOffsetAngle) * 100);
    //     TempFarVerticalAdptiveAngle = (int16_t)(RadarPara.FarVerticalAdptiveAngle * 100);
    // } else {
    //     TempFarHorizontalAdptiveAngle = 0;
    //     TempFarVerticalAdptiveAngle   = 0;
    // }



    if(!CalibrationPara.Master_Result){
       *StatusArray  = 0x01;
      
    }else if(CalibrationPara.Master_Result == 1){
       *StatusArray  = 0x00; 
    }else if(CalibrationPara.Master_Result == 2)
    {
        *StatusArray  = 0x02;
    }
    
    *(StatusArray + 1) = CalibrationPara.adaptive_PB; 
    *(StatusArray + 2) = CalibrationPara.errType;


}


/* 函数名: SaveAdaptiveCalStatus_DID_0x0106()
 * 描述：保存售后校准状态
 * 输入: StatusArray
 *        byte 0: 标定结果 (0x00=标定成功, 0x01=标定进行中, 0x02=标定未完成)
 *        byte 1: 标定失败原因 (0x00：无故障, 0x01：标定超时, 0x02：角度偏差过大, 0x03：NVM写入错误,
 *                             0x04:用户终止, 0x05:SDA 校准失败, 0x06~0xFE：预留, 0xFF:标定例程未开始)
 *        byte 2~3: 水平角度误差值 (Resolution=0.01, Offset=0) 
 *                  Data[2] = (HorizontalAngle >> 8) & 0xFF;
 *                  Data[3] = HorizontalAngle & 0xFF;
 *        byte 4~5: 垂直角度误差值 (Resolution=0.01, Offset=0)
 *                  Data[4] = (VerticalAngle >> 8) & 0xFF;
 *                  Data[5] = VerticalAngle & 0xFF;
 * 
 * 返回值：NA
 */
void SaveAdaptiveCalStatus_DID_0x4902(uint8_t * StatusArray)
{
    // for(uint8_t index = 0; index < DataLength_Data_4902; index++)
    // {
    //     FLASH_DID_DATA_BUF[index + FaultDID_ADDR2_4902Add] = StatusArray[index];
    // }

    // qspi_Write_DID_flg = TRUE;
}

void Adaptive_CalibrationClear(void)
{
   uint16_t i;
    if (CalibrationPara.Start == 1||CalibrationPara.Start == 2)
    {
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
        CalibrationPara.Error_Number = 0;
        

        CalibrationPara.TEMP_PB = 0;  //标定进度清零

        for (i = 0; i < CalibrationTime*Timeframe; i++)
        {
            CalibrationPara.Xdata[i] = 0;
            CalibrationPara.Ydata[i] = 0;
        }
        CalibrationPara.Adap_Angle = 0;
        
        // if( CalibrationPara.Counter < 3)
        // {
        //     CalibrationPara.Counter++;
        //     CalibrationPara.errType =0;
        //     CalibrationPara.Driving_Profile = 0;
        // }
        // else
        // {
        //     CalibrationPara.Counter = 0;
        // }
    }
}


void Adaptive_Calibration(uint32_t point_count,GTRACK_measurementPoint *PeakList)
{
    
    uint8_t tempProgress;
    
    Adaptive_CalibrationSaveData(point_count,PeakList); // 保存数据

    if (CalibrationPara.FalseFrame > (CalibrationTime / 10))
    {
        CalibrationPara.Adaptive_step = 7; //zjn test

        Adaptive_CalibrationClear(); // 标定放弃
        CalibrationPara.Driving_Profile = 0x10; // 目标不充分
    }
    else if (CalibrationPara.Frame >= CalibrationTime || CalibrationPara.DataNum >= CalibrationTime * Timeframe)
    {
        CalibrationPara.Adaptive_step = 8; //zjn test
        tempProgress = CalibrationPara.Counter * 14 + 10;
        Calibration_Progress(tempProgress);

        if (CalibrationPara.DataNum >= (CalibrationTime * Timeframe * 3 / 5))
        {
            CalibrationPara.Adaptive_step = 9; //zjn test

            Adaptive_CalibrationPolyFit(); // 线性回归拟合
            //Adaptive_CalibrationFinish();
        }
        else
        {
            CalibrationPara.Adaptive_step = 10; //zjn test

            Adaptive_CalibrationClear(); // 标定放弃
            CalibrationPara.Driving_Profile = 0x10; // 目标不充分
        }
    }
}


// 数据保存
void Adaptive_CalibrationSaveData(uint32_t point_count,GTRACK_measurementPoint *PeakList)
{
    uint8_t  flag;
    uint8_t  Calibration_flag;
    uint32_t i;
    uint8_t  Start_num = 0;
    //float32_t tmpCalibrationRange;
    float32_t temp_speed_gap = 0xff;
    float32_t temp_Ydata;
    uint8_t   rang_flag = 0;
    uint8_t   tempProgress;
    flag         = Body_Posture_Detection(); //车身姿态检测
    tempProgress = CalibrationPara.Counter * 14;
    tempProgress += 2;
    Calibration_Progress(tempProgress);

printf("flag: %d\n",flag);
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


        if (CalibrationPara.Start == 0) // 没有开始标定
        {
            CalibrationPara.AveYdata = 0;
            rang_flag                = Rang_judge(point_count,PeakList); // 判断栅栏距离

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
            //printf("lower_bound = %f, upper_bound = %f\n",lower_bound,upper_bound);

            for (i = 0; i < point_count ;i++) // 读取车辆旁边障碍物的位置，存储数据
            {
                Calibration_flag = CAL_Target_Filtering(PeakList, i);
                //printf("calibration_flag = %f\n",Calibration_flag);
                if (Calibration_flag && rang_flag) {
                    float azimuth = PeakList[i].vector.azimuth * 180 / PI;
                    temp_Ydata    = PeakList[i].vector.range * sin((RadarPara.InstallAngle + azimuth) * PI / 180);

                    if (temp_Ydata > lower_bound && temp_Ydata < upper_bound) {
                        Start_num++;
                        CalibrationPara.AveYdata += temp_Ydata;
                    }
                }
            }


            tempProgress = CalibrationPara.Counter * 14;
            tempProgress += 5;
            Calibration_Progress(tempProgress);
            //printf("start_num = %d\n",Start_num);
            if (Start_num >= Timeframe) {
                CalibrationPara.AveYdata = CalibrationPara.AveYdata / Start_num;
                if ((CalibrationPara.AveYdata > 0.5f) && (CalibrationPara.AveYdata < 7.0f)) //3.0-4.0-4.5-5.0-6.0-8.0
                {
                    CalibrationPara.Start = 1;
                    CalibrationPara.SteeringAngle = Message_VehicleMsg.SteeringAngle; // 存储标定开始的方向盘转角
                    CalibrationPara.Velocity = (Message_VehicleMsg.Velocity); // 存储标定开始的车速


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
                    tempProgress = CalibrationPara.Counter * 14;
                    tempProgress += 8;
                    Calibration_Progress(tempProgress);

                    CalibrationPara.Adaptive_step = 1; //zjn test
                } else {
                    CalibrationPara.Adaptive_step = 2; //zjn test
                    //CalibrationPara.Start = 2;      //单帧点符合但是栅栏横向距离不符合
                    Adaptive_CalibrationClear(); // 标定放弃
                    //CalibrationPara.errType = 0x06;
                    CalibrationPara.Driving_Profile = 0x10; //目标不充分
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
                CalibrationPara.Adaptive_step = 3; //zjn test
                //CalibrationPara.Start = 2;     //单帧点不符合
                Adaptive_CalibrationClear(); // 标定放弃
                //CalibrationPara.errType = 0x06;
                CalibrationPara.Driving_Profile = 0x10; //目标不充分
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
            if ((fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle)
                 < 3.0f) //5//6//10// 方向盘转角偏差小于10才行。
                && (fabs((Message_VehicleMsg.Velocity) - CalibrationPara.Velocity) < 3.0f)) // 3
            {
                Start_num = 0;
                for (i = 0; i < point_count ;i++) {
                    Calibration_flag = CAL_Target_Filtering(PeakList, i);

                    if (Calibration_flag) {
                        float azimuth_deg = PeakList[i].vector.azimuth * 180 / PI;
                        float X = PeakList[i].vector.range * cos((RadarPara.InstallAngle + azimuth_deg) * PI / 180);
                        float Y = PeakList[i].vector.range * sin((RadarPara.InstallAngle + azimuth_deg) * PI / 180);

                        if (CalibrationPara.AveYdata < 2.2f) {
                            if (CalibrationPara.DataNum < CalibrationTime * Timeframe
                                && Y > (CalibrationPara.AveYdata - (Calibration_Ydata_gap - 0.2))
                                && Y < (CalibrationPara.AveYdata + (Calibration_Ydata_gap - 0.2)) && X > 5.0f) {
                                CalibrationPara.Xdata[CalibrationPara.DataNum] = X;
                                CalibrationPara.Ydata[CalibrationPara.DataNum] = Y;
                                CalibrationPara.DataNum++;
                                Start_num++;
                            }
                        } else {
                            if (CalibrationPara.DataNum < CalibrationTime * Timeframe
                                && Y > (CalibrationPara.AveYdata - Calibration_Ydata_gap)
                                && Y < (CalibrationPara.AveYdata + Calibration_Ydata_gap) && X > 5.0f) {
                                CalibrationPara.Xdata[CalibrationPara.DataNum] = X;
                                CalibrationPara.Ydata[CalibrationPara.DataNum] = Y;
                                CalibrationPara.DataNum++;
                                Start_num++;
                            }
                        }
                    }
                }


                CalibrationPara.Frame++; // 帧数+1
                printf("Start_num_2: %d\n", Start_num);
                printf("CalibrationPara.DataNum:%d\n", CalibrationPara.DataNum);

                if (Start_num <= (Timeframe / 2 + 1)) {
                    CalibrationPara.FalseFrame++; // 连续无目标的帧数

                    if (CalibrationPara.FalseFrame > (CalibrationTime / 18)) //10
                    {
                        CalibrationPara.Adaptive_step = 5; //zjn test
                        Adaptive_CalibrationClear();       // 标定放弃
                        //CalibrationPara.errType = 0x06;
                        CalibrationPara.Driving_Profile = 0x10; //目标不充分
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

                CalibrationPara.Adaptive_step = 6;                                                  //zjn test
                if (fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle) > 10.0f) //横摆角速度过大
                {
                    CalibrationPara.Driving_Profile = 0x04;
                } else if (fabs((Message_VehicleMsg.Velocity) - CalibrationPara.Velocity)
                           > 3.0f) //纵向加速度过大
                {
                    CalibrationPara.Driving_Profile = 0x08;
                }

                //CalibrationPara.errType = 0x06;

            }
        }
    } else {
        CalibrationPara.Error_Number++; // 车速、档位、转弯半径不对的时候，不进行标定
        Adaptive_CalibrationClear();    // 标定放弃
                                        // if (CalibrationPara.Error_Number > 500) // 30s时间
                                        // {
//CalibrationPara.errType = 0x06;  //车辆信号不满足
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
    uint16_t  i      = 0;
    float32_t a;
    float32_t b;

    for (i = 0; i < CalibrationPara.DataNum; i++) 
    {
        float32_t x = CalibrationPara.Xdata[i];
        float32_t y = CalibrationPara.Ydata[i];

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

    printf("/********************************This_is_result*******************************/\n");
    printf("/******************************************************************************/\n");
    printf("/*****************************Adap_Angle = %f****************************/\n",CalibrationPara.Adap_Angle);
    printf("/******************************************************************************/\n");
    printf("/******************************************************************************/\n");
    printf("/******************************************************************************/\n");
    CAL_MODE = CALIBRATION_EXIT;

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







uint8_t Rang_judge(uint32_t point_count,GTRACK_measurementPoint *PeakList)
{
    uint16_t i = 0;
    uint8_t result = 0;
    uint16_t rang0_2 = 0, rang2_4 = 0, rang4_7 = 0;

    for (i = 0; i < point_count;i++)
    {
        result = CAL_Target_Filtering(PeakList, i); 
        if (result)
        {
            float32_t temp_Ydata = PeakList[i].vector.range * sin((RadarPara.InstallAngle + (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180);
            if (temp_Ydata < 2.01f) {
                rang0_2++;
            } else if (temp_Ydata < 4.01f) {
                rang2_4++;
            } else {
                rang4_7++;
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




uint8_t Body_Posture_Detection(void)
{
    uint8_t result = 0;
    float velocity = Message_VehicleMsg.Velocity;
    float steeringAngle = fabs(Message_VehicleMsg.SteeringAngle);
    float curveRadius = fabs(Message_VehicleMsg.CurveRadius);
    float yawRate = fabs(Message_VehicleMsg.YawRate);

    result = (velocity > Calibration_MinVelocity ) && (velocity < Calibration_MaxVelocity) 
             && (steeringAngle < Calibration_MaxSteeringAngle) 
             && (curveRadius > Calibration_MaxRoadCurve) 
             && (yawRate < 0.8f);

    return result;
}



void Calibration_Progress(uint8_t pace)
{
    CalibrationPara.TEMP_PB = max(pace, CalibrationPara.TEMP_PB);
    CalibrationPara.adaptive_PB = max(CalibrationPara.TEMP_PB, CalibrationPara.adaptive_PB);
}



// uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList,uint8_t i)
// {
//     uint8_t Calibration_flag = 0;
//     float32_t temp_speed_gap = 0xff;
    
//     if ((PeakList[i].vector.range > CalibrationRangeMin) 
//         && (PeakList[i].vector.range < CalibrationRangeMax)
//         && ((PeakList[i].vector.azimuth * 180 / PI) > -10.0f)
//         && (PeakList[i].snr>= Calibration_MinRCs)) // 判断此目标是否是静态目标
//     {
//         if ((((RadarPara.InstallPosition == INSTALL_LEFT_FRONT) || (RadarPara.InstallPosition == INSTALL_RIGHT_FRONT))
//              && (PeakList[i].vector.doppler < 0.0f))|| (((RadarPara.InstallPosition != INSTALL_LEFT_FRONT)&& (RadarPara.InstallPosition != INSTALL_RIGHT_FRONT))
//              && (PeakList[i].vector.doppler > 0.0f))) 
//         {
//             if ((RadarPara.InstallPosition == INSTALL_LEFT_FRONT)|| (RadarPara.InstallPosition == INSTALL_RIGHT_FRONT)) {
//                 temp_speed_gap = fabs(PeakList[i].vector.doppler/ cos((RadarPara.InstallAngle - (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180)+ (Message_VehicleMsg.Velocity / 3.6f));
//             } else {
//                 temp_speed_gap = fabs(PeakList[i].vector.doppler/ cos((RadarPara.InstallAngle - (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180)- (Message_VehicleMsg.Velocity / 3.6f));
//             }
//             if ((Message_VehicleMsg.Velocity / 3.6f) < 4.1f) {
//                 Calibration_flag = (temp_speed_gap < (Message_VehicleMsg.Velocity / 3.6f) * 0.15f);
//             } else if (((Message_VehicleMsg.Velocity / 3.6f) >= 4.1f)
//                        && ((Message_VehicleMsg.Velocity / 3.6f) < 8.3f)) {
//                 Calibration_flag = (temp_speed_gap < ((Message_VehicleMsg.Velocity / 3.6f) * 0.1f));
//             } else {
//                 Calibration_flag = (temp_speed_gap < ((Message_VehicleMsg.Velocity / 3.6f) * 0.08f));
//             }
//         }
//     }

//     return Calibration_flag;
// }
uint8_t CAL_Target_Filtering(GTRACK_measurementPoint *PeakList,uint8_t i)
{
    uint8_t Calibration_flag = 0;
    float32_t temp_speed_gap = 0xff;
    
    if (PeakList[i].vector.range > CalibrationRangeMin 
        && PeakList[i].vector.range < CalibrationRangeMax
        && (PeakList[i].vector.azimuth * 180 / PI) < 10.0f
        && PeakList[i].snr>= Calibration_MinRCs)
    {
        printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,PeakList[i].vector.range,i,PeakList[i].vector.doppler,i,PeakList[i].vector.azimuth * 180 / PI,i,PeakList[i].snr);
        bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_LEFT_FRONT || RadarPara.InstallPosition == INSTALL_RIGHT_FRONT);
        bool isDopplerNegative = PeakList[i].vector.doppler < 0.0f;

        if ((isLeftOrRightFront && isDopplerNegative) || (!isLeftOrRightFront && !isDopplerNegative))
        {
           // printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,PeakList[i].vector.range,i,PeakList[i].vector.doppler,i,PeakList[i].vector.azimuth,i,PeakList[i].snr);
            
            float32_t cosValue = cos((RadarPara.InstallAngle + (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180);
            float32_t speed = Message_VehicleMsg.Velocity;
           // printf("speed = %f\n",speed);
            float32_t threshold = 0.15f;

            if (speed >= 4.1f && speed < 8.3f)
            {
                threshold = 0.1f;
            }
            else if (speed >= 8.3f)
            {
                threshold = 0.08f;//0.08
            }

            if (isLeftOrRightFront)
            {
                temp_speed_gap = fabs(PeakList[i].vector.doppler / cosValue + speed);
            }
            else
            {
                temp_speed_gap = fabs(PeakList[i].vector.doppler / cosValue - speed);
            }

            Calibration_flag = (temp_speed_gap < speed * threshold);
        }
    }

    return Calibration_flag;
}

/******************************END OF FILE*************************************/



