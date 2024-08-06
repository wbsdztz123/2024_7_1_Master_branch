
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

    int result = 0;
    uint16_t i;
    CalibrationPara.Start = 0;
    CalibrationPara.Step = 1;
    CalibrationPara.Master_Result = 0;
    CalibrationPara.Error_Number = 0;
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
    CAL_MODE = CALIBRATION_RUNING;
    return result;
}



void Adaptive_CalibrationClear(void)
{
    uint16_t i;
        printf("clear calibration\n");
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
    
        CalibrationPara.errType =0;
        CalibrationPara.Driving_Profile = 0;


}


void Adaptive_Calibration(const or_point_cloud_format_t *PeakList)
{

    uint8_t tempProgress;
    
    Adaptive_CalibrationSaveData(PeakList); // 保存数据

    if (CalibrationPara.FalseFrame > (CalibrationTime / 10))
    {
        CalibrationPara.Adaptive_step = 7; //zjn test
        printf("111\n");
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
            printf("131\n");
            Adaptive_CalibrationClear(); // 标定放弃
            CalibrationPara.Driving_Profile = 0x10; // 目标不充分
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
    tempProgress = CalibrationPara.Counter * 14;
    tempProgress += 2;
    Calibration_Progress(tempProgress);

    printf("poind_count = %d\n",PeakList->point_count);
sleep(1);
    if (flag) // 车辆处于可以标定的状态，进行标定
    {
        CalibrationPara.Error_Number = 0; // 车速、档位、转弯正确，则时间清零。

printf("CalibrationPara.Start2 = %d\n", CalibrationPara.Start);
        if (CalibrationPara.Start == 0) // 没有开始标定
        {
            CalibrationPara.AveYdata = 0;
            rang_flag                = Rang_judge(PeakList); // 判断栅栏距离

            //printf("rang_flag = %d\n",rang_flag);
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
                printf("181\n");
                Adaptive_CalibrationClear(); // 标定放弃
                break;
            }


            for (i = 0; i < PeakList->point_count; i++) // 读取车辆旁边障碍物的位置，存储数据
            {
                Calibration_flag = CAL_Target_Filtering(PeakList, i);

                if (Calibration_flag && rang_flag) {
                    //printf("a")
                    float azimuth = PeakList->term[i].azimuth * 180 / PI;
                    temp_Ydata    = PeakList->term[i].range * sin((0 + azimuth) * PI / 180);
                    //printf("temp_Ydata = %f\n",temp_Ydata);
                    if (temp_Ydata > lower_bound && temp_Ydata < upper_bound) 
                    {
                        Start_num++;
                        CalibrationPara.AveYdata += temp_Ydata;
                    }
                }
            }

            //printf("CalibrationPara.AveYdata = %f\n", CalibrationPara.AveYdata);
            tempProgress = CalibrationPara.Counter * 14;
            tempProgress += 5;
            Calibration_Progress(tempProgress);
            printf("Start_num = %d\n", Start_num);
            sleep(1);
            if (Start_num >= Timeframe) {
                CalibrationPara.AveYdata = CalibrationPara.AveYdata / Start_num;

                printf("CalibrationPara.AveYdata = %f\n", CalibrationPara.AveYdata);

                if ((CalibrationPara.AveYdata > 0.5f) && (CalibrationPara.AveYdata < 8.0f)) //3.0-4.0-4.5-5.0-6.0-8.0
                {
                    CalibrationPara.Start = 1;

                    CalibrationPara.SteeringAngle = Message_VehicleMsg.SteeringAngle; // 存储标定开始的方向盘转角
                    CalibrationPara.Velocity = (Message_VehicleMsg.Velocity / 3.6f); // 存储标定开始的车速


                    tempProgress = CalibrationPara.Counter * 14;
                    tempProgress += 8;
                    Calibration_Progress(tempProgress);

                    CalibrationPara.Adaptive_step = 1; //zjn test
                } else {
                    CalibrationPara.Adaptive_step = 2; //zjn test
                    printf("233\n");
                    Adaptive_CalibrationClear(); // 标定放弃
                    CalibrationPara.Driving_Profile = 0x10; //目标不充分

                }
            } else {
                CalibrationPara.Adaptive_step = 3; //zjn test
                printf("240\n");
                Adaptive_CalibrationClear(); // 标定放弃
                CalibrationPara.Driving_Profile = 0x10; //目标不充分

            }

        } else // 已经开始标定
        {
            if ((fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle) < 3.0f) //5//6//10// 方向盘转角偏差小于10才行。
                && (fabs((Message_VehicleMsg.Velocity / 3.6f) - CalibrationPara.Velocity) < 3.0f)) // 3
            {
                Start_num = 0;
                for (i = 0; i < PeakList->point_count; i++) {
                    Calibration_flag = CAL_Target_Filtering(PeakList, i);
                    //printf("DATA_RUNING_3_Calibration_flag = %d\n",Calibration_flag);

                    if (Calibration_flag) {
                        float azimuth_deg = PeakList->term[i].azimuth * 180 / PI;
                        float X = PeakList->term[i].range * cos((0 + azimuth_deg) * PI / 180);
                        float Y = PeakList->term[i].range * sin((0 + azimuth_deg) * PI / 180);


                        if (CalibrationPara.AveYdata < 2.2f) {
                            if (CalibrationPara.DataNum < CalibrationTime * Timeframe
                                && Y > (CalibrationPara.AveYdata - (Calibration_Ydata_gap - 0.2))
                                && Y < (CalibrationPara.AveYdata + (Calibration_Ydata_gap - 0.2)) && X > 8.0f) {
                                CalibrationPara.Xdata[CalibrationPara.DataNum] = X;
                                CalibrationPara.Ydata[CalibrationPara.DataNum] = Y;
                                CalibrationPara.DataNum++;
                                Start_num++;
                            }
                        } else {
                            if (CalibrationPara.DataNum < CalibrationTime * Timeframe
                                && Y > (CalibrationPara.AveYdata - Calibration_Ydata_gap)
                                && Y < (CalibrationPara.AveYdata + Calibration_Ydata_gap) && X > 8.0f) 
                            {
                                CalibrationPara.Xdata[CalibrationPara.DataNum] = X;
                                CalibrationPara.Ydata[CalibrationPara.DataNum] = Y;
                                CalibrationPara.DataNum++;
                                Start_num++;
                            }
                        }
                    }
                }


                CalibrationPara.Frame++; // 帧数+1
                if (Start_num <= (Timeframe / 2 + 1)) {
                        CalibrationPara.FalseFrame++; // 连续无目标的帧数

                    if (CalibrationPara.FalseFrame > (CalibrationTime / 18)) //10
                    {
                        printf("DATA_RUNING_FalseFrame\n");
                        CalibrationPara.Adaptive_step = 5; //zjn test
                        Adaptive_CalibrationClear();       // 标定放弃
                        CalibrationPara.Driving_Profile = 0x10; //目标不充分
                    }

                } else {
                    CalibrationPara.FalseFrame = 0;

                }
            } else {
                printf("DATA_RUNING_Message_VehicleMsg\n");
                Adaptive_CalibrationClear(); // 标定放弃

                CalibrationPara.Adaptive_step = 6;                                                  //zjn test
                if (fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle) > 10.0f) //横摆角速度过大
                {
                    CalibrationPara.Driving_Profile = 0x04;
                } else if (fabs((Message_VehicleMsg.Velocity / 3.6f) - CalibrationPara.Velocity) > 3.0f) //纵向加速度过大
                {
                    CalibrationPara.Driving_Profile = 0x08;
                }


            }
        }
    } 
    else {
        CalibrationPara.Error_Number++; // 车速、档位、转弯半径不对的时候，不进行标定
        printf("DATA_RUNING_Message_VehicleMsg2\n");
        Adaptive_CalibrationClear();    // 标定放弃

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

    for (i = 0; i < CalibrationPara.DataNum; i++) 
    {
        float32_t x = CalibrationPara.Xdata[i];
        
        float32_t y = CalibrationPara.Ydata[i];
        //printf("x: %f, y: %f \n", x, y);
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
}





uint8_t Rang_judge(const or_point_cloud_format_t *PeakList)
{
    uint32_t i = 0;
    uint8_t result = 0;
    uint16_t rang0_4 = 0, rang4_8 = 0, rang8_ = 0;

    for (i = 0; i < PeakList->point_count; i++)
    {
        result = CAL_Target_Filtering(PeakList, i); 
        if((PeakList->term[i].range > 5) && (PeakList->term[i].range < 25))
        {
            if ((result))
            {
                float32_t temp_Ydata = PeakList->term[i].range * sin((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
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


uint8_t Body_Posture_Detection(void)
{
    printf("Message_VehicleMsg.Velocity = %f\n",Message_VehicleMsg.Velocity);
    uint8_t result = 0;
    float velocity = Message_VehicleMsg.Velocity / 3.6f;
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


uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList, uint8_t i)
{
    uint8_t Calibration_flag = 0;
    float32_t temp_speed_gap = 0xff;
    
    if (PeakList->term[i].range > CalibrationRangeMin 
        && PeakList->term[i].range < CalibrationRangeMax
        && (PeakList->term[i].azimuth * 180 / PI) > 0.0f
        && (PeakList->term[i].azimuth * 180 / PI) < 55.0f
        && PeakList->term[i].snr >= Calibration_MinRCs)
    {
        //printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,PeakList->term[i].range,i,PeakList->term[i].doppler,i,PeakList->term[i].azimuth,i,PeakList->term[i].snr);
        bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_FRONT);
        bool isDopplerNegative = PeakList->term[i].doppler < 0.0f;
        //printf("isLeftOrRightFront = %d, isDopplerNegative = %d\n",isLeftOrRightFront,isDopplerNegative);

        if ((isLeftOrRightFront && isDopplerNegative) || (!isLeftOrRightFront && !isDopplerNegative))
        {
            float32_t cosValue = cos((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
            float32_t speed = Message_VehicleMsg.Velocity / 3.6f;
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

/******************************END OF FILE*************************************/



