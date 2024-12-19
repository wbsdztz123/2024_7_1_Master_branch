#include "Adaptive.h"
#include "NEW_Adaptive.h"
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
    float32_t i;
    uint8_t subscript;
    float32_t ang;
    uint8_t  j;
    if(CalibrationPara.DataNum < POINT_THRESHOLD)  //数据量2000
    {
        Adaptive_CalibrationSaveData(PeakList); // 保存数据
        tempProgress =  5 +(CalibrationPara.DataNum%200) * 5;
        Calibration_Progress(tempProgress);
    }
    else{
        //计算角度的SSE
        tempProgress = 55;
        Calibration_Progress(tempProgress);
        

        // if(CalibrationPara.SSE_subscript < SSE_LEN)
        // {
        //     ang = TRAVERSE_START + (TRAVERSE_INCREMEETS * CalibrationPara.SSE_subscript);
        //     for( i = 0 ;i < POINT_THRESHOLD;i++)
        //     {
        //         CalibrationPara.Ang_SSE[CalibrationPara.SSE_subscript] += CALGet_SSE(ang,i);
        //     }
        //     //printf("CalibrationPara.Ang_SSE[%d] = %f\n",CalibrationPara.SSE_subscript,CalibrationPara.Ang_SSE[CalibrationPara.SSE_subscript]);
        //     CalibrationPara.SSE_subscript++;
            
        //     tempProgress += CalibrationPara.SSE_subscript * 2;
        //     Calibration_Progress(tempProgress);
        // }else{
        //     j = CAL_min_subscript(CalibrationPara.Ang_SSE, SSE_LEN);
        //     CAL_finsh(j);
        // }
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

    if (flag) // 车辆处于可以标定的状态，进行标定
    {
        tempProgress = 5;
        Calibration_Progress(tempProgress);
        if (CalibrationPara.Start == 0) // 没有开始标定
        {
            CalibrationPara.AveYdata = 0;
            for (i = 0; i < PeakList->point_count; i++)     // 读取车辆旁边障碍物的位置，存储数据
            {
                Calibration_flag = CAL_Target_Filtering(PeakList, i);//静止点筛选

                if (Calibration_flag) 
                {   float YD= PeakList->term[i].range * sin((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
                    float XD = PeakList->term[i].range * cos((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);

                    printf("YD = %f, XD = %f\n",YD,XD);
                    
                    YD_XD_writing(YD, XD);

                    //printf("ang = %f,dopplor = %f,rang = %f,VehicleMsg_Vel = %f\n",PeakList->term[i].azimuth*180/PI,PeakList->term[i].doppler,PeakList->term[i].range,Message_VehicleMsg.Velocity/3.6f);
                    CALSaved_Sample_Parameters(PeakList, i);//保存静止点
                }
            }
        }
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




uint8_t Body_Posture_Detection(void)
{
    //printf("Message_VehicleMsg.Velocity = %f\n",Message_VehicleMsg.Velocity);
    uint8_t result = 0;
    float velocity = Message_VehicleMsg.Velocity / 3.6f;
    float steeringAngle = fabs(Message_VehicleMsg.SteeringAngle);
    float curveRadius = fabs(Message_VehicleMsg.CurveRadius);
    float yawRate = fabs(Message_VehicleMsg.YawRate);

    result = (velocity > Calibration_MinVelocity ) && (velocity < Calibration_MaxVelocity) 
             && (steeringAngle < Calibration_MaxSteeringAngle) 
             && (curveRadius > Calibration_MaxRoadCurve) 
             && (yawRate < 0.8f);
    //printf("result= %d\n",result);
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
    float32_t temp_ang = fabs(PeakList->term[i].azimuth * 180 / PI);
    if (PeakList->term[i].range > CalibrationRangeMin 
        && PeakList->term[i].range < CalibrationRangeMax
        && temp_ang < 55.0f
        // && (PeakList->term[i].azimuth * 180 / PI) > 0.0f
        // && (PeakList->term[i].azimuth * 180 / PI) < -55.0f
        && PeakList->term[i].snr >= Calibration_MinRCs)
    {
        //printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,PeakList->term[i].range,i,PeakList->term[i].doppler,i,PeakList->term[i].azimuth,i,PeakList->term[i].snr);
        bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_FRONT);
        bool isDopplerNegative = PeakList->term[i].doppler < 0.0f;
        //printf("isLeftOrRightFront = %d, isDopplerNegative = %d\n",isLeftOrRightFront,isDopplerNegative);

        if ((isLeftOrRightFront && isDopplerNegative) || (!isLeftOrRightFront && !isDopplerNegative))
        {
            float32_t cosValue = cos((0 + temp_ang) * PI / 180);
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

void CALSaved_Sample_Parameters(const or_point_cloud_format_t *PeakList,uint8_t i)
{   
    float32_t peak_ANG = PeakList->term[i].azimuth * 180 / PI;
    if ((peak_ANG > -40.0f) && (peak_ANG < -30.0f)
        && (CalibrationPara.count_minus_40_to_minus_30 < 0)) {
        CalibrationPara.count_minus_40_to_minus_30++;
        //printf("CalibrationPara.count_minus_40_to_minus_30 = %d\n",CalibrationPara.count_minus_40_to_minus_30);

    } else if ((peak_ANG > -30.0f) && (peak_ANG < -20.0f)
               && (CalibrationPara.count_minus_30_to_minus_20 < 0)) {
        CalibrationPara.count_minus_30_to_minus_20++;
        //printf("CalibrationPara.count_minus_30_to_minus_20 = %d\n",CalibrationPara.count_minus_30_to_minus_20);
    } else if ((peak_ANG > -20.0f) && (peak_ANG < -10.0f)
               && (CalibrationPara.count_minus_20_to_minus_10 < 0)) {
        CalibrationPara.count_minus_20_to_minus_10++;
        //printf("CalibrationPara.count_minus_20_to_minus_10 = %d\n",CalibrationPara.count_minus_20_to_minus_10);
    } else if ((peak_ANG > -10.0f) && (peak_ANG < 0.0f)
               && (CalibrationPara.count_minus_10_to_minus_0 < 600)) {
        CalibrationPara.count_minus_10_to_minus_0++;
        //printf("CalibrationPara.count_minus_10_to_minus_0 = %d\n",CalibrationPara.count_minus_10_to_minus_0);
    } else if ((peak_ANG > 0.0f) && (peak_ANG < 10.0f)
               && (CalibrationPara.count_0_to_10 < 0)) {
        CalibrationPara.count_0_to_10++;
        //printf("CalibrationPara.count_0_to_10 = %d\n",CalibrationPara.count_0_to_10);
    } else if ((peak_ANG > 10.0f) && (peak_ANG < 20.0f)
               && (CalibrationPara.count_10_to_20 < 0)) {
        CalibrationPara.count_10_to_20++;
        //printf("CalibrationPara.count_10_to_20 = %d\n",CalibrationPara.count_10_to_20);
    } else if ((peak_ANG > 20.0f) && (peak_ANG < 30.0f)
               && (CalibrationPara.count_20_to_30 <0)) {
        CalibrationPara.count_20_to_30++;
        //printf("CalibrationPara.count_20_to_30 = %d\n",CalibrationPara.count_20_to_30);
    } else if ((peak_ANG > 30.0f) && (peak_ANG < 40.0f)
               && (CalibrationPara.count_30_to_40 < 0)) {
        CalibrationPara.count_30_to_40++;
        //printf("CalibrationPara.count_30_to_40 = %d\n",CalibrationPara.count_30_to_40);
    } else {
        return;
    }
    // printf("CalibrationPara.DataNum = %d\n",CalibrationPara.DataNum);
    // printf("CalibrationPara.count_minus_40_to_minus_30 = %d\n",CalibrationPara.count_minus_40_to_minus_30);
    // printf("CalibrationPara.count_minus_30_to_minus_20 = %d\n",CalibrationPara.count_minus_30_to_minus_20);
    // printf("CalibrationPara.count_minus_20_to_minus_10 = %d\n",CalibrationPara.count_minus_20_to_minus_10);
    // printf("CalibrationPara.count_minus_10_to_minus_0 = %d\n",CalibrationPara.count_minus_10_to_minus_0);
    // printf("CalibrationPara.count_0_to_10 = %d\n",CalibrationPara.count_0_to_10);
    // printf("CalibrationPara.count_10_to_20 = %d\n",CalibrationPara.count_10_to_20);
    // printf("CalibrationPara.count_20_to_30 = %d\n",CalibrationPara.count_20_to_30);
    // printf("CalibrationPara.count_30_to_40 = %d\n",CalibrationPara.count_30_to_40);

    CalibrationPara.Vradar[CalibrationPara.DataNum]    = PeakList->term[i].doppler;
    CalibrationPara.Vtarget[CalibrationPara.DataNum]   = (Message_VehicleMsg.Velocity / 3.6f);
    CalibrationPara.Ang_radar[CalibrationPara.DataNum] = (PeakList->term[i].azimuth);

    if( CalibrationPara.DataNum < POINT_THRESHOLD)//
    {
        CalibrationPara.DataNum++;
    }
    return;
}









void CAL_finsh(uint8_t i)
{
    float32_t TmpLinearAngle = 0;
    uint8_t tempProgress;
    uint8_t StatusArray[9] = {0};

    RadarPara.FarHorizontalAdptiveAngle  = TmpLinearAngle;
    RadarPara.FarVerticalAdptiveAngle    = RadarPara.FarVerticalOffsetAngle;
    RadarPara.TempHorizontalAdptiveAngle = RadarPara.FarHorizontalAdptiveAngle;
    RadarPara.TempVerticalAdptiveAngle   = RadarPara.FarVerticalOffsetAngle;
    tempProgress                         = 100;
    Calibration_Progress(tempProgress);
    CalibrationPara.Step = 3; // 标定完成
    
    printf("/********************************This_is_result*******************************/\n");
    printf("/******************************************************************************/\n");
    printf("/*****************************TmpLinearAngle = %f****************************/\n",TmpLinearAngle);
    printf("/******************************************************************************/\n");
    printf("/******************************************************************************/\n");
    printf("/******************************************************************************/\n");

    CalibrationPara.Driving_Profile = 0x00;
    StatusArray[0]                  = 0x00;
    StatusArray[1]                  = 0x00;
    StatusArray[2]                  = (((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) >> 8) & 0xFF;
    StatusArray[3]                  = ((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) & 0xFF;
    StatusArray[4]                  = (((int16_t)(RadarPara.FarVerticalOffsetAngle * 100)) >> 8) & 0xFF;
    StatusArray[5]                  = ((int16_t)(RadarPara.FarVerticalOffsetAngle * 100)) & 0xFF;

    //Adaptive_Calibration_Exit(StatusArray);

    if ((fabs(TmpLinearAngle) > 5.0f) || (fabs(RadarPara.FarVerticalAdptiveAngle) > 3.0f)) {
        StatusArray[0] = 0x00;
        StatusArray[1] = 0x02;
        // Adaptive_Calibration_Exit(StatusArray);
        // SetDtcCalOutOfRange_0x9ED546(TRUE);
        // SetDtcMissCal_0x9ED554(TRUE);
        CalibrationPara.Master_Result = 2; // 雷达标定结果为失败
        CalibrationPara.errType       = 0x02;
    } else {
        CalibrationPara.Master_Result = 1; // 雷达标定结果为成功
        // SetDtcCalOutOfRange_0x9ED546(FALSE);
        // SetDtcCalOutOfRange_0x9ED955(FALSE);
        // SetDtcMissCal_0x9ED554(FALSE);
    }
    CAL_MODE = CALIBRATION_EXIT;
}
/******************************END OF FILE*************************************/



