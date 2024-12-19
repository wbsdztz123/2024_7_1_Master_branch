#include "P2341_Adaptive.h"
CalibrationParaS CalibrationPara = {0};
Message_VehicleMsgS Message_VehicleMsg = {0};
RadarParaS RadarPara = {0};
GTRACK_measurementPoint Peakpoint[GTRACK_NUM_POINTS_MAX] = {0};
extern CALIBRATION_MODE CAL_MODE;

extern void Calibration_Screening_Angle(void);
extern void YD_XD_writing(float YD,float XD);


void Adaptive_CalibrationInit(void)
{
    uint16_t i;
    CalibrationPara.Step = 1;
    CalibrationPara.Start = 0;
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

    for(i = 0; i < CalibrationTime*Timeframe; i++)
    {
        CalibrationPara.Xdata[i] = 0;
        CalibrationPara.Ydata[i] = 0;
    }
    CalibrationPara.Adap_Angle = 0;

    if (CalibrationPara.Step == 1)
    {
        RadarPara.FarHorizontalOffsetAngle = 0; // 远区自适应水平角度清零
    }
    CAL_MODE = CALIBRATION_RUNING;
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
        
        Calibration_Screening_Angle();
        CalibrationPara.TEMP_PB = 0;  //标定进度清零

        for (i = 0; i < CalibrationTime*Timeframe; i++)
        {
            CalibrationPara.Xdata[i] = 0;
            CalibrationPara.Ydata[i] = 0;
        }
        CalibrationPara.Adap_Angle = 0;
        
        if( CalibrationPara.Counter < 3)
        {
            CalibrationPara.Counter++;
            CalibrationPara.errType =0;
            CalibrationPara.Driving_Profile = 0;
        }
        else
        {
            CalibrationPara.Counter = 0;
        }
    }
}


void Adaptive_Calibration(uint32_t gNumPoints, GTRACK_measurementPoint *PeakList)
{
    if(CalibrationPara.Step == 1)
    {
        Adaptive_CalibrationSaveData(gNumPoints, PeakList); //  保存数据
        if(CalibrationPara.FalseFrame > (CalibrationTime/18))
        {
            Adaptive_CalibrationClear();
        }
        else if((CalibrationPara.Frame >= CalibrationTime) || (CalibrationPara.DataNum >= CalibrationTime*Timeframe)) // 帧数够了，或者数据满了。
        {
            if(CalibrationPara.DataNum >= (CalibrationTime*Timeframe*2/3))// (CalibrationTime*Timeframe*2/3))
            {
                Adaptive_CalibrationPolyFit(); // 线性回归拟合
                Adaptive_CalibrationFinish();
            }
            else
            {
                Adaptive_CalibrationClear();
            }
        }
         if (CalibrationPara.Counter >= 3)
        {
            Adaptive_CalibrationClear();
            CalibrationPara.Step = 3;          //标定完成
            CalibrationPara.Master_Result = 2; //  雷达标定结果为失败
        }
    }
}

void Adaptive_CalibrationSaveData(uint32_t gNumPoints, GTRACK_measurementPoint *PeakList)
{
    //CAN_DCANData  TxMessage;
    uint8_t flag;
    uint8_t Calibration_flag;
    uint8_t i;
    uint8_t Start_num = 0;
    float32_t tmpCalibrationRange;
    float32_t temp_speed_gap = 0xff;
    float32_t temp_Ydata;

    Message_VehicleMsg.Velocity = Message_VehicleMsg.Velocity/3.6f;
    RadarPara.InstallPosition = INSTALL_LEFT_BACK;
    flag = ((Message_VehicleMsg.Velocity > Calibration_MinVelocity)  && (Message_VehicleMsg.Velocity < Calibration_MaxVelocity));
    //flag = flag && (Message_VehicleMsg.Gear == 4);
    flag = flag && (fabs(Message_VehicleMsg.SteeringAngle) <  Calibration_MaxSteeringAngle); // 方向盘转角限制
    flag = flag && (fabs(Message_VehicleMsg.CurveRadius) >  Calibration_MaxRoadCurve); // 转弯半径限制

    if(flag) // 车辆处于可以标定的状态，进行标定
    {
        CalibrationPara.Error_Number = 0;// 车速、档位、转弯正确，则时间清零。
        if(CalibrationPara.Start == 0) // 没有开始标定
        {
            CalibrationPara.AveYdata = 0;
            for(i=0; i < gNumPoints; i++) // 读取车辆旁边障碍物的位置，存储数据
            {
                if ((PeakList[i].vector.range > CalibrationRangeMin) 
                && (PeakList[i].vector.range < CalibrationRangeMax) 
                && ((PeakList[i].vector.azimuth * 180 / PI) < 0) 
                && ((PeakList[i].vector.azimuth * 180 / PI) > -35.0f) 
                && (PeakList[i].snr >= Calibration_MinRCs))  // 判断此目标是否是静态目标
                {

                    if((((RadarPara.InstallPosition == INSTALL_LEFT_FRONT)||(RadarPara.InstallPosition == INSTALL_RIGHT_FRONT))&&(PeakList[i].vector.doppler< 0))
                        ||(((RadarPara.InstallPosition != INSTALL_LEFT_FRONT)&&(RadarPara.InstallPosition != INSTALL_RIGHT_FRONT))&&(PeakList[i].vector.doppler> 0)))
                    {
                        if((RadarPara.InstallPosition == INSTALL_LEFT_FRONT)||(RadarPara.InstallPosition == INSTALL_RIGHT_FRONT))
                        {
                            temp_speed_gap = fabs(PeakList[i].vector.doppler/cos((RadarInstallAngle + (PeakList[i].vector.azimuth * 180/PI))*PI/180) + Message_VehicleMsg.Velocity);
                        }
                        else
                        {
                            temp_speed_gap = fabs(PeakList[i].vector.doppler/cos((RadarInstallAngle + (PeakList[i].vector.azimuth * 180/PI))*PI/180) - Message_VehicleMsg.Velocity);
                        }
                        if(Message_VehicleMsg.Velocity < 4.1f)
                        {
                            Calibration_flag = temp_speed_gap < Message_VehicleMsg.Velocity*0.2f;
                        }
                        else if((Message_VehicleMsg.Velocity >= 4.1f) && (Message_VehicleMsg.Velocity < 8.3f))
                        {
                            Calibration_flag = temp_speed_gap < Message_VehicleMsg.Velocity*0.12f;
                        }
                        else
                        {
                            Calibration_flag = temp_speed_gap < Message_VehicleMsg.Velocity*0.1f;
                        }

                        if(Calibration_flag)
                        {
                            temp_Ydata = PeakList[i].vector.range * sin((RadarInstallAngle + (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180);
                            if((temp_Ydata > -0.5f) && (temp_Ydata < 4.0f))
                            {
                                Start_num++;
                                CalibrationPara.AveYdata = CalibrationPara.AveYdata + temp_Ydata;
                            }
                        }
                    }
                }
            }
            if(Start_num >= Timeframe)
            {
                CalibrationPara.AveYdata = CalibrationPara.AveYdata/Start_num;
                if((CalibrationPara.AveYdata > 0.5f ) && (CalibrationPara.AveYdata < 3.5f))
                {
                    CalibrationPara.Start = 1;
                    CalibrationPara.SteeringAngle = Message_VehicleMsg.SteeringAngle;
                    CalibrationPara.Velocity = Message_VehicleMsg.Velocity;
                }
                else
                {
                    CalibrationPara.Start = 2;//单帧点符合但是栅栏横向距离不符合
                    Adaptive_CalibrationClear();
                }
            }
            else
            {
                 CalibrationPara.Start = 2;     //单帧点不符合
                 Adaptive_CalibrationClear(); // 标定放弃
            }
        }
        else
        {
            if((fabs(Message_VehicleMsg.SteeringAngle - CalibrationPara.SteeringAngle) < 10)
               && (fabs(Message_VehicleMsg.Velocity - CalibrationPara.Velocity) < 3))  //3*3.6 kmh
            {
                Start_num = 0;
                for(i=0; i < gNumPoints; i++)
                {
                    if ((PeakList[i].vector.range > CalibrationRangeMin) 
                    && (PeakList[i].vector.range < CalibrationRangeMax) 
                    && ((PeakList[i].vector.azimuth * 180 / PI) < 0.0f) 
                    && ((PeakList[i].vector.azimuth * 180 / PI) > -35.0f) 
                    && (PeakList[i].snr >= Calibration_MinRCs)) 
                    {
                        if((((RadarPara.InstallPosition == INSTALL_LEFT_FRONT)||(RadarPara.InstallPosition == INSTALL_RIGHT_FRONT))&&(PeakList[i].vector.doppler < 0))
                                ||(((RadarPara.InstallPosition != INSTALL_LEFT_FRONT)&&(RadarPara.InstallPosition != INSTALL_RIGHT_FRONT))&&(PeakList[i].vector.doppler > 0)))
                        {
                            if((RadarPara.InstallPosition == INSTALL_LEFT_FRONT)||(RadarPara.InstallPosition == INSTALL_RIGHT_FRONT))
                            {
                                temp_speed_gap = fabs(PeakList[i].vector.doppler/cos((RadarInstallAngle + (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180)  + Message_VehicleMsg.Velocity);
                            }
                            else
                            {
                                temp_speed_gap = fabs(PeakList[i].vector.doppler/cos((RadarInstallAngle + (PeakList[i].vector.azimuth * 180 / PI)) * PI / 180)  - Message_VehicleMsg.Velocity);
                            }

                            if(Message_VehicleMsg.Velocity < 4.1f)
                            {
                                Calibration_flag = temp_speed_gap < Message_VehicleMsg.Velocity*0.2f;
                            }
                            else if((Message_VehicleMsg.Velocity >= 4.1f) && (Message_VehicleMsg.Velocity < 8.3f))
                            {
                                Calibration_flag = temp_speed_gap < Message_VehicleMsg.Velocity*0.12f;
                            }
                            else
                            {
                                Calibration_flag = temp_speed_gap < Message_VehicleMsg.Velocity*0.1f;
                            }
                            if(Calibration_flag)
                         {
                                CalibrationPara.Xdata[CalibrationPara.DataNum] = PeakList[i].vector.range * cos((RadarInstallAngle + (PeakList[i].vector.azimuth * 180 / PI))*PI/180);
                                CalibrationPara.Ydata[CalibrationPara.DataNum] = PeakList[i].vector.range * sin((RadarInstallAngle + (PeakList[i].vector.azimuth * 180 / PI))*PI/180);
                                if((CalibrationPara.DataNum < CalibrationTime*Timeframe) && (CalibrationPara.Ydata[CalibrationPara.DataNum] > (CalibrationPara.AveYdata - 0.7f))
                                    && (CalibrationPara.Ydata[CalibrationPara.DataNum] < (CalibrationPara.AveYdata + 0.7f)))   //1
                                {
                                    YD_XD_writing(CalibrationPara.Ydata[CalibrationPara.DataNum],CalibrationPara.Xdata[CalibrationPara.DataNum]);

                                    CalibrationPara.DataNum++;
                                    Start_num++;

                                }
                            }
                        }
                    }
                }
                CalibrationPara.Frame++;

                if(Start_num <= (Timeframe/2 + 1))
                {
                    CalibrationPara.FalseFrame++;

                    if(CalibrationPara.FalseFrame > (CalibrationTime/18))
                    {
                        Adaptive_CalibrationClear();
                    }
                }
                else
                {
                    CalibrationPara.FalseFrame = 0;
                }
            }
            else
            {
                Adaptive_CalibrationClear();
            }
        }
    }
    else
    {
        CalibrationPara.Error_Number++;
        if(CalibrationPara.Error_Number > 500)
        {
            CalibrationPara.Step = 3;
            CalibrationPara.Master_Result = 2;
        }

        Adaptive_CalibrationClear();
    }
}

void Adaptive_CalibrationPolyFit(void)
{
    float32_t  sum_x2 = 0;
    float32_t  sum_y = 0;
    float32_t  sum_x = 0;
    float32_t  sum_xy = 0;
    uint16_t   i=0;
    float32_t a;
    float32_t b;
  

    for (i = 0; i < CalibrationPara.DataNum; i++)
    {
        sum_x2 = sum_x2 + CalibrationPara.Xdata[i]*CalibrationPara.Xdata[i];
        sum_y = sum_y + CalibrationPara.Ydata[i];
        sum_x = sum_x + CalibrationPara.Xdata[i];
        sum_xy = sum_xy + CalibrationPara.Xdata[i]*CalibrationPara.Ydata[i];
    }

    a = (CalibrationPara.DataNum*sum_xy - sum_x*sum_y)/(CalibrationPara.DataNum*sum_x2 - sum_x*sum_x);
    b = (sum_x2*sum_y - sum_x*sum_xy)/(CalibrationPara.DataNum*sum_x2-sum_x*sum_x);

    CalibrationPara.Adap_B = b;
    CalibrationPara.Adap_A = a;
    CalibrationPara.Adap_Angle = atan(a)*180/PI;

    printf("CalibrationPara.AveYdata = %f\n",CalibrationPara.AveYdata);
    printf("/********************************This_is_result*******************************/\n");
    printf("/*****************************************************************************/\n");
    printf("/*****************************Adap_Angle = %f**************************/\n",CalibrationPara.Adap_Angle);
    printf("/*****************************************************************************/\n");
    printf("/*****************************************************************************/\n");
    printf("/*****************************************************************************/\n");
    CAL_MODE = CALIBRATION_EXIT;

}

void Adaptive_CalibrationFinish(void)
{
    float32_t TmpLinearAngle = 0;
    //CAN_DCANData  TxMessage;
    if ((CalibrationPara.Adap_B > 0.5f) && (CalibrationPara.Adap_B < 3.5f)) //(b > 1.3f) && (b < 2.8f))
    {
        if (CalibrationPara.Step == 1)
        {
            TmpLinearAngle = -CalibrationPara.Adap_Angle;
        }

        if (fabs(TmpLinearAngle) < 5)
        {
            if (CalibrationPara.Step == 1)
            {
                //printf("This is result of Adaptive Calibration\r\n");
                //printf("RadarPara.FarHorizontalOffsetAngle = %f\r\n", TmpLinearAngle);
                
                RadarPara.FarHorizontalOffsetAngle= TmpLinearAngle;
                CalibrationPara.Step = 3;          // 标定结束
                CalibrationPara.Master_Result = 1; // 标定成功
                //CalibrationPara.Driving_Profile = 0x00;
                //Config_WriteAllConfig();
                // TxMessage.dataLength  = 8U;
                // TxMessage.msgLostFlag = 0;
                // TxMessage.msgData[0]  = 0X00;
                // TxMessage.msgData[1]  = 1;
                // TxMessage.msgData[2]  = 1;
                // TxMessage.msgData[3]  = 1;
                // TxMessage.msgData[4]  = 1;
                // TxMessage.msgData[5]  = 1;
                // TxMessage.msgData[6]  = 1;
                // TxMessage.msgData[7]  = 0;
                // CAN1_Send_Data(0x711, &TxMessage.msgData, TxMessage.dataLength);
            }

        }
        else
        {
            Adaptive_CalibrationClear(); 
             
        }
    }
}
