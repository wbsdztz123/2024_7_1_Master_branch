#include "calibration_common.h"

cal_extern_para_t calibration_extern_para = {0};
adaptive_status_out_t adap_status_out = {0};
offline_status_out_t offline_status_out = {0};
calibration_adaptive_result_content_t adap_result = {0};




//cal_result_t cal_result = {0};

// offline_status_out_t *offline_cal_get_status(void)
// {
//     return &offline_status_out;
// }


int switching_mode_debug()
{
    #define SWITCHOVER_FAILURE_SUCCESS 0
    #define SWITCHOVER_FAILURE -1

     int result = SWITCHOVER_FAILURE;
    // /******************flow debug*******************/
    // _atomic_store(RadarPara.WorkMode, mode);
    // const radar_work_mode_t work_mode_cal = _atomic_load(RadarPara.WorkMode, radar_work_mode_t);
    // if (work_mode_cal == mode) {
        result = SWITCHOVER_FAILURE_SUCCESS;
    

    return result;
    /******************flow debug*******************/
}

bool GetDtcCalOutOfRange(void)
{
    float32_t HorizontalAngle_temp = 0, VerticalAngle_temp = 0;
    // HorizontalAngle_temp = RadarPara.FarHorizontalOffsetAngle + RadarPara.FarHorizontalAdptiveAngle;
    // VerticalAngle_temp   = RadarPara.FarVerticalAdptiveAngle + RadarPara.FarVerticalOffsetAngle;

    if ((fabs(HorizontalAngle_temp) > Calibration_Tolerance)
        || (fabs(VerticalAngle_temp) > Calibration_elevTolerance)) {
        return FALSE;
    } else {
        return TRUE;
    }
}

float GetCalHorizontalAngle(void)
{
    return 0.0f;
}

float GetCalVertiAngle(void)
{
    return 0.0f;
}


/**************************************offline****************************************/
// void offline_calibrate_func(const or_point_cloud_format_t *PeakList)
// {
//     offline_flow_control_func(PeakList);
// }


// const offline_status_out_t *offline_status_out_func(void)
// {
//     //offline_real_time_status_set_func();
//     return &offline_status_out;
// }

/* 函数名: GetOfflineCalStatus()
 * 描述：获取下线校准状态
 * 返回值：StatusArray
 *        byte 0: 例程状态 (0x00=标定成功, 0x01=标定进行中, 0x02=标定超时, 0x03=垂直角度偏差过大, 
 *                         0x04=水平角度偏差过大, 0x05=目标丢失, 0x06=写入NVM失败)
 *        byte 1~2: 水平角度误差值 (Resolution=0.01, Offset=0) 
 *                  Data[1] = (HorizontalAngle >> 8) & 0xFF;
 *                  Data[2] = HorizontalAngle & 0xFF;
 *        byte 3~4: 垂直角度误差值 (Resolution=0.01, Offset=0)
 *                  Data[3] = (VerticalAngle >> 8) & 0xFF;
 *                  Data[4] = VerticalAngle & 0xFF;
 */
// void GetOfflineCalStatus(uint8_t *StatusArray)
// {
//     int16_t TempFarHorizontalOffsetAngle, TempFarVerticalOffsetAngle;
//     offline_real_time_status_set_func();
    
//     TempFarHorizontalOffsetAngle = (int16_t)(offline_status_out.horizontal_angle_deviation * 100);
//     TempFarVerticalOffsetAngle   = (int16_t)(offline_status_out.vertical_angle_deviation * 100);

//     *StatusArray       = (uint8_t)offline_status_out.Calibration_Status;
//     *(StatusArray + 1) = (uint8_t)((TempFarHorizontalOffsetAngle >> 8) & 0xFF);
//     *(StatusArray + 2) = (uint8_t)(TempFarHorizontalOffsetAngle & 0xFF);
//     *(StatusArray + 3) = (uint8_t)((TempFarVerticalOffsetAngle >> 8) & 0xFF);
//     *(StatusArray + 4) = (uint8_t)(TempFarVerticalOffsetAngle & 0xFF);
// }


// bool offline_start_func(void)
// {
//     #define OFFLINE_START_SUCCESS 0
//     #define OFFLINE_START_FAIL 1

//     if(offline_start())
//     {
//         return OFFLINE_START_SUCCESS;
//     }else{
//         return OFFLINE_START_FAIL;
//     }
// }
/**************************************offline****************************************/




/**************************************adaptive**************************************/
void adaptive_calibrate_func(const or_point_cloud_format_t *PeakList)
{
    adaptive_flow_control_func(PeakList);
}



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
void GetAdaptiveCalStatus(uint8_t *StatusArray)
{
    int16_t adaptive_angle_h, adaptive_angle_v;
    adaptive_real_time_status_set_func();
    
    adaptive_angle_h = (int16_t)(adap_status_out.output_adapt_angle_h * 100);
    adaptive_angle_v  = (int16_t)(adap_status_out.output_adapt_angle_v * 100);

    *StatusArray       = (uint8_t)adap_status_out.output_result;
    *(StatusArray + 1) = (uint8_t)adap_status_out.output_progress;
    *(StatusArray + 2) = (uint8_t)adap_status_out.output_cause_of_failure;
    *(StatusArray + 3) = (uint8_t)((adaptive_angle_h >> 8) & 0xFF);
    *(StatusArray + 4) = (uint8_t)(adaptive_angle_h & 0xFF);
    *(StatusArray + 5) = (uint8_t)((adaptive_angle_v >> 8) & 0xFF);
    *(StatusArray + 6) = (uint8_t)(adaptive_angle_v & 0xFF);
}




/* 函数名: AdaptiveCalStart()
 * 描述：售后校准开始
 * 返回值：0x00=标定例程成功开启
 *        0x01=标定正在运行中
 *        0x02=标定失败
 */
bool adaptive_start_func(void)
{
    #define ADAPTIVE_START_SUCCESS 0x00
    #define ADAPTIVE_START_FAIL    0x02
    if(adaptive_start())
    {
        return ADAPTIVE_START_SUCCESS;
    }else
    {
        return ADAPTIVE_START_FAIL;
    }
}

calibration_adaptive_result_content_t* get_adapt_status(void)
{
    return &adap_result;
}










/* 函数名: SaveOfflineCalStatus_DID_0x4901()
 * 描述：保存下线校准状态
 * 输入: StatusArray
 *        byte 0: 下线校准状态 (0x00：标定未完成, 0x01：标定已完成)
 *        byte 1~5: 保留




/**************************************adaptive**************************************/
