
/****************************************************************************
 *                        File: adaptive_calibration.c                      *
 *                        @sjw20210713                                           *
 ****************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "adaptive_calibration.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
//#include "../user_app/tracker/OutPut.h"
extern double  gModelParamK;

#define LOG_INFO _3sradar_tui_print
#define LOG_ERROR _3sradar_tui_print
#define LOG_DEBUG _3sradar_tui_print
#define LOG_WARN _3sradar_tui_print

// extern int8_t Out_Put_Vehicle_List(uint8_t i);

calib_adapt_format_t      adapt_format ={0};
Point                     dataset[MAX_POINTS]; // 存储单帧所有数据点
int                       total_points = 0;    // 实际数据点数量




static void adaptive_data_clear(void);
static int32_t adaptive_wokemode_check(void);
static int32_t adaptive_init(void);
static bool body_posture_detection(void);
//static uint8_t CAL_Target_Filtering(const or_point_cloud_term_t *term);
uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList, uint8_t i);
static uint8_t Range_Density_Analysis(const or_point_cloud_format_t *PeakList);
static void calibration_init_process(const or_point_cloud_format_t *PeakList);
static bool Target_Filtering_Check(const or_point_cloud_term_t *term, uint8_t zone);
static void update_calibration_progress(uint8_t step, uint8_t offset);
static void activate_calibration(void);
static bool is_valid_calibration_range(float y_data);
void calibration_data_collection(const or_point_cloud_format_t *PeakList);
static void store_calibration_data(Point3D point, float range);
static void adaptive_data_volume_judg(void);
void calibration_adaptive_polyfit(void);
void calibration_adaptive_finish(void);
static void calibration_result_write(void);
static void calibration_result_read(void);
static void calibration_adaptive_end(void);
static void adaptive_result_set(calibration_adaptive_result_kind_t result,calibration_adaptive_error_kind_t error_type,float angle_h,float angle_v);







bool adaptive_start()
{
        memset(&adapt_format, 0, sizeof(calib_adapt_format_t));

    //memset(&offline_status_out, 0, sizeof(offline_status_out));
    if (1) {
        adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_START;
        adapt_format.adaptve_calibrationpara.adapt_result = CALIBRATION_ADAPTIVE_IN_PROGRESS;
        return true;
    }
    return false;

}


/* 函数名：AdaptiveCalTimeoutProc()
 * 描述：售后校准超时回调函数
 * 输入：clock: clock object
 *       argv: 传入参数
 * 返回值：NA
 */

// static void AdaptiveCalTimeoutProc(ClockP_Object *clock, void *argv)
// {

//     //adaptive_result_set(CALIBRATION_ADAPTIVE_IS_FAIL,CALIBRATION_ADAPTIVE_IN_TIMEOUT,adapt_format.adaptve_calibrationpara.apat_angle_h,adapt_format.adaptve_calibrationpara.apat_angle_v);
// }


/* 函数名：AdaptiveCalClockLaunch()
 * 描述：启动售后校准超时clock
 * 输入：period: 超时时长(usec)-
 * 返回值：NA
 * 备注：停止并销毁clock示例代码如下，
 *      ClockP_stop(adaptive_cal_clock);
 *      ClockP_destruct(adaptive_cal_clock);
 */
void AdaptiveCalClockLaunch(uint64_t period)
{
    
    // ClockP_Params clock_params;

    // ClockP_Params_init(&clock_params);
    // clock_params.timeout  = ClockP_usecToTicks(period);
    // clock_params.start    = 1;
    // clock_params.callback = &AdaptiveCalTimeoutProc;
    // clock_params.args     = NULL;

    // ClockP_construct(&adaptive_cal_clock, &clock_params);
    
}

void adaptive_real_time_status_set_func(void)
{
    adap_status_out.output_result = adapt_format.adaptve_calibrationpara.adapt_result;
    adap_status_out.output_progress = adapt_format.adaptve_calibrationpara.adaptive_PB;
    adap_status_out.output_cause_of_failure = adapt_format.adaptve_calibrationpara.errType;
    if((adapt_format.adaptve_calibrationpara.errType == CALIBRATION_ADAPTIVE_IS_SUCCESSFUL) || (adapt_format.adaptve_calibrationpara.errType == CALIBRATION_ADAPTIVE_IS_ANGLE_OVERSHOOT))
    {
        adap_status_out.output_adapt_angle_h = adapt_format.adaptve_calibrationpara.apat_angle_h;
        adap_status_out.output_adapt_angle_v = adapt_format.adaptve_calibrationpara.apat_angle_v;
    }
}








void Calibration_Progress(uint8_t pace)
{
    adapt_format.adaptve_calibrationpara.TEMP_PB     = max(pace, adapt_format.adaptve_calibrationpara.TEMP_PB);
    adapt_format.adaptve_calibrationpara.adaptive_PB = max(adapt_format.adaptve_calibrationpara.TEMP_PB, adapt_format.adaptve_calibrationpara.adaptive_PB);
}

// 计算两个点之间的欧氏距离
double calculate_distance(const Point *a, const Point *b)
{
    double sum = 0.0;
    for (int i = 0; i < DIMENSIONS; i++) {
        sum += pow(a->coords[i] - b->coords[i], 2);
    }
    return sqrt(sum);
}
// 查找某个点的ε邻域内的所有点（返回值为邻域点数量）
int find_neighbors(int point_idx, double eps, int *neighbors)
{
    int count = 0;
    for (int i = 0; i < total_points; i++) {
        if (calculate_distance(&dataset[point_idx], &dataset[i]) <= eps) {
            neighbors[count++] = i;
        }
    }
    return count;
}
// DBSCAN算法主函数
void dbscan(double eps, int min_pts)
{
    int cluster_id = 0;
    for (int i = 0; i < total_points; i++) {
        if (dataset[i].visited) continue;
        dataset[i].visited = true;

        int neighbors[MAX_POINTS];
        int num_neighbors = find_neighbors(i, eps, neighbors);

        if (num_neighbors < min_pts) {
            // 标记为噪声（后续可能被归入其他簇）
            dataset[i].cluster_id = -1;
        } else {
            // 创建新簇
            cluster_id++;
            dataset[i].cluster_id = cluster_id;

            // 扩展簇
            for (int j = 0; j < num_neighbors; j++) {
                int neighbor_idx = neighbors[j];
                if (!dataset[neighbor_idx].visited) {
                    dataset[neighbor_idx].visited = true;
                    int sub_neighbors[MAX_POINTS];
                    int sub_num = find_neighbors(neighbor_idx, eps, sub_neighbors);
                    if (sub_num >= min_pts) {
                        // 将新邻域点加入处理队列（这里直接扩展数组）
                        for (int k = 0; k < sub_num; k++) {
                            neighbors[num_neighbors++] = sub_neighbors[k];
                        }
                    }
                }
                // 如果点未被归类到任何簇，则加入当前簇
                if (dataset[neighbor_idx].cluster_id == -1 || dataset[neighbor_idx].cluster_id == 0) {
                    dataset[neighbor_idx].cluster_id = cluster_id;
                }
            }
        }
    }
}




void adaptive_result_set(calibration_adaptive_result_kind_t result,calibration_adaptive_error_kind_t error_type,float angle_h,float angle_v)
{
    // uint8_t StatusArray[9] = {0};
    // adapt_format.adaptve_calibrationpara.adapt_result = result; // 雷达标定结果为失败
    // adapt_format.adaptve_calibrationpara.errType = error_type; // 角度偏差过大

    // adap_result.last_result = result;
    // adap_result.adapt_angle_h = angle_h; // 水平角度误差值
    // adap_result.adapt_angle_v = angle_v; // 垂直角度误差值

    // RadarPara.FarHorizontalAdptiveAngle = adap_result.adapt_angle_h;
    // RadarPara.FarVerticalAdptiveAngle = adap_result.adapt_angle_v;

    // StatusArray[0] = result;
    // StatusArray[1] = error_type;
    // StatusArray[2] = (((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) >> 8) & 0xFF;
    // StatusArray[3] = ((int16_t)(RadarPara.FarHorizontalAdptiveAngle * 100)) & 0xFF;
    // StatusArray[4] = (((int16_t)(RadarPara.FarVerticalAdptiveAngle * 100)) >> 8) & 0xFF;
    // StatusArray[5] =((int16_t)(RadarPara.FarVerticalAdptiveAngle * 100)) & 0xFF;
    // //SaveAdaptiveCalStatus_DID_0x4902(StatusArray);
    // //Config_WriteAllConfig();
    // calibration_result_write();
}
///////////////////////新框架///////////////////////////////////////////////////

static void adaptive_data_clear(void)
{
    uint16_t i;
    adapt_format.adaptve_calibrationpara.SteeringAngle = 0;
    adapt_format.adaptve_calibrationpara.YawRate = 0;
    adapt_format.adaptve_calibrationpara.Velocity = 0;
    adapt_format.adaptve_calibrationpara.Frame = 0;
    adapt_format.adaptve_calibrationpara.DataNum = 0;
    adapt_format.adaptve_calibrationpara.FalseFrame = 0;
    adapt_format.adaptve_calibrationpara.AveYdata = 0;
    adapt_format.adaptve_calibrationpara.Adap_A = 0;
    adapt_format.adaptve_calibrationpara.Adap_B = 0;
    adapt_format.adaptve_calibrationpara.Adap_Angle = 0;
    adapt_format.adaptve_calibrationpara.errType = 0;
    adapt_format.adaptve_calibrationpara.TEMP_PB = 0;
    adapt_format.adaptve_calibrationpara.apat_angle_h = 0;
    adapt_format.adaptve_calibrationpara.apat_angle_v = 0;

    adapt_format.adaptve_calibrationpara.driving_profile = 0;

    for (i = 0; i < SINGLE_DATA_AMOUNT; i++)
    {

        adapt_format.adaptve_calibrationpara.xdata[i] = 0;
        adapt_format.adaptve_calibrationpara.ydata[i] = 0;
        adapt_format.adaptve_calibrationpara.rangdata[i] = 0;
        adapt_format.adaptve_calibrationpara.elevdata[i] = 0;
    }
    adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_BODY_POSTURE_DETECTION;
}


static int32_t adaptive_wokemode_check(void)
{
    int32_t                 check_result  = ADAPTIVE_WORKMODE_CHECK_NO_SUCCESS;
    // const radar_work_mode_t work_mode_now = _atomic_load(RadarPara.WorkMode, radar_work_mode_t); //当前工作模式API
    // if (work_mode_now != WORKIN_MODE_ADAPTIVE_CALIBRATION) {
         check_result = ADAPTIVE_WORKMODE_CHECK_SUCCESS;
    // }

    adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_INIT;
    return check_result;
}

static int32_t adaptive_init(void)
{
    int32_t init_result = ADAPTIVE_INIT_SUCCESS;
    //memset(dataset, 0, sizeof(dataset));
    calibration_extern_para.cal_install_angle = 0;
    calibration_extern_para.cal_installation = INSTALL_FRONT;
    adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_BODY_POSTURE_DETECTION;
    return init_result;
}



/**
 * @brief 车身姿态有效性检测
 * @return true - 姿态有效, false - 姿态无效
 * @note 各阈值参数说明:
 *   - 速度范围: (MinVel, MaxVel) 开区间
 *   - 转向角: 绝对值小于 MaxSteeringAngle
 *   - 道路曲率: 绝对值大于 MinCurveRadius
 *   - 横摆角速度: 绝对值小于 MaxYawRate
 */
static bool body_posture_detection(void)
{
    //printf("body_posture_detection");
    #define BODY_POSTERE_DETECTION 2
    // ================= 输入有效性验证 =================
    const Message_VehicleMsgS *pMsg = &Message_VehicleMsg;

    // 验证浮点数值有效性
    if (!isfinite(pMsg->Velocity) || !isfinite(pMsg->SteeringAngle) || !isfinite(pMsg->CurveRadius)
        || !isfinite(pMsg->YawRate)) {
        return false;
    }

    // ================= 参数预处理 =================
    const float velocity_mps = fabsf(KMH_TO_MS(pMsg->Velocity)); // km/h -> m/s
    const float abs_steering = fabsf(pMsg->SteeringAngle);
    const float abs_curvature = fabsf(pMsg->CurveRadius);
    const float abs_yaw_rate  = fabsf(pMsg->YawRate);

    // ================= 分层条件检测 =================
    const bool is_velocity_valid = (velocity_mps > ADAPTIVE_MIN_VELOCITY) && (velocity_mps < ADAPTIVE_MAX_VELOCITY);

    const bool is_steering_valid = (abs_steering < ADAPTIVE_MAX_STEERING_ANGLE);

    const bool is_curvature_valid = (abs_curvature > ADAPTIVE_MIN_CURVERADIUS); // 修正变量名歧义

    const bool is_yawrate_valid = (abs_yaw_rate < ADAPTIVE_MAX_YAWRATE);

    // ================= 诊断日志输出 =================
    if (!is_velocity_valid) {
      //  LOG_WARN("Velocity out of range: %.1f m/s (Req: %.1f~%.1f)", velocity_mps, ADAPTIVE_MIN_VELOCITY,
      //     ADAPTIVE_MAX_VELOCITY);
    }
    if (!is_steering_valid) {
      //  LOG_WARN("Steering angle overflow: %.1f deg > %.1f", abs_steering, ADAPTIVE_MAX_STEERING_ANGLE);
    }
    if (!is_curvature_valid) {
      //  LOG_WARN("Curve radius too small: %.1f m < %.1f", abs_curvature, ADAPTIVE_MIN_CURVERADIUS);
    }
    if (!is_yawrate_valid) {
      //  LOG_WARN("Yaw rate overflow: %.2f rad/s > 0.8", abs_yaw_rate);
    }
    update_calibration_progress(PROGRESS_STEP,BODY_POSTERE_DETECTION);
    // ================= 综合判断 =================
    const bool is_posture_valid = is_velocity_valid && is_steering_valid && is_curvature_valid && is_yawrate_valid;

    return is_posture_valid;

    //return true;
}


/**
 * @brief 校准目标过滤函数
 * @param PeakList 点云数据结构体指针
 * @param i 目标点索引
 * @return uint8_t 1=有效目标, 0=无效目标
 * @note 执行流程:
 *   1. 输入有效性检查
 *   2. 基础几何条件筛选
 *   3. 安装位置相关多普勒验证
 *   4. 动态速度阈值计算
 */


//static uint8_t CAL_Target_Filtering(const or_point_cloud_term_t *term)
uint8_t CAL_Target_Filtering(const or_point_cloud_format_t *PeakList, uint8_t i)
{
    uint8_t Calibration_flag = 0;
    float32_t temp_speed_gap = 0xff;
    //printf("CAL_Target_Filtering\n");
    if (PeakList->term[i].range > X_DISTANCE_MIN 
        && PeakList->term[i].range < X_DISTANCE_MAX
        && (PeakList->term[i].azimuth * 180 / PI) > 0.0f
        && (PeakList->term[i].azimuth * 180 / PI) < 55.0f
        && PeakList->term[i].snr >= ADAPTIVE_MIN_RCS)
    {
        //bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_LEFT_FRONT || RadarPara.InstallPosition == INSTALL_RIGHT_FRONT);
        bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_FRONT);
        bool isDopplerNegative = PeakList->term[i].doppler < 0.0f;

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
    // =============== 输入有效性验证 ===============
    // if (!term) {// || i >= point_count
    //     //LOG_ERROR("Invalid input: PeakList=%p, index=%u", PeakList, i);
    //     return 0;
    // }

    // // =============== 基础几何条件筛选 ===============
    // /* 计算方位角度并验证范围 */
    // const float azimuth_deg   = RAD_TO_DEG(term->azimuth);
    // const bool  is_geom_valid = (term->range > X_DISTANCE_MIN ) && (term->range < X_DISTANCE_MAX )
    //                            && (azimuth_deg > AZIMUTH_ANGLE_MIN) && (azimuth_deg < AZIMUTH_ANGLE_MAX)
    //                            && (term->snr >= ADAPTIVE_MIN_RCS);

    // if (!is_geom_valid) return 0;

    // // =============== 安装位置判断 ===============
    // const bool is_front_install = ((calibration_extern_para.cal_installation == INSTALL_FRONT)
    //                               || (calibration_extern_para.cal_installation == INSTALL_LEFT_FRONT)
    //                               || (calibration_extern_para.cal_installation == INSTALL_RIGHT_FRONT));
    // const bool is_doppler_negative = (term->doppler < 0.0f);

    // /* 安装位置与多普勒符号匹配检查 */
    // if ((is_front_install && !is_doppler_negative) || (!is_front_install && is_doppler_negative)) {
    //     return 0;
    // }

    // // =============== 动态阈值计算 ===============
    // const float speed_ms        = KMH_TO_MS(Message_VehicleMsg.Velocity); //API
    // float       threshold_coeff = SPEED_THRESHOLD_LOW;                    // 默认阈值系数

    // /* 速度分段阈值选择 */
    // if (speed_ms >= SPEED_SEGMENT_HIGH) {
    //     threshold_coeff = SPEED_THRESHOLD_HIGH;
    // } else if (speed_ms >= SPEED_SEGMENT_LOW) {
    //     threshold_coeff = SPEED_THRESHOLD_LOW;
    // }

    // // =============== 速度偏差计算 ===============
    // float azimuth_calib_deg = 0;

    // if (calibration_extern_para.cal_installation == INSTALL_FRONT
    //     || calibration_extern_para.cal_installation == INSTALL_BACK) {
    //         azimuth_calib_deg = term->azimuth;
    // } else {
    //     azimuth_calib_deg = DEG_TO_RAD(calibration_extern_para.cal_install_angle) - term->azimuth;
    // }
    // //Out_Put_Vehicle_List(0x04);
    // const float cos_value = cosf(azimuth_calib_deg);
    // if (fabsf(cos_value) < EPSILON) return 0;
    // /* 多普勒速度补偿计算 */
    // const float expected_doppler = (is_front_install ? -speed_ms : speed_ms);
    // const float speed_gap        = fabsf((term->doppler / cos_value) - expected_doppler);
    // //Out_Put_Vehicle_List(0x05);
    // // =============== 最终判定 ===============
    // const bool is_speed_valid = (speed_gap < (speed_ms * threshold_coeff));
    //  //LOG_DEBUG("Target: speed_gap=%.3f, threshold=%.3f, valid=%d", speed_gap, speed_ms * threshold_coeff,
    //            //is_speed_valid;
    // return is_speed_valid ? 1 : 0;

}


/**
 * @brief 统计有效目标在Y轴方向的分布密度
 * @param PeakList 点云数据结构体指针
 * @return 区域标识: 
 *          1 = 低密度区(0-4m)目标更多
 *          2 = 高密度区(4-8m)目标更多
 *          0 = 无效输入或无有效目标
 */
static uint8_t Range_Density_Analysis(const or_point_cloud_format_t *PeakList)
{
    // ================= 输入有效性验证 =================
    if (!PeakList || PeakList->point_count == 0) {
       // LOG_ERROR("Invalid input: PeakList=%p", PeakList);
        return 0;
    }

    // ================= 初始化统计计数器 =================
    uint16_t count_y_low  = 0; // Y轴0-4米区域计数
    uint16_t count_y_high = 0; // Y轴4-8米区域计数
    const or_point_cloud_term_t *term = NULL; // 当前目标点指针
    // ================= 主处理循环 =================
    
    for (uint32_t i = 0; i < PeakList->point_count; ++i) {

        term = &PeakList->term[i];

        // 阶段1: 快速过滤无效距离点
        if (term->range <= RANGE_MIN || term->range >= RANGE_MAX) {
            continue;
        }

        // 阶段2: 目标有效性验证
        
        const uint8_t is_valid_target = CAL_Target_Filtering(PeakList,i);
        if (!is_valid_target) {
            continue;
        }
        //printf("is_valid_target = %d\n", is_valid_target);
        //Out_Put_Vehicle_List(0x06);
        // 阶段3: 坐标转换计算
        float azimuth_deg = 0;
        if (calibration_extern_para.cal_installation == INSTALL_FRONT
            || calibration_extern_para.cal_installation == INSTALL_BACK) {
                azimuth_deg = term->azimuth;
        } else {
                azimuth_deg = DEG_TO_RAD(calibration_extern_para.cal_install_angle) - term->azimuth;
        }
        //const float azimuth_deg = DEG_TO_RAD(calibration_extern_para.cal_install_angle) - term->azimuth; /* 根据安装位置调整方位角计算偏移量 */
        //const float azimuth_deg = RAD_TO_DEG(azimuth_deg_temp);
        //Out_Put_Vehicle_List(0x07);
        const float y_offset = term->range * sinf(azimuth_deg);
        //printf("y_offset = %f\n", y_offset);
        //Out_Put_Vehicle_List(0x08);
        // 阶段4: Y轴区域统计
        if (y_offset > 0.0f && y_offset < Y_SEGMENT_LOW) {
            ++count_y_low;
        } else if (y_offset >= Y_SEGMENT_LOW && y_offset < Y_SEGMENT_HIGH) {
            ++count_y_high;
        }
    }

    // ================= 密度比较决策 =================
    //LOG_INFO("Y轴区域统计: 低区=%u, 高区=%u", count_y_low, count_y_high);

    if (count_y_low > count_y_high) {
        return DENSE_ZONE_LOW;
    } else if (count_y_high > 0) { // 避免高区为0时返回2
        return DENSE_ZONE_HIGH;
    }
    return 0; // 无有效目标
}


/**
 * @brief 标定初始化处理流程
 * @param PeakList 点云数据结构体指针
 */
static void calibration_init_process(const or_point_cloud_format_t *PeakList)
{
    
    //printf("calibration_init_process");
    // ================= 初始化校验 =================
    if (!PeakList) {
        return;
    }    
    // printf("Message_VehicleMsg.Velocity = %f\n", Message_VehicleMsg.Velocity);
    // printf("Message_VehicleMsg.SteeringAngle = %f\n", Message_VehicleMsg.SteeringAngle);
    // printf("Message_VehicleMsg.CurveRadius = %f\n", Message_VehicleMsg.CurveRadius);
    // printf("Message_VehicleMsg.YawRate = %f\n", Message_VehicleMsg.YawRate);

    //printf("peaklist->point_count = %d\n", PeakList->point_count);
    // printf("peaklist->term[0].range = %f\n", PeakList->term[0].range);
    // printf("peaklist->term[0].azimuth = %f\n", PeakList->term[0].azimuth);    
    // printf("peaklist->term[0].snr = %f\n", PeakList->term[0].snr);
    // printf("peaklist->term[0].doppler = %f\n", PeakList->term[0].doppler);
    
    // ================= 环境感知预处理 =================
    adapt_format.adaptve_calibrationpara.AveYdata = 0.0f;
    
    const uint8_t range_zone = Range_Density_Analysis(PeakList); // 重构后的区域分析函数

    // ================= 动态边界设置 =================
    float lower_bound, upper_bound;
    switch (range_zone) {
    case DENSE_ZONE_HIGH: // 4.01-8.0米高密度区
        lower_bound = 4.01f;
        upper_bound = 8.0f;
        break;
    case DENSE_ZONE_LOW: // 0-4.01米低密度区
        lower_bound = 0.0f;
        upper_bound = 4.01f;
        break;
    default: // 无效区域处理
        lower_bound = 0.0f;
        upper_bound = 15.0f;
        adaptive_data_clear();

        return;
    }

    // ================= 有效数据收集 =================
    uint32_t valid_count = 0;
    float    y_data_sum  = 0.0f;
    const or_point_cloud_term_t *term        = NULL ; // 当前目标点指针
    
    for (uint32_t i = 0; i < PeakList->point_count; ++i) {
        term = &PeakList->term[i];
        // 阶段1: 快速过滤
        if (!CAL_Target_Filtering(PeakList,i)) { // 封装过滤逻辑
            continue;
        }
        // 阶段2: 坐标计算
        //const float                  azimuth_deg = RAD_TO_DEG(term->azimuth);
        float azimuth_calib_deg = 0;

        if (calibration_extern_para.cal_installation == INSTALL_FRONT
            || calibration_extern_para.cal_installation == INSTALL_BACK) {
                azimuth_calib_deg = term->azimuth;
        } else {
            azimuth_calib_deg = DEG_TO_RAD(calibration_extern_para.cal_install_angle) - term->azimuth;
        }
        //Out_Put_Vehicle_List(0x14);
        const float y_offset  =  term->range * sinf(azimuth_calib_deg);
        //Out_Put_Vehicle_List(0x0A);
        // 阶段3: 区域验证
        if (y_offset > lower_bound && y_offset < upper_bound) {
            y_data_sum += y_offset;
            ++valid_count;
        }
    }

    // ================= 标定启动决策 =================
    //Out_Put_Vehicle_List(0x0B);
    update_calibration_progress(PROGRESS_STEP, INITIAL_PROGRESS);
//printf("valid_count  = %d\n", valid_count);
    if (valid_count >= CALIBRATION_MIN_SAMPLES) {
        adapt_format.adaptve_calibrationpara.AveYdata = y_data_sum / valid_count;
        if (is_valid_calibration_range(adapt_format.adaptve_calibrationpara.AveYdata)) {
            activate_calibration();
            //Out_Put_Vehicle_List(0x0C);
            update_calibration_progress(PROGRESS_STEP, FINAL_PROGRESS_OFFSET);
            adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_DATUM_SELECTION;  // 进入数据收集阶段
        } else {
            adaptive_data_clear(); // 标定放弃
        }
    } else {
        adaptive_data_clear(); // 标定放弃
    }
}

// ================= 工具函数 =================
/**
 * @brief 目标过滤检查
 */
static bool Target_Filtering_Check(const or_point_cloud_term_t *term, uint8_t zone)
{
    // return CAL_Target_Filtering(term) && (zone != 0);
    return (zone != 0);
}


/**
 * @brief 更新标定进度
 */
static void update_calibration_progress(uint8_t step, uint8_t offset)
{
    const uint8_t progress = adapt_format.adaptve_calibrationpara.Counter * step + offset;
    Calibration_Progress(progress);
}

/**
 * @brief 激活标定参数
 */
static void activate_calibration(void)
{
    adapt_format.adaptve_calibrationpara.SteeringAngle = Message_VehicleMsg.SteeringAngle;
    adapt_format.adaptve_calibrationpara.Velocity      = KMH_TO_MS(Message_VehicleMsg.Velocity);

// 调试数据记录
#ifdef CALIBRATION_DEBUG
    //log_calibration_data(adapt_format.adaptve_calibrationpara.SteeringAngle, adapt_format.adaptve_calibrationpara.AveYdata);
#endif
}

/**
 * @brief 校验Y数据有效性
 */
static bool is_valid_calibration_range(float y_data)
{
    return (y_data > YDATA_LOWER_BOUND_CASE1) && (y_data < YDATA_UPPER_BOUND_CASE1);
}


/**
 * @brief 执行标定数据收集流程
 * @param PeakList 点云数据结构体指针
 */
void calibration_data_collection(const or_point_cloud_format_t *PeakList)
{
    // ================= 输入有效性校验 =================
    if (!PeakList || PeakList->point_count == 0) {
       // LOG_ERROR("Invalid point cloud data");
        return;
    }

    // ================= 状态条件检查 =================
    const bool is_steer_valid =
        fabsf(Message_VehicleMsg.SteeringAngle - adapt_format.adaptve_calibrationpara.SteeringAngle) < STEERING_TOLERANCE;
    const bool is_velocity_valid =
        fabsf(KMH_TO_MS(Message_VehicleMsg.Velocity) - adapt_format.adaptve_calibrationpara.Velocity) < VELOCITY_TOLERANCE_MS;

    if (!is_steer_valid || !is_velocity_valid) {
        adaptive_data_clear(); //姿态突变
        return;
    }
    // ================= 数据收集循环 =================
    //Out_Put_Vehicle_List(0x10);
    uint32_t       valid_count     = 0;
    const uint32_t max_data_points = SINGLE_DATA_AMOUNT;
    const float    y_gap           = ADAPTIVE_YDATA_GAP;
    const or_point_cloud_term_t *term = 0;
    Point3D point = {0};

    for (uint32_t i = 0; i < PeakList->point_count; ++i) {
        term = &PeakList->term[i];
        // 阶段1: 快速过滤
        //Out_Put_Vehicle_List(0x11);
        if (!CAL_Target_Filtering(PeakList,i)) {
            continue;
        }
        // 阶段2: 坐标计算
        float azimuth_rad = 0;
        if (calibration_extern_para.cal_installation == INSTALL_FRONT
            || calibration_extern_para.cal_installation == INSTALL_BACK) {
             azimuth_rad = term->azimuth;
        } else {
             azimuth_rad = DEG_TO_RAD(calibration_extern_para.cal_install_angle) - term->azimuth;
        }
        //Out_Put_Vehicle_List(0x0D);

        const float   elevation_rad = term->elevation;
        point.x = term->range * cosf(azimuth_rad);
        point.z = elevation_rad;
        point.y = term->range * sinf(azimuth_rad);

        // 阶段3: 有效性验
        //Out_Put_Vehicle_List(0x0E);
        const bool is_y_in_range = (point.y > (adapt_format.adaptve_calibrationpara.AveYdata - y_gap)) && (point.y < (adapt_format.adaptve_calibrationpara.AveYdata + y_gap));
        const bool is_x_valid = ((point.x > X_DISTANCE_MIN)&&(point.x < X_DISTANCE_MAX));
        if ((adapt_format.adaptve_calibrationpara.DataNum < max_data_points) && is_y_in_range && is_x_valid) {
            store_calibration_data(point, term->range);
            valid_count++;
        }


        if((adapt_format.adaptve_calibrationpara.DataNum +1)%300 == 0)
        {
            {
                adapt_format.adaptve_calibrationpara.Counter+=1;
                update_calibration_progress(PROGRESS_STEP, 0);
            }
        }
        //Out_Put_Vehicle_List(0x13);
    }
    // ================= 标定状态更新 =================
    
    //Out_Put_Vehicle_List(0x12);
    adapt_format.adaptve_calibrationpara.Frame++;

    if (valid_count <= CALIB_TIMEFRAME_HALF) {
        if (++adapt_format.adaptve_calibrationpara.FalseFrame > CALIB_FAIL_FRAME_THRESH) {
            adaptive_data_clear();
            adapt_format.adaptve_calibrationpara.driving_profile = 0x10;
        }
    } else {
        adapt_format.adaptve_calibrationpara.FalseFrame = 0;
    }
}

// ================= 工具函数 =================
/**
 * @brief 存储标定数据点
 */
static void store_calibration_data(Point3D point, float range)
{
    const uint32_t idx            = adapt_format.adaptve_calibrationpara.DataNum;
    adapt_format.adaptve_calibrationpara.xdata[idx]    = point.x;
    adapt_format.adaptve_calibrationpara.ydata[idx]    = point.y;
    adapt_format.adaptve_calibrationpara.elevdata[idx]  = RAD_TO_DEG(point.z); // elevation转换
    adapt_format.adaptve_calibrationpara.rangdata[idx] = range;
    adapt_format.adaptve_calibrationpara.DataNum++;
}

static void adaptive_data_volume_judg(void)
{
    if (adapt_format.adaptve_calibrationpara.FalseFrame > (ADAPTIVE_FRAME_NUM / CALIBRATION_MIN_SAMPLES)) {
        adaptive_data_clear();            // 标定放弃
        adapt_format.adaptve_calibrationpara.driving_profile = 0x10; // 目标不充分
    } else if (adapt_format.adaptve_calibrationpara.Frame >= ADAPTIVE_FRAME_NUM || adapt_format.adaptve_calibrationpara.DataNum >= SINGLE_DATA_AMOUNT) {
        if (adapt_format.adaptve_calibrationpara.DataNum >= (SINGLE_DATA_AMOUNT * 3 / 5)) {
            adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_FINISH; // 进入拟合阶段
        } else {
            adaptive_data_clear();            // 标定放弃
            adapt_format.adaptve_calibrationpara.driving_profile = 0x10; // 目标不充分
        }
    }
}



void calibration_adaptive_polyfit(void)
{
    float32_t sum_x2 = 0;
    float32_t sum_y  = 0;
    float32_t sum_x  = 0;
    float32_t sum_xy = 0;
    uint32_t  i      = 0;
    float32_t a;
    float32_t b;
    //水平
    for (i = 0; i < adapt_format.adaptve_calibrationpara.DataNum; i++) {
        float32_t x = adapt_format.adaptve_calibrationpara.xdata[i];
        float32_t y = adapt_format.adaptve_calibrationpara.ydata[i];

        sum_x2 += x * x;
        sum_y += y;
        sum_x += x;
        sum_xy += x * y;
    }
    float32_t denominator = adapt_format.adaptve_calibrationpara.DataNum * sum_x2 - sum_x * sum_x;
    a = (adapt_format.adaptve_calibrationpara.DataNum * sum_xy - sum_x * sum_y) / denominator;
    b = (sum_x2 * sum_y - sum_x * sum_xy) / denominator;
    adapt_format.adaptve_calibrationpara.Adap_B = b;
    adapt_format.adaptve_calibrationpara.Adap_A = a;
    adapt_format.adaptve_calibrationpara.Adap_Angle =  RAD_TO_DEG(atan(a));
    
    //垂直
    float32_t sum_ele = 0;
    int16_t   count   = 0;
    for (int j = 0; j < adapt_format.adaptve_calibrationpara.DataNum; j++) {
        if (adapt_format.adaptve_calibrationpara.rangdata[j] >= 50 && adapt_format.adaptve_calibrationpara.rangdata[j] <= 60) {
            sum_ele += adapt_format.adaptve_calibrationpara.elevdata[j];
            count++;
        }
    }
    if (count != 0) {
        float32_t averagePitch        = sum_ele / count;
        adapt_format.adaptve_calibrationpara.Adap_eleAngle = averagePitch;
    }

    // printf("/********************************This_is_result*******************************/\n");
    // printf("/******************************************************************************/\n");
    printf("/*****************************Adap_Angle = %f****************************/\n",adapt_format.adaptve_calibrationpara.Adap_Angle);
    // printf("/******************************************************************************/\n");
    // printf("/******************************************************************************/\n");
    // printf("/******************************************************************************/\n");
}



void calibration_adaptive_finish(void)
{
    // #define CALIBRATION_ADAPTIVE_FINISH 1

    // offline_status_out_t *p_offline_status = offline_cal_get_status();//获取下线标定角度
    // /*debug*/

    // uint8_t StatusArray[9] = {0};
    // /*debug*/
    // float32_t TmpLinearAngle    = 0;
    // float32_t TmpLineareleAngle = 0;
    // if ((adapt_format.adaptve_calibrationpara.Adap_B > YDATA_LOWER_BOUND_CASE1)
    //     && (adapt_format.adaptve_calibrationpara.Adap_B < YDATA_UPPER_BOUND_CASE1)) 
    // {
    //         TmpLinearAngle    = adapt_format.adaptve_calibrationpara.Adap_Angle;
    //         TmpLineareleAngle = adapt_format.adaptve_calibrationpara.Adap_eleAngle;
    //         update_calibration_progress(PROGRESS_STEP,PROGRESS_STEP);

    //         adapt_format.adaptve_calibrationpara.Temp_A[adapt_format.adaptve_calibrationpara.Counter]   = TmpLinearAngle;
    //         adapt_format.adaptve_calibrationpara.Temp_ele[adapt_format.adaptve_calibrationpara.Counter] = TmpLineareleAngle;
    //         adapt_format.adaptve_calibrationpara.Counter++;

    //     if (adapt_format.adaptve_calibrationpara.Counter > ADAPTIVE_COUNTER - 1) {
    //         for (int i = 0; i < (adapt_format.adaptve_calibrationpara.Counter); i++) {
    //             for (int j = 0; j < (adapt_format.adaptve_calibrationpara.Counter - i - 1); j++) {
    //                 if (adapt_format.adaptve_calibrationpara.Temp_A[j] > adapt_format.adaptve_calibrationpara.Temp_A[j + 1]) {
    //                     TmpLinearAngle                = adapt_format.adaptve_calibrationpara.Temp_A[j];
    //                     adapt_format.adaptve_calibrationpara.Temp_A[j]     = adapt_format.adaptve_calibrationpara.Temp_A[j + 1];
    //                     adapt_format.adaptve_calibrationpara.Temp_A[j + 1] = TmpLinearAngle;
    //                 }
    //             }
    //         }
    //         for (int i = 0; i < (adapt_format.adaptve_calibrationpara.Counter); i++) {
    //             for (int j = 0; j < (adapt_format.adaptve_calibrationpara.Counter - i - 1); j++) {
    //                 if (adapt_format.adaptve_calibrationpara.Temp_ele[j] > adapt_format.adaptve_calibrationpara.Temp_ele[j + 1]) {
    //                     TmpLineareleAngle               = adapt_format.adaptve_calibrationpara.Temp_ele[j];
    //                     adapt_format.adaptve_calibrationpara.Temp_ele[j]     = adapt_format.adaptve_calibrationpara.Temp_ele[j + 1];
    //                     adapt_format.adaptve_calibrationpara.Temp_ele[j + 1] = TmpLineareleAngle;
    //                 }
    //             }
    //         }
        
            /*
            if (CHEACK_MODE == adapt_format.adaptve_calibrationpara.adaptive_Workmode) {
                TmpLinearAngle                = -adapt_format.adaptve_calibrationpara.Temp_A[3];
                TmpLineareleAngle             = RadarPara.RadarSelfDeviation + 0.5f - adapt_format.adaptve_calibrationpara.Temp_ele[3];
                adapt_format.adaptve_calibrationpara.Adaptive_step = 0x02; //检验完成
                adapt_format.adaptve_calibrationpara.Adaptive_Check_Angle    = TmpLinearAngle;
                adapt_format.adaptve_calibrationpara.Adaptive_Check_eleAngle = TmpLineareleAngle;

            } else 
            if (ADAPTIVE_MODE == adapt_format.adaptve_calibrationpara.adaptive_Workmode) {
       
                    TmpLineareleAngle = -adapt_format.adaptve_calibrationpara.Temp_A[3];
                    TmpLineareleAngle = RadarPara.RadarSelfDeviation + 0.5f - adapt_format.adaptve_calibrationpara.Temp_ele[3];
            */
           
    //                 if (calibration_extern_para.cal_installation == INSTALL_LEFT_BACK) {
    //                     TmpLinearAngle += 0.0f; //0.80f;
    //                 } else if (calibration_extern_para.cal_installation == INSTALL_RIGHT_BACK) {
    //                     TmpLinearAngle += 0.0f; //1.52f;
    //                 }
                    
    //                 adapt_format.adaptve_calibrationpara.apat_angle_h = TmpLinearAngle - p_offline_status->horizontal_angle_deviation;/********Need to modify**********/


    //                 if ((TmpLineareleAngle < (OfflineCalibration_elevTolerance_authentic - EPSILON))
    //                     && TmpLineareleAngle > (Calibration_elevTolerance + EPSILON)) {
    //                     TmpLineareleAngle = 2.6f + 0.3f * rand() / RAND_MAX * 1.0f; //随机数[2.6,2.9]
    //                 } else if (TmpLineareleAngle > (-(OfflineCalibration_elevTolerance_authentic) + EPSILON)
    //                            && TmpLineareleAngle < (-(Calibration_elevTolerance)-EPSILON)) {
    //                     TmpLineareleAngle = -2.9f + 0.3f * rand() / RAND_MAX * 1.0f; //随机数[-2.9,-2.6]
    //                 } else {
    //                     //do nothing
    //                 }
    //                 adapt_format.adaptve_calibrationpara.apat_angle_v    = TmpLineareleAngle - p_offline_status->vertical_angle_deviation;

    //                 update_calibration_progress(PROGRESS_STEP,CALIBRATION_ADAPTIVE_FINISH);   //标定完成但是未确定标定结果是否写入，进度99%，写入成功100% 

    //                 if ((fabs(TmpLinearAngle) > (Calibration_Tolerance + EPSILON))
    //                     || (fabs(TmpLineareleAngle) > (Calibration_elevTolerance + EPSILON))) {

    //                         adaptive_result_set(CALIBRATION_ADAPTIVE_IS_FAIL,CALIBRATION_ADAPTIVE_IS_ANGLE_OVERSHOOT,adapt_format.adaptve_calibrationpara.apat_angle_h,adapt_format.adaptve_calibrationpara.apat_angle_v);
                            
    //                 } else 
    //                 {
    //                     adaptive_result_set(CALIBRATION_ADAPTIVE_IS_SUCCESSFUL,CALIBRATION_ADAPTIVE_IS_NO_FAIL,adapt_format.adaptve_calibrationpara.apat_angle_h,adapt_format.adaptve_calibrationpara.apat_angle_v);
    //                 }
    //     }else
    //     {
    //         adaptive_data_clear();
    //     }
    // }
    // else
    // {
    //     adaptive_data_clear();
    // }
}

static void calibration_result_write(void)
{
    /*标定结果写入API*/
    adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_END; //标定结束
}

static void calibration_result_read(void)
{
    /*标定结果读取API*/
    /*读取直到读到的标定数据与写入的标定数据一致才判断为标定成功*/
}

static void calibration_adaptive_end(void)
{
    #define CALIBRATION_ADAPTIVE_END  2
    CAL_MODE = CALIBRATION_EXIT;
    update_calibration_progress(PROGRESS_STEP, CALIBRATION_ADAPTIVE_END);
    switching_mode_debug();
    // if (adap_result.last_result == CALIBRATION_ADAPTIVE_IS_SUCCESSFUL)
    // {
    //     calibration_result_read();
    //     /*读取直到读到的标定数据与写入的标定数据一致才判断为标定成功*/
    // }else
    // {
    //     //标定失败API
    // }
}



int32_t adaptive_flow_control_func(const or_point_cloud_format_t *PeakList)
{

    //printf("\r");
    //printf("adapt_format.adaptve_calibrationpara.adaptive_PB =  %d\n",adapt_format.adaptve_calibrationpara.adaptive_PB);
    switch (adapt_format.adaptve_calibrationpara.Step) {
    case ADAPTIVE_START:
        /* code */
        //set_time(10)开启标定计时API
        //AdaptiveCalClockLaunch(ADAPTIVE_CAL_TIMEOUT_CYCLE);
        adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_INIT;
        break;
    case ADAPTIVE_WORKMODE_CHECK:
        /* code */
        adaptive_wokemode_check();
        break;
    case ADAPTIVE_INIT:
        /* code */
        adaptive_init();
        break;
    case ADAPTIVE_BODY_POSTURE_DETECTION:
        /* code */
        if(body_posture_detection())
        {//printf("body_posture_detection success\n");
            calibration_init_process(PeakList);
        }else
        {   
            //车辆信号不满足
            adaptive_data_clear();
        }
        
        break;
    case ADAPTIVE_DATUM_SELECTION:
        calibration_data_collection(PeakList);
        adaptive_data_volume_judg();
        break;
    case ADAPTIVE_FINISH:
    //printf("calibration_adaptive_polyfit\n");
        calibration_adaptive_polyfit(); // 线性回归拟合
        calibration_adaptive_finish();
        adapt_format.adaptve_calibrationpara.Step = ADAPTIVE_END;
        //calibration_result_write();
        /* code */
        //stop_time();
    break;
    case ADAPTIVE_END:
        /* code */
        
        calibration_adaptive_end();
        //stop_time();
        break;

    default:
        break;
    }
    
    printf("当前标定进度；%d\r",adapt_format.adaptve_calibrationpara.adaptive_PB);
    usleep(60000);
    return 0;
}


/******************************END OF FILE*************************************/
