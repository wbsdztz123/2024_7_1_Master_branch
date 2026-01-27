/****************************************************************************
 *                        File: 1014calibration.c                    *
 *                          @zhujunnan20251016                                           *
 ****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <locale.h>
#include "1014calibration.h"
static_peak_t cal_peak[MAX_STATIC_PEAK_NUM] = {0};
or_point_cloud_format_t optimal_frame = {0};
adapt_params_t adapt_params = {0};
data_confidence_t peak_confidence = {0};
/*存在问题  多帧目标收集*/
/*条件不满  后续策略*/

float calculate_overall_confidence(void)
{
    float score = 0.0f;    // 实际得分
    float max_score = 0.0f; // 理论最大得分

    if(peak_confidence.valid_point_count >= 100) {  // 确保有足够统计意义
    // 内点评分 = 内点比例×70% + 内点数量得分×30%
    float inlier_score = (peak_confidence.inlier_ratio * 0.7f) + (fmin(peak_confidence.inlier_count / 150.0f, 1.0f) * 0.3f);
    score += inlier_score * 0.3f;  // 应用30%权重
    }
    max_score += 0.3f;  // 内点维度的最大可能得分
    
    // 2. 残差质量评估 (权重: 70%)
    if(peak_confidence.avg_residual < 0.5f) {  // 平均残差在合理范围内
    // 残差评分 = 平均残差质量×60% + 残差一致性×40%
        float residual_score = (1.0f - peak_confidence.avg_residual / 0.5f) * 0.6f + (1.0f - fmin(peak_confidence.residual_std / 0.5f, 1.0f)) * 0.4f;
        score += residual_score * 0.7f;  // 应用30%权重
    }
    max_score += 0.7f;

    return (max_score > 0) ? (score / max_score) : 0.0f;
}


//点云基础滤波  统计通过基础滤波的点数
//return：点数
uint16_t Basic_filtering_of_point_cloud(const or_point_cloud_format_t *PeakList)
{
    int16_t inlier_count = 0; // 内点数量
    for(int i = 0; i < PeakList->point_count; i++)
    {
        if((!point_doppler_filter(PeakList->term[i].doppler))||(!point_param_filter(PeakList->term[i].azimuth,PeakList->term[i].range)))
        {
            continue;
        }else
        {
            inlier_count++;
        }
    }
    return inlier_count;
}

//残差统计
void calculate_residual_stats(const or_point_cloud_format_t *PeakList, float candidate_angle)
{
    float candidate_angle_rad = DEG_TO_RAD(candidate_angle);  
    float sum_residual = 0.0f;          // 残差总和，用于计算平均值
    float sum_squared_residual = 0.0f;  // 残差平方和，用于计算标准差
    int count = 0;                      // 有效点数

    for(int i = 0; i < PeakList->point_count; i++)
    {
        if((!point_doppler_filter(PeakList->term[i].doppler))||(!point_param_filter(PeakList->term[i].azimuth,PeakList->term[i].range)))
        {
            continue;
        }
        float32_t perdicted_doppler = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(PeakList->term[i].azimuth + DEG_TO_RAD(candidate_angle)); //预测的径向速度
        float residual = fabsf(PeakList->term[i].doppler - perdicted_doppler); //残差
        // printf("PeakList->term[i].doppler = %f\n",PeakList->term[i].doppler);
       // printf("residual = %f\n",residual);

        sum_residual += residual;//残差总和
        sum_squared_residual += residual * residual;//残差平方和
        count++;//有效点数
    }

    if(count > 0)
    {
        peak_confidence.avg_residual = sum_residual / count; //平均残差

        float variance = (sum_squared_residual / count) - (peak_confidence.avg_residual * peak_confidence.avg_residual);//方差

        peak_confidence.residual_std = sqrtf(fmaxf(variance, 0.0f));  //标准差 fmaxf确保非负
    }else
    {
        peak_confidence.avg_residual = 1000.0f;
        peak_confidence.residual_std = 1000.0f;
    }
}

bool evaluate_data_confidence(const or_point_cloud_format_t *PeakList,float candidate_angle,or_point_cloud_format_t* this_frame_point_cloud)
{
    //printf("peak_confidence.frame_num1 = %d\n",peak_confidence.frame_num);
    //printf("point_count1 = %d\n",this_frame_point_cloud->point_count);
    #define CAL_FRAME_NUM 5 

    float32_t this_frame_score = 0.0f; //本帧得分

    peak_confidence.inlier_count = count_inliers_for_candidate(PeakList,adapt_params.candidate_yaws); //本帧静止点数
    peak_confidence.valid_point_count = Basic_filtering_of_point_cloud(PeakList); //通过基础滤波的点数
    peak_confidence.inlier_ratio = (peak_confidence.valid_point_count > 0) ? (float)peak_confidence.inlier_count / peak_confidence.valid_point_count : 0.0f;//静止点比例

    //平均残差越小说明拟合越好，标准差越小说明一致性越高
    calculate_residual_stats(PeakList, candidate_angle);//残差统计

   // peak_confidence.quality_level = (uint8_t)(peak_confidence.confidence_score * 5); 
    this_frame_score = calculate_overall_confidence(); //.
    printf("this_frame_score= %f\n",this_frame_score);

    if(peak_confidence.frame_num < CAL_FRAME_NUM)
    {
        if(this_frame_score > peak_confidence.confidence_score)
        {
            peak_confidence.confidence_score = this_frame_score;
            memcpy(this_frame_point_cloud, PeakList, sizeof(or_point_cloud_format_t));
            printf("peak_confidence.confidence_score= %f\n",peak_confidence.confidence_score);
        }
        peak_confidence.frame_num++;
    }else
    {
        peak_confidence.frame_num = 0;
        peak_confidence.confidence_score = 0;
        return true;
    }
    return false;
}


void calibration_conditions_not_met(void)
{
    memset(&adapt_params,0,sizeof(adapt_params_t));
    adapt_params.cal_step = CAL_INIT;
    memset(&cal_peak,0,sizeof(static_peak_t)*MAX_STATIC_PEAK_NUM);
}

void init_calibration(void)
{
    memset(&adapt_params,0,sizeof(adapt_params_t));
    adapt_params.cal_step = CAL_RANSAC;
    memset(&cal_peak,0,sizeof(static_peak_t)*MAX_STATIC_PEAK_NUM);
}

bool point_doppler_filter(const float32_t doppler)
{
    bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_LEFT_FRONT || RadarPara.InstallPosition == INSTALL_RIGHT_FRONT || RadarPara.InstallPosition == INSTALL_FRONT);
    bool isDopplerNegative = doppler < 0.0f;

    return (isLeftOrRightFront && isDopplerNegative);
}

bool point_param_filter(const float32_t azimuth,const float32_t range)
{
    bool isangle_valid = fabsf(RAD_TO_DEG(azimuth)) < 65.0f;
    bool isrange_valid = (range > 5.0f)&&(range < 80.0f);
    return (isangle_valid && isrange_valid);
}

bool validate_calibration_result(const static_peak_t *cal_static_Peak,float calibrated_yaw) 
{
    float total_rasidual = 0.0f;
    int valid_count = 0;

    for(int i = 0;i < adapt_params.static_peak_num;i++)
    {
        float predicted = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(cal_static_Peak[i].angle + DEG_TO_RAD(calibrated_yaw));//预测的径向速度

        float residual = fabsf(cal_static_Peak[i].doppler - predicted); //速度残差

        total_rasidual += residual;
    }
    float average_residual = total_rasidual / adapt_params.static_peak_num;

    bool is_valid = (average_residual < 0.12f)&&(fabsf(calibrated_yaw)<=5.0f); //速度残差阈

    printf("average_residual=%.3fm/s, calibrated_yaw=%.3f, adapt_params.static_peak_num=%d, is_valid=%s\n",
    average_residual, calibrated_yaw, adapt_params.static_peak_num, is_valid ? "yes" : "no");
    if(is_valid)
    {
        adapt_params.cal_step = CAL_STOP;
    }else
    {
        adapt_params.cal_step = CAL_INIT; 
    }
    return is_valid;
}

// 初始值：从内点计算平均偏航角
float compute_initial_yaw_from_inliers(const static_peak_t *cal_static_Peak)
{
    float sum_yaw = 0.0f;
    int16_t valid_count = 0;
    for(int i = 0;i < adapt_params.static_peak_num;i++)
    {
        float ratio = (-cal_static_Peak[i].doppler)/KMH_TO_MS(Message_VehicleMsg.Velocity); //cos(α + ψ)
        if(fabs(ratio) <= 1.0f)
        {
            float angle_rad = cal_static_Peak[i].angle;//雷达探测角度
            float sin_theta = sqrtf(1.0f - ratio * ratio);   //sin(α + ψ)
            if (angle_rad < 0) 
            {
                // 右侧点，sin(α + ψ) 应该为负
                sin_theta = -sin_theta;
            }
           float true_angle = atan2f(sin_theta, ratio);
            //if(cal_static_Peak[i].angle > 0.0f)
            //{
            //     //printf("angle_rad= %f\n",RAD_TO_DEG(angle_rad));
            //     float true_angle = acosf(ratio);//目标真实角度
            //     //printf("true_angle = %f\n",RAD_TO_DEG(true_angle));
            //     //printf("yaw_rad_DEG1111 = %f\n",RAD_TO_DEG(yaw_rad));
            // }
            float yaw_rad = true_angle - angle_rad;//偏差角度
            sum_yaw += yaw_rad; 
            valid_count ++;
        }
    }


    if(valid_count > MAX_STATIC_PEAK_NUM*0.5f)
    {
        //printf("yaw_valid_count: %f\n",(sum_yaw / valid_count));
        return sum_yaw / valid_count;  //平均偏差(弧度)
    }else
    {
        calibration_conditions_not_met();//数据量不足
        return 0.0f;
    }

}

//梯度计算
//yaw_deg: 偏航角(弧度)
float compute_gradient(const static_peak_t *cal_static_Peak,float yaw_deg)
{
    // printf("\n=== 梯度诊断 ===\n");
    // printf("当前偏航角: %.3f°\n", RAD_TO_DEG(yaw_deg));
    // printf("车辆速度: %.2f m/s\n", KMH_TO_MS(Message_VehicleMsg.Velocity));
    float gradient = 0.0f;
    int used_points = 0;
    for ( int i = 0; i < adapt_params.static_peak_num;i++)
    {
        float angle_rad = cal_static_Peak[i].angle;
        if(cal_static_Peak[i].angle > 0.0f)
        {
            float predicted = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(angle_rad + yaw_deg);//期望目标速度
            
            float residual = cal_static_Peak[i].doppler - predicted;//速度残差
            
            if(fabs(residual) > 1.0f)
            {
                continue;
            }
            gradient += residual*(-KMH_TO_MS(Message_VehicleMsg.Velocity))*sinf(angle_rad + yaw_deg); //梯度
            //  printf("peak%d: 角度=%-6.1f° 测量速度=%-7.2f 预测速度=%-7.2f\n",i,RAD_TO_DEG(angle_rad),cal_static_Peak[i].doppler,predicted);
            //  printf("速度残差:%f\n",residual);
            //  printf("梯度贡献:%f\n",residual*(-KMH_TO_MS(Message_VehicleMsg.Velocity))*sinf(angle_rad + yaw_deg));

            used_points++;
        }
    }
    if(used_points > 0){
        gradient /= used_points;
        printf("平均梯度: %.6f\n", gradient);
    }

        if (gradient < 0) {
            printf("梯度符号: 负 → 应该增加偏航角\n");
        } else {
            printf("梯度符号: 正 → 应该减小偏航角\n");
        }
    return gradient;
}


float gradient_descent_optimization(const or_point_cloud_format_t *PeakList) {

    float LEARNING_RATE = 0.01f; //自学习率
    const int MAX_ITERATIONS = 1000; // 最大迭代次数
    //const float TOLERANCE = DEG_TO_RAD(0.05f);
    const float32_t MAX_ANGLE_ERROR = DEG_TO_RAD(5.0f); // 最大偏航角误差

    float current_yaw = compute_initial_yaw_from_inliers(cal_peak); //初始值：从内点计算平均偏航角

    if (0 == current_yaw)
    {
        current_yaw = DEG_TO_RAD(adapt_params.candidate_yaws);
        return 0.0f; //内点质量异常
    }

    printf("内点计算平均偏航角current_yaw = %.3f\n",RAD_TO_DEG(current_yaw));
    printf("RANSAC_calibration_angle=%.3f\n",adapt_params.candidate_yaws);
    float32_t angle_dif = fabsf(RAD_TO_DEG(current_yaw) - adapt_params.candidate_yaws);
    if(angle_dif > 1.0f)
    {
        calibration_conditions_not_met();
        printf("fabsf(RAD_TO_DEG(current_yaw) - adapt_params.candidate_yaws = %f\n",angle_dif);
        return 0.0f; 
    }

    for(int i = 0; i < MAX_ITERATIONS; i++)
    {
        float gradient = compute_gradient(cal_peak,current_yaw);

        if (fabsf(gradient) < 0.1f) {
            LEARNING_RATE = 0.01f;  // 精细调整
        } else {
            LEARNING_RATE = 0.001f; // 粗略调整
        }
        printf("gradient = %f\n",gradient);

        // 梯度下降更新
        float new_yaw = current_yaw - LEARNING_RATE * gradient;

        // 应用物理约束
        if (fabsf(new_yaw) > MAX_ANGLE_ERROR)
        {
            calibration_conditions_not_met();//重新角度
        }
        // 收敛检查
        float angle_change_rad = fabsf(new_yaw - current_yaw);
        printf("收敛偏差angle_change_rad = %f\n",angle_change_rad);
        printf("迭代次数:%d\n",i);
         if (angle_change_rad < 1e-6f) {
            break;
         }
        current_yaw = new_yaw;
        printf("角度迭代current_yaw = %f\n",RAD_TO_DEG(current_yaw));
    }
    adapt_params.best_ca_angle = RAD_TO_DEG(current_yaw); //更新

    printf("gradient_descent_optimization_new= F(%.3f)\n",adapt_params.best_ca_angle);
    adapt_params.cal_step = RESULT_VERIFICATION;
    return current_yaw;//当前偏航角
}


float estimate_yaw_from_all_points(const or_point_cloud_format_t *PeakList) {
    
    //基于最小二乘的梯度下降优化
    return gradient_descent_optimization(PeakList);
}

//内点收集
int16_t collection_internal_point(const or_point_cloud_format_t* PeakList,float candidate_angle)
{
    uint8_t Calibration_flag = 0;
    float32_t temp_speed_gap = 0xff;

    int16_t inlier_count = 0; // 内点数量
    for(int i = 0; i < PeakList->point_count; i++)
    {
        if((!point_doppler_filter(PeakList->term[i].doppler))||(!point_param_filter(PeakList->term[i].azimuth,PeakList->term[i].range)))
        {
            continue;
        }
        float32_t perdicted_doppler = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(PeakList->term[i].azimuth + DEG_TO_RAD(candidate_angle)); //预测的径向速度
        float residual = fabsf(PeakList->term[i].doppler - perdicted_doppler); //速度残差

        uint8_t Calibration_flag = 0;
        float32_t temp_speed_gap = 0xff;
        float32_t cosValue = cos((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
        float32_t speed = Message_VehicleMsg.Velocity / 3.6f;
        float32_t threshold = 0.15f;

        if (speed >= 4.1f && speed < 8.3f)
        {
            threshold = 0.10f;//0.12//0.13
        }
        else if (speed >= 8.3f)//0.1//.12
        {
            threshold = 0.08f;
        }
        temp_speed_gap = fabs(PeakList->term[i].doppler / cosValue + speed);

        Calibration_flag = (temp_speed_gap < speed * threshold);
        
        if ((residual<RANSAC_THRESHOLD) && adapt_params.static_peak_num < MAX_STATIC_PEAK_NUM)
        {
            cal_peak[adapt_params.static_peak_num].angle = PeakList->term[i].azimuth;
            cal_peak[adapt_params.static_peak_num].doppler = PeakList->term[i].doppler;
            cal_peak[adapt_params.static_peak_num].range = PeakList->term[i].range;
            cal_peak[adapt_params.static_peak_num].snr = PeakList->term[i].snr;
            cal_peak[adapt_params.static_peak_num].residual = residual;
           ang_dopp_rang_snr_vel(RAD_TO_DEG(cal_peak[adapt_params.static_peak_num].angle),cal_peak[adapt_params.static_peak_num].doppler,cal_peak[adapt_params.static_peak_num].range,Message_VehicleMsg.Velocity,cal_peak[adapt_params.static_peak_num].residual);
            adapt_params.static_peak_num++;
        }
    }
}

//快速内点检索
int16_t count_inliers_for_candidate(const or_point_cloud_format_t *PeakList,float candidate_angle)
{
    int16_t inlier_count = 0; // 内点数量
    for(int i = 0; i < PeakList->point_count; i++)
    {
        if((!point_doppler_filter(PeakList->term[i].doppler))||(!point_param_filter(PeakList->term[i].azimuth,PeakList->term[i].range)))
        {
            continue;
        }
        float32_t perdicted_doppler = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(PeakList->term[i].azimuth + DEG_TO_RAD(candidate_angle)); //预测的径向速度
        float residual = fabsf(PeakList->term[i].doppler - perdicted_doppler); //残差

        uint8_t Calibration_flag = 0;
        float32_t temp_speed_gap = 0xff;
        float32_t cosValue = cos((0 + (PeakList->term[i].azimuth * 180 / PI)) * PI / 180);
        float32_t speed = Message_VehicleMsg.Velocity / 3.6f;
        float32_t threshold = 0.15f;

        if (speed >= 4.1f && speed < 8.3f)
        {
            threshold = 0.10f;//0.12//0.13
        }
        else if (speed >= 8.3f)//0.1//.12
        {
            threshold = 0.08f;
        }
        temp_speed_gap = fabs(PeakList->term[i].doppler / cosValue + speed);

        Calibration_flag = (temp_speed_gap < speed * threshold);
        if (residual < RANSAC_THRESHOLD)
        {
            inlier_count++;
        }
    }
    return inlier_count;
}
//快速内点检索
int16_t advanced_original_point_filtering(float candidate_angle)
{
    int16_t inlier_count = 0; // 内点数量
    for(int i = 0; i < MAX_STATIC_PEAK_NUM; i++)
    {
        if((!point_doppler_filter(cal_peak[i].doppler))||(!point_param_filter(cal_peak[i].angle,cal_peak[i].range)))
        {
            continue;
        }
        float32_t perdicted_doppler = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(cal_peak[i].angle + DEG_TO_RAD(candidate_angle)); //预测的径向速度
        float32_t residual = fabsf(cal_peak[i].doppler - perdicted_doppler); //残差
        if (residual < RANSAC_THRESHOLD)
        {
            inlier_count++;
        }
    }
    return inlier_count;
}


int find_max_num_index(int16_t *best_candidate_angle,int16_t num_candidates)
{
    int max_index = 0;
    int16_t max_value = best_candidate_angle[0];
    for(int i = 1; i < num_candidates; i++)
    {
        if(best_candidate_angle[i] > max_value)
        {
            max_value = best_candidate_angle[i];
            max_index = i;
        }
    }
    return max_index;
}


int16_t  RANSAC_calibration(const or_point_cloud_format_t *PeakList)
{
    float initial_guess = 0.0f;//默认

    int16_t inlier_count = 0;//内点数量

    float candidate_yaws[] = {-5.0f,-4.5f,-4.0f,-3.5f,-3.0f,-2.5f,-2.0f,-1.5f,-1.0f,-0.5f,0.1f,0.5f,1.0f,1.5f,2.0f,2.5f,3.0f,3.5f,4.0f,4.5f,5.0f};
    const int num_candidates = CANDIDATE_ANGLE_NUM;//候选角度数

    int best_inlier_count = 0;  //最佳内点数
    int best_candidate_index = -1; //最佳候选角度索引

    for(int i = 0; i < num_candidates; i++)
    {
        float candidate_yaw = candidate_yaws[i];
        int16_t inlier_count = count_inliers_for_candidate(PeakList, candidate_yaw);

        if((inlier_count > best_inlier_count)&&(inlier_count > MIN_INLIER_COUNT))
        {
            best_inlier_count = inlier_count;
            best_candidate_index = i;
        }
    }

    //printf("best_inlier_count = %d\n",best_inlier_count);
    if((best_candidate_index > -1)&&(adapt_params.cal_frame < MAX_CANDIDATE_FRAME))//防止best_candidate_angle数组越界
    {
        adapt_params.best_candidate_angle[best_candidate_index]++;//候选角度契合度
        adapt_params.cal_frame++;
    }

    if(adapt_params.cal_frame >= MAX_CANDIDATE_FRAME) //前150帧确定最佳候选角度  
    {
        best_candidate_index = find_max_num_index(adapt_params.best_candidate_angle,num_candidates);//返回契合度最高的候选角度索引
        adapt_params.candidate_yaws = candidate_yaws[best_candidate_index];//契合度最高的预期角度

        if(evaluate_data_confidence(PeakList,adapt_params.candidate_yaws, &optimal_frame))
        {
            printf("optimal_frame_count = %d\n",optimal_frame.point_count);
            collection_internal_point(&optimal_frame,adapt_params.candidate_yaws);
            if(adapt_params.static_peak_num >= MAX_STATIC_PEAK_NUM)
            {
                // 角度估计
                Precise_angle_estimation_calibration(adapt_params.candidate_yaws);
                adapt_params.cal_step = CAL_STOP;
                // adapt_params.cal_step = ANGLE_ESTIMATION;
            }
            adapt_params.best_ca_index = best_candidate_index;
        }

        //adapt_params.cal_step = CAL_STOP;
    }
    return adapt_params.best_inlier_count;
}


int16_t Precise_angle_estimation_calibration(float32_t  yaw_angle)
{
    //or_point_cloud_format_t optimal_frame = {0};
#define PRECISE_ANGLE_NUM 11
    float initial_guess = 0.0f;//默认
    int16_t inlier_count = 0;//内点数量 

    const int num_candidates = PRECISE_ANGLE_NUM;//候选角度数
    
    float candidate_yaws[PRECISE_ANGLE_NUM] = {0};
    for(int i = 0;i < num_candidates;i++)
    {
        candidate_yaws[i] = yaw_angle + (i-5)*0.1f;
    }
    
    int best_inlier_count = 0;  //最佳内点数
    int best_candidate_index = -1; //最佳候选角度索引

    for(int i = 0; i < num_candidates; i++)
    {
        float candidate_yaw = candidate_yaws[i];

        int16_t inlier_count = advanced_original_point_filtering(candidate_yaw);

        if((inlier_count > best_inlier_count)&&(inlier_count > MIN_INLIER_COUNT))
        {
            best_inlier_count = inlier_count;
            best_candidate_index = i;
        }
    }
    printf("adapt_params.candidate_yaws = %.3f\n",yaw_angle);
    if((best_candidate_index > -1))//防止best_candidate_angle数组越界
    {
        adapt_params.candidate_yaws = candidate_yaws[best_candidate_index];//契合度最高的预期角度
        printf("Precise_angle_estimation_calibration_new= F(%.3f)\n",adapt_params.candidate_yaws);
    }




    // collection_internal_point(&optimal_frame,adapt_params.candidate_yaws);
    // if(adapt_params.static_peak_num >= MAX_STATIC_PEAK_NUM)
    // {
    //     //角度估计
    //     adapt_params.cal_step = ANGLE_ESTIMATION;
    // }
    // adapt_params.best_ca_index = best_candidate_index;

    return adapt_params.best_inlier_count;
}


void adapt_calibration(const or_point_cloud_format_t *PeakList)
{

    if((fabs(Message_VehicleMsg.SteeringAngle)>3.0f)&&(fabs(Message_VehicleMsg.SteeringAngle)<5.0f)&&(fabsf(Message_VehicleMsg.YawRate)<0.7f)&&(Message_VehicleMsg.Velocity>20.0f)&(Message_VehicleMsg.Velocity<50.0f))
    {
            switch (adapt_params.cal_step)
            {
            case CAL_INIT:
                init_calibration();
                //adapt_params.cal_step = CAL_RANSAC;
                break;
            case CAL_RANSAC:
                RANSAC_calibration(PeakList);
                break;
            case ANGLE_ESTIMATION:
                //gradient_descent_optimization(PeakList);
                break;

            case RESULT_VERIFICATION:
                //validate_calibration_result(cal_peak,adapt_params.best_ca_angle);
                break;    
            
            case CAL_STOP:

                break;   
            
            default:
                break;
            }
    }

    
}


