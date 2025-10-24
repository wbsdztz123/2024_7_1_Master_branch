/****************************************************************************
 *                        File: 1014calibration.c                    *
 *                          @zhujunnan20251016                                           *
 ****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <locale.h>
#include "1014calibration.h"
static_peak_t cal_peak[MAX_STATIC_PEAK_NUM] = {0};
adapt_params_t adapt_params = {0};
/*存在问题  多帧目标收集*/
/*条件不满?? 后续策略*/


/*

*/
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

bool point_doppler_filter(const or_point_cloud_term_t *point_cloud)
{
    bool isLeftOrRightFront = (RadarPara.InstallPosition == INSTALL_LEFT_FRONT || RadarPara.InstallPosition == INSTALL_RIGHT_FRONT || RadarPara.InstallPosition == INSTALL_FRONT);
    bool isDopplerNegative = point_cloud->doppler < 0.0f;

    return (isLeftOrRightFront && isDopplerNegative);
}

bool point_param_filter(const or_point_cloud_term_t *point_cloud)
{
    bool isangle_valid = fabsf(RAD_TO_DEG(point_cloud->azimuth)) < 65.0f;
    bool isrange_valid = (point_cloud->range > 5.0f)&&(point_cloud->range < 80.0f);
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

    bool is_valid = (average_residual < 0.12f)&&(fabsf(calibrated_yaw)<=5.0f); //速度残差阈??

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
            //   if(cal_static_Peak[i].angle > 0.0f)
            //   {
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
            // printf("peak%d: 角度=%-6.1f° 测量速度=%-7.2f 预测速度=%-7.2f\n",i,RAD_TO_DEG(angle_rad),cal_static_Peak[i].doppler,predicted);
            //  printf("速度残差:%f\n",residual);
            //  printf("梯度贡献:%f\n",residual*(-KMH_TO_MS(Message_VehicleMsg.Velocity))*sinf(angle_rad + yaw_deg));
;
    
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
    //const float TOLERANCE = DEG_TO_RAD(0.05f);;
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
   // printf("angle_dif = %f\n",angle_dif);
    if(angle_dif > 0.5f)
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
     return current_yaw;//当前偏航??
}



float estimate_yaw_from_all_points(const or_point_cloud_format_t *PeakList) {
    
    //基于最小二乘的梯度下降优化
    return gradient_descent_optimization(PeakList);
}

//内点收集
int16_t collection_internal_point(const or_point_cloud_format_t *PeakList,float candidate_angle)
{
    int16_t inlier_count = 0; // 内点数量
    for(int i = 0; i < PeakList->point_count; i++)
    {
        
        if((!point_doppler_filter(&PeakList->term[i]))||(!point_param_filter(&PeakList->term[i])))
        {
            continue;
        }
        float32_t perdicted_doppler = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(PeakList->term[i].azimuth + DEG_TO_RAD(candidate_angle)); //预测的径向速度
        float residual = fabsf(PeakList->term[i].doppler - perdicted_doppler); //速度残差

        if (residual < RANSAC_THRESHOLD && adapt_params.static_peak_num < MAX_STATIC_PEAK_NUM)
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
   // printf("candidate_angle////////////////////////// = %f\n",candidate_angle);
    int16_t inlier_count = 0; // 内点数量
    for(int i = 0; i < PeakList->point_count; i++)
    {
        if((!point_doppler_filter(&PeakList->term[i]))||(!point_param_filter(&PeakList->term[i])))
        {
            continue;
        }
        float32_t perdicted_doppler = -KMH_TO_MS(Message_VehicleMsg.Velocity)*cosf(PeakList->term[i].azimuth + DEG_TO_RAD(candidate_angle)); //预测的径向速度
        float residual = fabsf(PeakList->term[i].doppler - perdicted_doppler); //残差
        // printf("PeakList->term[i].doppler = %f\n",PeakList->term[i].doppler);
       // printf("residual = %f\n",residual);
        if (residual < RANSAC_THRESHOLD)
        {
            inlier_count++;
        }
    }
   // printf("inlier_count = %d\n",inlier_count);
    return inlier_count;
}

int find_max_num_index(int16_t *best_candidate_angle,int16_t num_candidates)
{
    int max_index = 0;
    int16_t max_value = best_candidate_angle[0];
    printf("best_candidate_angle[0] = %d\n",best_candidate_angle[0]);
    for(int i = 1; i < num_candidates; i++)
    {
        printf("best_candidate_angle[%d] = %d\n",i,best_candidate_angle[i]);
        if(best_candidate_angle[i] > max_value)
        {
            max_value = best_candidate_angle[i];
            max_index = i;
        }
    }
    return max_index;
}



int16_t RANSAC_calibration(const or_point_cloud_format_t *PeakList)
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
    if((best_candidate_index > -1)&&(adapt_params.cal_frame < MAX_CANDIDATE_FRAME))//防止best_candidate_angle数组越界
    {
        adapt_params.best_candidate_angle[best_candidate_index]++;//候选角度契合度
        adapt_params.cal_frame++;
    }


    if(adapt_params.cal_frame >= MAX_CANDIDATE_FRAME) //前150帧确定最佳候选角度  
    {
        best_candidate_index = find_max_num_index(adapt_params.best_candidate_angle,num_candidates);//返回契合度最高的候选角度索引
        adapt_params.candidate_yaws = candidate_yaws[best_candidate_index];
        printf("best_candidate_candidate_yaws: %f\n",adapt_params.candidate_yaws);

            if(adapt_params.best_inlier_count > MIN_BEST_INLIER_COUNT+40 && best_candidate_index >= 0)
            {
                collection_internal_point(PeakList,adapt_params.candidate_yaws);
                
                if(adapt_params.static_peak_num >= MAX_STATIC_PEAK_NUM)
                {
                //printf("adapt_params.static_peak_num**************************** = %d\n",adapt_params.static_peak_num);
                    adapt_params.candidate_yaws = candidate_yaws[best_candidate_index];
                    // 角度估计
                    adapt_params.cal_step = ANGLE_ESTIMATION;
                }
                adapt_params.best_ca_index = best_candidate_index;
            }else
            {
            }
            adapt_params.cal_step = CAL_STOP;
    }

    

    // if(best_inlier_count > 0)
    // {
    //     float best_yaw = candidate_yaws[best_candidate_index];

    //     if (fabsf(best_yaw - initial_guess > 2.0f)) {
    //         // 用初始猜测再验证一??
    //         int verify_count = count_inliers_for_candidate(PeakList,initial_guess);
    //         if (verify_count > best_inlier_count * 0.8f) {
    //             // 初始猜测也不错，合并结果

    //         }
    // }
    
    return adapt_params.best_inlier_count;
}


void adapt_calibration(const or_point_cloud_format_t *PeakList)
{

    if((fabs(Message_VehicleMsg.SteeringAngle)>3.0f)&&(fabs(Message_VehicleMsg.SteeringAngle)<7.0f)&&(fabsf(Message_VehicleMsg.YawRate)<0.7f)&&(Message_VehicleMsg.Velocity>20.0f)&(Message_VehicleMsg.Velocity<50.0f))
    {
//printf("adapt_params.cal_step = %d\n",adapt_params.cal_step);
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
                gradient_descent_optimization(PeakList);
                break;

            case RESULT_VERIFICATION:
                validate_calibration_result(cal_peak,adapt_params.best_ca_angle);
                break;    
            
            case CAL_STOP:

                break;   
            
            default:
                break;
            }
    }

    
}

// 统计符合当前模型的内点数??  通过速度筛选出符合条件的点 
/*
- PeakList: 所有雷达点云数??
- num_points: 点云数量
- vehicle_speed: 车辆速度
- candidate_yaw: 当前候选的偏航角（度）
- threshold: 残差阈??(速度偏差阈??)
- inlier_mask: 输出内点标记数组
*/
// int count_inliers(const or_point_cloud_format_t *PeakList, int num_points, float vehicle_speed, 
//                  float candidate_yaw, float threshold, bool* inlier_mask) {
//     int inlier_count = 0;
//     float yaw_rad = DEG_TO_RAD(candidate_yaw);
    
//     for (int i = 0; i < num_points; i++) {
//         // if (!is_point_in_valid_fov(PeakList, &config)) {
//         //     continue;
//         // }
        
//         // 计算残差
//         float angle_rad = PeakList->term[i].azimuth;

//         float predicted_velocity = -vehicle_speed * cosf(angle_rad + yaw_rad);   //计算纠正后的目标速度

//         float residual = fabsf(PeakList->term[i].doppler - predicted_velocity);  //实际探测速度与计算速度偏差
        
//         // 判断是否为内??
//         bool is_inlier = (residual < threshold);
//         // if (inlier_mask) {
//         //     inlier_mask[i] = is_inlier;
//         // }
        
//         if (is_inlier) {
//             inlier_count++;
//         }
//     }
    
//     return inlier_count;
// }

