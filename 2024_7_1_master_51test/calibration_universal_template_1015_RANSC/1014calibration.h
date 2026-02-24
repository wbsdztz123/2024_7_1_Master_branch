
#ifndef CALIBRATION_H
#define CALIBRATION_H


#include "commapi.h"
#include "calibration_common.h"
#define MAX_STATIC_PEAK_NUM 300
#define  RANSAC_THRESHOLD  0.10f // 内点残差筛选阈值

#define MIN_INLIER_COUNT 10
#define MIN_BEST_INLIER_COUNT 50

#define CANDIDATE_ANGLE_NUM 21

#define MAX_CANDIDATE_FRAME 100

typedef enum{
    CAL_INIT = 0,
    CAL_RANSAC,
    ANGLE_ESTIMATION,
    RESULT_VERIFICATION,
    CAL_STOP
}cal_step_t;

typedef struct static_point_t {
    float angle;
    float doppler;
    float range;
    float snr;
    float residual;
}static_peak_t;

typedef struct adaptive_params_t {
    cal_step_t cal_step;
    uint16_t static_peak_num;
    uint8_t best_ca_index;
    float best_ca_angle;
    int16_t best_inlier_count;
    uint16_t best_candidate_angle[CANDIDATE_ANGLE_NUM];
    float candidate_yaws;
    uint16_t cal_frame;

}adapt_params_t;


typedef struct {
    float confidence_score;      // 总体可信度分数 (0-1)，越高越可信
    uint8_t quality_level;       // 质量等级 (1-5)，5为最佳
    
    // 多维度评估指标
    uint16_t inlier_count;       // 内点数量 - 匹配模型的点数
    float inlier_ratio;          // 内点比例 - 内点数/总有效点数
    float avg_residual;          // 平均残差 - 测量值与预测值的平均误差
    float residual_std;          // 残差标准差 - 残差的离散程度
    float speed_consistency;     // 速度一致性 - 暂未使用，可扩展
    uint16_t point_distribution; // 点云分布质量 - 空间分布得分
    uint8_t frame_num;    // 帧数 每5帧挑选最优一帧
    
    // 环境条件
    float vehicle_speed;         // 车辆速度 km/h - 校准时的车速
    uint16_t valid_point_count;  // 有效点数量 - 通过滤波的点数
    uint8_t is_straight_road;    // 是否直道 - 1=直道，0=弯道
} data_confidence_t;

int16_t Precise_angle_estimation_calibration(float32_t  yaw_angle);
void calibration_conditions_not_met(void);
void init_calibration(void);
bool point_doppler_filter(const float32_t doppler);
bool validate_calibration_result(const static_peak_t *cal_static_Peak,float calibrated_yaw);
float compute_initial_yaw_from_inliers(const static_peak_t *cal_static_Peak);
float compute_gradient(const static_peak_t *cal_static_Peak,float yaw_deg);
float gradient_descent_optimization(const or_point_cloud_format_t *PeakList);
float estimate_yaw_from_all_points(const or_point_cloud_format_t *PeakList);
int16_t collection_internal_point(const or_point_cloud_format_t *PeakList,float candidate_angle);
int16_t count_inliers_for_candidate(const or_point_cloud_format_t *PeakList,float candidate_angle);
int16_t RANSAC_calibration(const or_point_cloud_format_t *PeakList);
void adapt_calibration(const or_point_cloud_format_t *PeakList);
bool point_param_filter(const float32_t azimuth,const float32_t range);

#endif