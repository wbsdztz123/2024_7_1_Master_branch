
#ifndef CALIBRATION_H
#define CALIBRATION_H


#include "commapi.h"
#include "calibration_common.h"
#define MAX_STATIC_PEAK_NUM 50
#define  RANSAC_THRESHOLD  0.08f // 内点筛选阈值

#define MIN_INLIER_COUNT 20
#define MIN_BEST_INLIER_COUNT 20


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
}static_peak_t;

typedef struct adaptive_params_t {
    uint8_t cal_step;
    uint16_t static_peak_num;
    uint8_t best_ca_index;
    float best_ca_angle;
    int16_t best_inlier_count;
    float candidate_yaws;

}adapt_params_t;

void calibration_conditions_not_met(void);
void init_calibration(void);
bool point_doppler_filter(const or_point_cloud_term_t *point_cloud);
bool validate_calibration_result(const static_peak_t *cal_static_Peak,float calibrated_yaw);
float compute_initial_yaw_from_inliers(const static_peak_t *cal_static_Peak);
float compute_gradient(const static_peak_t *cal_static_Peak,float yaw_deg);
float gradient_descent_optimization(const or_point_cloud_format_t *PeakList);
float estimate_yaw_from_all_points(const or_point_cloud_format_t *PeakList);
int16_t collection_internal_point(const or_point_cloud_format_t *PeakList,float candidate_angle);
int16_t count_inliers_for_candidate(const or_point_cloud_format_t *PeakList,float candidate_angle);
int16_t RANSAC_calibration(const or_point_cloud_format_t *PeakList);
void adapt_calibration(const or_point_cloud_format_t *PeakList);


#endif