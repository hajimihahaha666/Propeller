#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
} pid_controller_t;

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float roll;
    float pitch;
    float yaw;
} imu_sample_t;

typedef struct {
    pid_controller_t acc_x;
    pid_controller_t acc_y;
    pid_controller_t acc_z;
    pid_controller_t gyro_x;
    pid_controller_t gyro_y;
    pid_controller_t gyro_z;
    pid_controller_t roll;
    pid_controller_t pitch;
    pid_controller_t yaw;

    imu_sample_t output;
    float complementary_alpha;
    float angle_blend_beta;
    bool initialized;
    int64_t last_update_ns;
} imu_filter_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd);
void pid_reset(pid_controller_t *pid);
float pid_smooth(pid_controller_t *pid, float target, float current, float dt);

void imu_filter_init(imu_filter_t *filter);
void imu_filter_reset(imu_filter_t *filter);
imu_sample_t imu_filter_update(imu_filter_t *filter, const imu_sample_t *input, int64_t now_ns);

#endif
