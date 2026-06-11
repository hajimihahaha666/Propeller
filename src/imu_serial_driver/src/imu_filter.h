#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdbool.h>
#include <stdint.h>

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
    float q[4];
    float integral_fb[3];
    float two_kp;
    float two_ki;
    float inv_sample_freq;

    imu_sample_t output;
    bool initialized;
    int64_t last_update_ns;
} imu_filter_t;

void imu_filter_init(imu_filter_t *filter);
void imu_filter_reset(imu_filter_t *filter);
void imu_filter_set_sample_freq(imu_filter_t *filter, float sample_freq_hz);
imu_sample_t imu_filter_update(imu_filter_t *filter, const imu_sample_t *input, int64_t now_ns);

#endif
