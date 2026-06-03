#ifndef IMU_POSITION_H
#define IMU_POSITION_H

#include <stdint.h>

#include "imu_filter.h"

typedef struct {
    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float acc_lp_x;
    float acc_lp_y;
    float acc_lp_z;
    float display_x;
    float display_y;
    float display_z;
    int64_t last_update_ns;
} imu_position_t;

void imu_position_init(imu_position_t *pos);
void imu_position_reset(imu_position_t *pos);
void imu_position_update(imu_position_t *pos, const imu_sample_t *sample, int64_t now_ns);

#endif
