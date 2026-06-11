#include "imu_position.h"

#include <math.h>
#include <stdbool.h>

#define GRAVITY_MS2 9.80665f
#define ACC_LP_ALPHA 0.98f
#define STATIONARY_ACC_DEV_THRESH 0.35f
#define STATIONARY_GYRO_THRESH 0.03f
#define MOTION_ACC_HP_THRESH 0.25f
#define MOTION_GYRO_THRESH 0.06f
#define MAX_VELOCITY 1.0f
#define MAX_POSITION 20.0f
#define POS_SMOOTH_ALPHA 0.12f

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void imu_position_init(imu_position_t *pos)
{
    imu_position_reset(pos);
}

void imu_position_reset(imu_position_t *pos)
{
    pos->pos_x = 0.0f;
    pos->pos_y = 0.0f;
    pos->pos_z = 0.0f;
    pos->vel_x = 0.0f;
    pos->vel_y = 0.0f;
    pos->vel_z = 0.0f;
    pos->acc_lp_x = 0.0f;
    pos->acc_lp_y = 0.0f;
    pos->acc_lp_z = 0.0f;
    pos->display_x = 0.0f;
    pos->display_y = 0.0f;
    pos->display_z = 0.0f;
    pos->last_update_ns = 0;
}

void imu_position_update(imu_position_t *pos, const imu_sample_t *sample, int64_t now_ns)
{
    if (pos->last_update_ns <= 0) {
        pos->last_update_ns = now_ns;
        pos->acc_lp_x = sample->ax;
        pos->acc_lp_y = sample->ay;
        pos->acc_lp_z = sample->az;
        return;
    }

    float dt = (float)(now_ns - pos->last_update_ns) / 1e9f;
    pos->last_update_ns = now_ns;
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    pos->acc_lp_x = ACC_LP_ALPHA * pos->acc_lp_x + (1.0f - ACC_LP_ALPHA) * sample->ax;
    pos->acc_lp_y = ACC_LP_ALPHA * pos->acc_lp_y + (1.0f - ACC_LP_ALPHA) * sample->ay;
    pos->acc_lp_z = ACC_LP_ALPHA * pos->acc_lp_z + (1.0f - ACC_LP_ALPHA) * sample->az;

    const float acc_hp_x = sample->ax - pos->acc_lp_x;
    const float acc_hp_y = sample->ay - pos->acc_lp_y;
    const float acc_hp_z = sample->az - pos->acc_lp_z;

    const float acc_mag = sqrtf(
        sample->ax * sample->ax + sample->ay * sample->ay + sample->az * sample->az);
    const float gyro_mag = sqrtf(
        sample->gx * sample->gx + sample->gy * sample->gy + sample->gz * sample->gz);
    const float acc_hp_mag = sqrtf(acc_hp_x * acc_hp_x + acc_hp_y * acc_hp_y + acc_hp_z * acc_hp_z);
    const float acc_dev = fabsf(acc_mag - GRAVITY_MS2);

    const bool stationary = acc_dev < STATIONARY_ACC_DEV_THRESH && gyro_mag < STATIONARY_GYRO_THRESH;
    const bool moving = acc_hp_mag > MOTION_ACC_HP_THRESH || gyro_mag > MOTION_GYRO_THRESH;

    if (stationary || !moving) {
        pos->vel_x = 0.0f;
        pos->vel_y = 0.0f;
        pos->vel_z = 0.0f;
        pos->pos_x *= 0.95f;
        pos->pos_y *= 0.95f;
        pos->pos_z *= 0.95f;
    } else {
        pos->vel_x += acc_hp_x * dt;
        pos->vel_y += acc_hp_y * dt;
        pos->vel_z += acc_hp_z * dt;

        pos->vel_x = clampf(pos->vel_x, -MAX_VELOCITY, MAX_VELOCITY);
        pos->vel_y = clampf(pos->vel_y, -MAX_VELOCITY, MAX_VELOCITY);
        pos->vel_z = clampf(pos->vel_z, -MAX_VELOCITY, MAX_VELOCITY);

        pos->pos_x += pos->vel_x * dt;
        pos->pos_y += pos->vel_y * dt;
        pos->pos_z += pos->vel_z * dt;

        pos->pos_x = clampf(pos->pos_x, -MAX_POSITION, MAX_POSITION);
        pos->pos_y = clampf(pos->pos_y, -MAX_POSITION, MAX_POSITION);
        pos->pos_z = clampf(pos->pos_z, -MAX_POSITION, MAX_POSITION);
    }

    pos->display_x += (pos->pos_x - pos->display_x) * POS_SMOOTH_ALPHA;
    pos->display_y += (pos->pos_y - pos->display_y) * POS_SMOOTH_ALPHA;
    pos->display_z += (pos->pos_z - pos->display_z) * POS_SMOOTH_ALPHA;
}
