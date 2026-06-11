#include "imu_filter.h"

#include <math.h>

#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.0174532925f
#define DEFAULT_DT 0.01f
#define GRAVITY_MS2 9.80665f

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

static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void mahony_get_euler(const imu_filter_t *filter, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float q0 = filter->q[0];
    const float q1 = filter->q[1];
    const float q2 = filter->q[2];
    const float q3 = filter->q[3];

    const float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    const float sinp = 2.0f * (q0 * q2 - q3 * q1);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf((float)M_PI / 2.0f, sinp);
    } else {
        pitch = asinf(sinp);
    }
    const float yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    *roll_deg = roll * RAD_TO_DEG;
    *pitch_deg = pitch * RAD_TO_DEG;
    *yaw_deg = yaw * RAD_TO_DEG;
}

void imu_filter_init(imu_filter_t *filter)
{
    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    filter->integral_fb[0] = 0.0f;
    filter->integral_fb[1] = 0.0f;
    filter->integral_fb[2] = 0.0f;
    filter->two_kp = 2.0f * 0.8f;
    filter->two_ki = 2.0f * 0.002f;
    filter->inv_sample_freq = DEFAULT_DT;
    filter->output = (imu_sample_t){0};
    filter->initialized = false;
    filter->last_update_ns = 0;
}

void imu_filter_reset(imu_filter_t *filter)
{
    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    filter->integral_fb[0] = 0.0f;
    filter->integral_fb[1] = 0.0f;
    filter->integral_fb[2] = 0.0f;
    filter->output = (imu_sample_t){0};
    filter->initialized = false;
    filter->last_update_ns = 0;
}

void imu_filter_set_sample_freq(imu_filter_t *filter, float sample_freq_hz)
{
    if (sample_freq_hz <= 1.0f) {
        return;
    }
    filter->inv_sample_freq = 1.0f / sample_freq_hz;
}

static float compute_dt(imu_filter_t *filter, int64_t now_ns)
{
    if (filter->last_update_ns <= 0) {
        filter->last_update_ns = now_ns;
        return filter->inv_sample_freq;
    }

    const float dt = (float)(now_ns - filter->last_update_ns) / 1e9f;
    filter->last_update_ns = now_ns;
    return clampf(dt, 0.001f, 0.05f);
}

static void mahony_update(imu_filter_t *filter, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float q0 = filter->q[0];
    float q1 = filter->q[1];
    float q2 = filter->q[2];
    float q3 = filter->q[3];

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        return;
    }

    const float inv_norm = 1.0f / norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    const float acc_mag = norm;
    float two_kp = filter->two_kp;
    if (fabsf(acc_mag - GRAVITY_MS2) > 1.2f) {
        two_kp *= 0.15f;
    }

    const float vx = 2.0f * (q1 * q3 - q0 * q2);
    const float vy = 2.0f * (q0 * q1 + q2 * q3);
    const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    const float half_t = dt * 0.5f;

    if (filter->two_ki > 0.0f) {
        filter->integral_fb[0] += filter->two_ki * ex * half_t;
        filter->integral_fb[1] += filter->two_ki * ey * half_t;
        filter->integral_fb[2] += filter->two_ki * ez * half_t;
        filter->integral_fb[0] = clampf(filter->integral_fb[0], -0.1f, 0.1f);
        filter->integral_fb[1] = clampf(filter->integral_fb[1], -0.1f, 0.1f);
        filter->integral_fb[2] = clampf(filter->integral_fb[2], -0.1f, 0.1f);
        gx += filter->integral_fb[0];
        gy += filter->integral_fb[1];
        gz += filter->integral_fb[2];
    }

    gx += two_kp * ex;
    gy += two_kp * ey;
    gz += two_kp * ez;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * half_t;

    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm < 1e-6f) {
        return;
    }
    const float inv_q = 1.0f / norm;
    filter->q[0] = q0 * inv_q;
    filter->q[1] = q1 * inv_q;
    filter->q[2] = q2 * inv_q;
    filter->q[3] = q3 * inv_q;
}

imu_sample_t imu_filter_update(imu_filter_t *filter, const imu_sample_t *input, int64_t now_ns)
{
    const float dt = compute_dt(filter, now_ns);
    filter->inv_sample_freq = dt;

    filter->output.ax = input->ax;
    filter->output.ay = input->ay;
    filter->output.az = input->az;
    filter->output.gx = input->gx;
    filter->output.gy = input->gy;
    filter->output.gz = input->gz;

    if (!filter->initialized) {
        mahony_update(filter, input->gx, input->gy, input->gz, input->ax, input->ay, input->az, dt);
        filter->initialized = true;
    } else {
        mahony_update(filter, input->gx, input->gy, input->gz, input->ax, input->ay, input->az, dt);
    }

    mahony_get_euler(filter, &filter->output.roll, &filter->output.pitch, &filter->output.yaw);
    filter->output.roll = wrap_angle_deg(filter->output.roll);
    filter->output.pitch = wrap_angle_deg(filter->output.pitch);
    filter->output.yaw = wrap_angle_deg(filter->output.yaw);

    return filter->output;
}
