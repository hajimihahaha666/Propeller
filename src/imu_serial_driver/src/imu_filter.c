#include "imu_filter.h"

#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f
#define DEFAULT_DT 0.01f

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

static float angle_diff_deg(float target, float current)
{
    return wrap_angle_deg(target - current);
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = 100.0f;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float sanitize_float(float value, float fallback)
{
    if (isnan(value) || isinf(value)) {
        return fallback;
    }
    return value;
}

float pid_smooth(pid_controller_t *pid, float target, float current, float dt)
{
    target = sanitize_float(target, current);
    current = sanitize_float(current, target);

    if (dt <= 0.0f) {
        return current;
    }

    const float error = target - current;
    pid->integral += error * dt;
    pid->integral = clampf(pid->integral, -pid->integral_limit, pid->integral_limit);

    const float derivative = clampf((error - pid->prev_error) / dt, -80.0f, 80.0f);
    pid->prev_error = error;

    const float output = current + pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return sanitize_float(output, target);
}

void imu_filter_init(imu_filter_t *filter)
{
    pid_init(&filter->acc_x, 0.15f, 0.005f, 0.04f);
    pid_init(&filter->acc_y, 0.15f, 0.005f, 0.04f);
    pid_init(&filter->acc_z, 0.15f, 0.005f, 0.04f);
    pid_init(&filter->gyro_x, 0.12f, 0.003f, 0.035f);
    pid_init(&filter->gyro_y, 0.12f, 0.003f, 0.035f);
    pid_init(&filter->gyro_z, 0.12f, 0.003f, 0.035f);
    pid_init(&filter->roll, 0.45f, 0.015f, 0.08f);
    pid_init(&filter->pitch, 0.45f, 0.015f, 0.08f);
    pid_init(&filter->yaw, 0.40f, 0.012f, 0.06f);

    filter->output = (imu_sample_t){0};
    filter->complementary_alpha = 0.90f;
    filter->angle_blend_beta = 0.85f;
    filter->initialized = false;
    filter->last_update_ns = 0;
}

void imu_filter_reset(imu_filter_t *filter)
{
    pid_reset(&filter->acc_x);
    pid_reset(&filter->acc_y);
    pid_reset(&filter->acc_z);
    pid_reset(&filter->gyro_x);
    pid_reset(&filter->gyro_y);
    pid_reset(&filter->gyro_z);
    pid_reset(&filter->roll);
    pid_reset(&filter->pitch);
    pid_reset(&filter->yaw);
    filter->output = (imu_sample_t){0};
    filter->initialized = false;
    filter->last_update_ns = 0;
}

static float compute_dt(imu_filter_t *filter, int64_t now_ns)
{
    if (filter->last_update_ns <= 0) {
        filter->last_update_ns = now_ns;
        return DEFAULT_DT;
    }

    const float dt = (float)(now_ns - filter->last_update_ns) / 1e9f;
    filter->last_update_ns = now_ns;
    return clampf(dt, 0.001f, 0.1f);
}

static void fuse_roll_pitch(
    imu_filter_t *filter,
    const imu_sample_t *input,
    float dt,
    float *target_roll,
    float *target_pitch)
{
    const float roll_gyro = filter->output.roll + input->gx * RAD_TO_DEG * dt;
    const float pitch_gyro = filter->output.pitch + input->gy * RAD_TO_DEG * dt;

    const float denom = sqrtf(input->ay * input->ay + input->az * input->az);
    const float roll_acc = atan2f(input->ay, input->az) * RAD_TO_DEG;
    const float pitch_acc = atan2f(-input->ax, denom) * RAD_TO_DEG;

    const float alpha = filter->complementary_alpha;
    const float beta = filter->angle_blend_beta;

    float roll_fused = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    float pitch_fused = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;

    roll_fused = beta * roll_fused + (1.0f - beta) * input->roll;
    pitch_fused = beta * pitch_fused + (1.0f - beta) * input->pitch;

    *target_roll = wrap_angle_deg(roll_fused);
    *target_pitch = wrap_angle_deg(pitch_fused);
}

static float fuse_yaw(imu_filter_t *filter, const imu_sample_t *input, float dt)
{
    const float gz = fabsf(input->gz) < 0.008f ? 0.0f : input->gz;
    const float yaw_gyro = wrap_angle_deg(filter->output.yaw + gz * RAD_TO_DEG * dt);
    const float alpha = filter->complementary_alpha;
    return wrap_angle_deg(alpha * yaw_gyro + (1.0f - alpha) * input->yaw);
}

static float apply_gyro_deadband(float value)
{
    return fabsf(value) < 0.008f ? 0.0f : value;
}

imu_sample_t imu_filter_update(imu_filter_t *filter, const imu_sample_t *input, int64_t now_ns)
{
    const float dt = compute_dt(filter, now_ns);

    imu_sample_t smoothed_input = *input;
    smoothed_input.gx = apply_gyro_deadband(smoothed_input.gx);
    smoothed_input.gy = apply_gyro_deadband(smoothed_input.gy);
    smoothed_input.gz = apply_gyro_deadband(smoothed_input.gz);

    if (!filter->initialized) {
        filter->output = smoothed_input;
        filter->initialized = true;
        return filter->output;
    }

    filter->output.ax = pid_smooth(&filter->acc_x, smoothed_input.ax, filter->output.ax, dt);
    filter->output.ay = pid_smooth(&filter->acc_y, smoothed_input.ay, filter->output.ay, dt);
    filter->output.az = pid_smooth(&filter->acc_z, smoothed_input.az, filter->output.az, dt);
    filter->output.gx = pid_smooth(&filter->gyro_x, smoothed_input.gx, filter->output.gx, dt);
    filter->output.gy = pid_smooth(&filter->gyro_y, smoothed_input.gy, filter->output.gy, dt);
    filter->output.gz = pid_smooth(&filter->gyro_z, smoothed_input.gz, filter->output.gz, dt);

    float target_roll = smoothed_input.roll;
    float target_pitch = smoothed_input.pitch;
    fuse_roll_pitch(filter, &smoothed_input, dt, &target_roll, &target_pitch);
    const float target_yaw = fuse_yaw(filter, &smoothed_input, dt);

    filter->output.roll = pid_smooth(
        &filter->roll,
        target_roll,
        filter->output.roll + angle_diff_deg(target_roll, filter->output.roll),
        dt);
    filter->output.roll = wrap_angle_deg(filter->output.roll);

    filter->output.pitch = pid_smooth(
        &filter->pitch,
        target_pitch,
        filter->output.pitch + angle_diff_deg(target_pitch, filter->output.pitch),
        dt);
    filter->output.pitch = wrap_angle_deg(filter->output.pitch);

    filter->output.yaw = pid_smooth(
        &filter->yaw,
        target_yaw,
        filter->output.yaw + angle_diff_deg(target_yaw, filter->output.yaw),
        dt);
    filter->output.yaw = wrap_angle_deg(filter->output.yaw);

    filter->output.ax = sanitize_float(filter->output.ax, smoothed_input.ax);
    filter->output.ay = sanitize_float(filter->output.ay, smoothed_input.ay);
    filter->output.az = sanitize_float(filter->output.az, smoothed_input.az);
    filter->output.gx = sanitize_float(filter->output.gx, smoothed_input.gx);
    filter->output.gy = sanitize_float(filter->output.gy, smoothed_input.gy);
    filter->output.gz = sanitize_float(filter->output.gz, smoothed_input.gz);

    return filter->output;
}
