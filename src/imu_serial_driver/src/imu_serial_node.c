#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_srvs/srv/empty.h>
#include <rosidl_runtime_c/string_functions.h>
#include "imu_filter.h"
#include "imu_position.h"
#define BAUDRATE B9600
#define SERIAL_PORT "/dev/ttyUSB0"
#define TCP_PORT 8888
#define BUFFER_SIZE 1024
#define SERIAL_CACHE_SIZE 512
#define FRAME_HEAD 0x55
#define FRAME_LEN 11
#define ACC_SCALE 16.0f
#define GYRO_SCALE 2000.0f
#define ANGLE_SCALE 180.0f

#define FRAME_TYPE_ACC 0x51
#define FRAME_TYPE_GYRO 0x52
#define FRAME_TYPE_ANGLE 0x53
#define GRAVITY_MS2 9.80665f
#define STARTUP_CALIB_SAMPLES 200

rcl_node_t node;
rcl_publisher_t imu_pub;
rcl_publisher_t euler_pub;
rcl_service_t zero_srv;
sensor_msgs__msg__Imu *imu_msg;
std_msgs__msg__Float64MultiArray *euler_msg;
std_srvs__srv__Empty_Request *zero_req;
std_srvs__srv__Empty_Response *zero_res;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_clock_t system_clock;

static int serial_fd = -1;
static int tcp_listen_fd = -1;
static int tcp_client_fd = -1;

static uint8_t serial_cache[SERIAL_CACHE_SIZE];
static size_t serial_cache_len = 0;

static float raw_ax = 0.0f;
static float raw_ay = 0.0f;
static float raw_az = 0.0f;
static float raw_gx = 0.0f;
static float raw_gy = 0.0f;
static float raw_gz = 0.0f;
static float raw_roll = 0.0f;
static float raw_pitch = 0.0f;
static float raw_yaw = 0.0f;

static float acc_offset_x = 0.0f;
static float acc_offset_y = 0.0f;
static float acc_offset_z = 0.0f;
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;
static float angle_offset_roll = 0.0f;
static float angle_offset_pitch = 0.0f;
static float angle_offset_yaw = 0.0f;

static bool data_updated = false;
static imu_filter_t imu_filter;
static imu_position_t imu_position;

static bool startup_calibrated = false;
static int startup_calib_count = 0;
static double startup_sum_ax = 0.0;
static double startup_sum_ay = 0.0;
static double startup_sum_az = 0.0;
static double startup_sum_gx = 0.0;
static double startup_sum_gy = 0.0;
static double startup_sum_gz = 0.0;
static float yaw_zero_offset = 0.0f;

void imu_zero_calibration(void);
static void reset_position(void);

static float apply_axis_zero(float value, float offset)
{
    return value - offset;
}

static float apply_angle_zero(float angle, float offset)
{
    float result = angle - offset;
    while (result > 180.0f) {
        result -= 360.0f;
    }
    while (result < -180.0f) {
        result += 360.0f;
    }
    return result;
}

static void euler_to_quaternion(float roll_deg, float pitch_deg, float yaw_deg, double q[4])
{
    const float roll = roll_deg * (float)M_PI / 180.0f;
    const float pitch = pitch_deg * (float)M_PI / 180.0f;
    const float yaw = yaw_deg * (float)M_PI / 180.0f;

    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    q[0] = (double)(cr * cp * cy + sr * sp * sy);
    q[1] = (double)(sr * cp * cy - cr * sp * sy);
    q[2] = (double)(cr * sp * cy + sr * cp * sy);
    q[3] = (double)(cr * cp * sy - sr * sp * cy);
}

static int open_serial(const char *port, speed_t baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int setup_tcp_server(uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    if (listen(fd, 1) < 0) {
        close(fd);
        return -1;
    }

    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    return fd;
}

static void get_zeroed_values(
    float *ax, float *ay, float *az,
    float *gx, float *gy, float *gz,
    float *roll, float *pitch, float *yaw)
{
    *ax = apply_axis_zero(raw_ax, acc_offset_x);
    *ay = apply_axis_zero(raw_ay, acc_offset_y);
    *az = apply_axis_zero(raw_az, acc_offset_z);
    *gx = apply_axis_zero(raw_gx, gyro_offset_x);
    *gy = apply_axis_zero(raw_gy, gyro_offset_y);
    *gz = apply_axis_zero(raw_gz, gyro_offset_z);
    *roll = apply_angle_zero(raw_roll, angle_offset_roll);
    *pitch = apply_angle_zero(raw_pitch, angle_offset_pitch);
    *yaw = apply_angle_zero(raw_yaw, angle_offset_yaw);
}

static void collect_startup_calibration_sample(void)
{
    if (startup_calibrated) {
        return;
    }

    startup_sum_ax += raw_ax;
    startup_sum_ay += raw_ay;
    startup_sum_az += raw_az;
    startup_sum_gx += raw_gx;
    startup_sum_gy += raw_gy;
    startup_sum_gz += raw_gz;
    startup_calib_count++;

    if (startup_calib_count < STARTUP_CALIB_SAMPLES) {
        return;
    }

    const float inv = 1.0f / (float)STARTUP_CALIB_SAMPLES;
    acc_offset_x = (float)startup_sum_ax * inv;
    acc_offset_y = (float)startup_sum_ay * inv;
    acc_offset_z = (float)startup_sum_az * inv - GRAVITY_MS2;
    gyro_offset_x = (float)startup_sum_gx * inv;
    gyro_offset_y = (float)startup_sum_gy * inv;
    gyro_offset_z = (float)startup_sum_gz * inv;
    yaw_zero_offset = 0.0f;

    imu_filter_reset(&imu_filter);
    reset_position();
    startup_calibrated = true;

    printf("[INFO] IMU Mahony 启动校准完成 (%d 样本)\n", STARTUP_CALIB_SAMPLES);
    printf("       陀螺零偏 gx=%.4f gy=%.4f gz=%.4f rad/s\n",
           gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

static imu_sample_t get_filtered_values(int64_t now_ns)
{
    imu_sample_t input;
    float unused_roll = 0.0f;
    float unused_pitch = 0.0f;
    float unused_yaw = 0.0f;

    get_zeroed_values(
        &input.ax, &input.ay, &input.az,
        &input.gx, &input.gy, &input.gz,
        &unused_roll, &unused_pitch, &unused_yaw);

    imu_sample_t output = imu_filter_update(&imu_filter, &input, now_ns);
    output.yaw = apply_angle_zero(output.yaw, yaw_zero_offset);
    return output;
}

static void reset_position(void)
{
    imu_position_reset(&imu_position);
}

static void update_position(const imu_sample_t *sample, int64_t now_ns)
{
    imu_position_update(&imu_position, sample, now_ns);
}

static void send_imu_to_tcp_client(const imu_sample_t *filtered)
{
    if (tcp_client_fd < 0) {
        return;
    }

    char payload[512];
    const int len = snprintf(
        payload,
        sizeof(payload),
        "IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        filtered->ax, filtered->ay, filtered->az,
        filtered->gx, filtered->gy, filtered->gz,
        filtered->roll, filtered->pitch, filtered->yaw,
        imu_position.display_x, imu_position.display_y, imu_position.display_z);

    if (len <= 0) {
        return;
    }

    if (write(tcp_client_fd, payload, (size_t)len) < 0) {
        close(tcp_client_fd);
        tcp_client_fd = -1;
        printf("[INFO] 上位机连接已断开\n");
    }
}

static void handle_tcp_commands(void)
{
    if (tcp_listen_fd < 0) {
        return;
    }

    if (tcp_client_fd < 0) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(tcp_listen_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd >= 0) {
            fcntl(client_fd, F_SETFL, fcntl(client_fd, F_GETFL, 0) | O_NONBLOCK);
            tcp_client_fd = client_fd;
            printf("[INFO] 上位机已连接: %s:%d\n",
                   inet_ntoa(client_addr.sin_addr),
                   ntohs(client_addr.sin_port));
        }
    }

    if (tcp_client_fd < 0) {
        return;
    }

    char cmd_buf[64];
    const ssize_t n = read(tcp_client_fd, cmd_buf, sizeof(cmd_buf) - 1);
    if (n > 0) {
        cmd_buf[n] = '\0';
        if (strstr(cmd_buf, "ZERO") != NULL) {
            imu_zero_calibration();
            const char *ack = "OK,ZERO\n";
            write(tcp_client_fd, ack, strlen(ack));
        }
    } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
        close(tcp_client_fd);
        tcp_client_fd = -1;
        printf("[INFO] 上位机连接已断开\n");
    }
}

void imu_zero_calibration(void)
{
    acc_offset_x = raw_ax;
    acc_offset_y = raw_ay;
    acc_offset_z = raw_az - GRAVITY_MS2;
    gyro_offset_x = raw_gx;
    gyro_offset_y = raw_gy;
    gyro_offset_z = raw_gz;
    angle_offset_roll = 0.0f;
    angle_offset_pitch = 0.0f;
    angle_offset_yaw = 0.0f;
    yaw_zero_offset = 0.0f;

    if (serial_fd >= 0) {
        const uint8_t cmd[] = {0xFF, 0xAA, 0x52, 0x00, 0x00};
        write(serial_fd, cmd, sizeof(cmd));
    }

    imu_filter_reset(&imu_filter);
    reset_position();

    printf("[INFO] IMU Mahony 已置零\n");
    printf("       加速度偏移: ax=%.3f ay=%.3f az=%.3f\n", acc_offset_x, acc_offset_y, acc_offset_z);
    printf("       角速度偏移: gx=%.4f gy=%.4f gz=%.4f rad/s\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

static void update_ros_messages(const imu_sample_t *filtered, int64_t now_ns)
{
    imu_msg->linear_acceleration.x = filtered->ax;
    imu_msg->linear_acceleration.y = filtered->ay;
    imu_msg->linear_acceleration.z = filtered->az;
    imu_msg->angular_velocity.x = filtered->gx;
    imu_msg->angular_velocity.y = filtered->gy;
    imu_msg->angular_velocity.z = filtered->gz;

    double quaternion[4];
    euler_to_quaternion(filtered->roll, filtered->pitch, filtered->yaw, quaternion);
    imu_msg->orientation.w = quaternion[0];
    imu_msg->orientation.x = quaternion[1];
    imu_msg->orientation.y = quaternion[2];
    imu_msg->orientation.z = quaternion[3];

    imu_msg->header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
    imu_msg->header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
    rosidl_runtime_c__String__assign(&imu_msg->header.frame_id, "imu_link");

    euler_msg->data.data[0] = (double)filtered->roll;
    euler_msg->data.data[1] = (double)filtered->pitch;
    euler_msg->data.data[2] = (double)filtered->yaw;

    rcl_publish(&imu_pub, imu_msg, NULL);
    rcl_publish(&euler_pub, euler_msg, NULL);
}

static void parse_imu_frame(const uint8_t *frame)
{
    switch (frame[1]) {
    case FRAME_TYPE_ACC: {
        const int16_t ax = (int16_t)((frame[3] << 8) | frame[2]);
        const int16_t ay = (int16_t)((frame[5] << 8) | frame[4]);
        const int16_t az = (int16_t)((frame[7] << 8) | frame[6]);

        raw_ax = ax / 32768.0f * ACC_SCALE * 9.81f;
        raw_ay = ay / 32768.0f * ACC_SCALE * 9.81f;
        raw_az = az / 32768.0f * ACC_SCALE * 9.81f;
        data_updated = true;
        break;
    }
    case FRAME_TYPE_GYRO: {
        const int16_t gx = (int16_t)((frame[3] << 8) | frame[2]);
        const int16_t gy = (int16_t)((frame[5] << 8) | frame[4]);
        const int16_t gz = (int16_t)((frame[7] << 8) | frame[6]);

        raw_gx = gx / 32768.0f * GYRO_SCALE * (float)M_PI / 180.0f;
        raw_gy = gy / 32768.0f * GYRO_SCALE * (float)M_PI / 180.0f;
        raw_gz = gz / 32768.0f * GYRO_SCALE * (float)M_PI / 180.0f;
        data_updated = true;
        break;
    }
    case FRAME_TYPE_ANGLE: {
        const int16_t roll_raw = (int16_t)((frame[3] << 8) | frame[2]);
        const int16_t pitch_raw = (int16_t)((frame[5] << 8) | frame[4]);
        const int16_t yaw_raw = (int16_t)((frame[7] << 8) | frame[6]);

        raw_roll = roll_raw / 32768.0f * ANGLE_SCALE;
        raw_pitch = pitch_raw / 32768.0f * ANGLE_SCALE;
        raw_yaw = yaw_raw / 32768.0f * ANGLE_SCALE;
        data_updated = true;
        break;
    }
    default:
        break;
    }
}

static void feed_serial_data(const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }

    if (serial_cache_len + len > SERIAL_CACHE_SIZE) {
        serial_cache_len = 0;
    }

    memcpy(serial_cache + serial_cache_len, data, len);
    serial_cache_len += len;

    while (serial_cache_len >= FRAME_LEN) {
        size_t head_index = 0;
        while (head_index < serial_cache_len && serial_cache[head_index] != FRAME_HEAD) {
            head_index++;
        }

        if (head_index > 0) {
            memmove(serial_cache, serial_cache + head_index, serial_cache_len - head_index);
            serial_cache_len -= head_index;
        }

        if (serial_cache_len < FRAME_LEN) {
            break;
        }

        uint8_t checksum = 0;
        for (int i = 0; i < 10; i++) {
            checksum += serial_cache[i];
        }

        if (checksum != serial_cache[10]) {
            memmove(serial_cache, serial_cache + 1, serial_cache_len - 1);
            serial_cache_len--;
            continue;
        }

        parse_imu_frame(serial_cache);
        memmove(serial_cache, serial_cache + FRAME_LEN, serial_cache_len - FRAME_LEN);
        serial_cache_len -= FRAME_LEN;
    }
}

void zero_service_callback(const void *req, void *res)
{
    (void)req;
    (void)res;
    imu_zero_calibration();
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    if (serial_fd < 0) {
        serial_fd = open_serial(SERIAL_PORT, BAUDRATE);
        if (serial_fd < 0) {
            printf("[ERROR] 无法打开串口 %s\n", SERIAL_PORT);
            return;
        }
        printf("[INFO] 幻尔 IMU 串口驱动启动成功: %s\n", SERIAL_PORT);
    }

    if (tcp_listen_fd < 0) {
        tcp_listen_fd = setup_tcp_server(TCP_PORT);
        if (tcp_listen_fd < 0) {
            printf("[ERROR] 无法启动 TCP 服务，端口 %d\n", TCP_PORT);
        } else {
            printf("[INFO] 上位机 TCP 服务已启动，端口 %d\n", TCP_PORT);
        }
    }

    handle_tcp_commands();

    uint8_t buf[BUFFER_SIZE];
    const ssize_t n = read(serial_fd, buf, BUFFER_SIZE);
    if (n > 0) {
        feed_serial_data(buf, (size_t)n);
        collect_startup_calibration_sample();
    }

    if (!startup_calibrated) {
        return;
    }

    if (serial_fd >= 0) {
        rcl_time_point_value_t now;
        rcl_clock_get_now(&system_clock, &now);
        const imu_sample_t filtered = get_filtered_values(now);
        update_position(&filtered, now);
        update_ros_messages(&filtered, now);
        send_imu_to_tcp_client(&filtered);
    }
    data_updated = false;
}

int main(int argc, char *argv[])
{
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, argc, (const char *const *)argv, &allocator);
    rcl_clock_init(RCL_SYSTEM_TIME, &system_clock, &allocator);
    rclc_node_init_default(&node, "imu_serial_node", "", &support);

    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw");

    rclc_publisher_init_default(
        &euler_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/imu/euler");

    rclc_service_init_default(
        &zero_srv,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
        "/imu/zero");

    imu_msg = sensor_msgs__msg__Imu__create();
    euler_msg = std_msgs__msg__Float64MultiArray__create();
    zero_req = std_srvs__srv__Empty_Request__create();
    zero_res = std_srvs__srv__Empty_Response__create();
    euler_msg->data.data = (double *)allocator.allocate(3 * sizeof(double), allocator.state);
    euler_msg->data.capacity = 3;
    euler_msg->data.size = 3;

    imu_filter_init(&imu_filter);
    imu_position_init(&imu_position);

    rcl_timer_t timer;
    rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(10),
        timer_callback,
        true);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_service(&executor, &zero_srv, zero_req, zero_res, zero_service_callback);

    rclc_executor_spin(&executor);

    if (tcp_client_fd >= 0) {
        close(tcp_client_fd);
    }
    if (tcp_listen_fd >= 0) {
        close(tcp_listen_fd);
    }
    if (serial_fd >= 0) {
        close(serial_fd);
    }

    rcl_timer_fini(&timer);
    rcl_service_fini(&zero_srv, &node);
    rcl_publisher_fini(&euler_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_node_fini(&node);
    rcl_clock_fini(&system_clock);
    rclc_support_fini(&support);
    sensor_msgs__msg__Imu__destroy(imu_msg);
    std_msgs__msg__Float64MultiArray__destroy(euler_msg);
    std_srvs__srv__Empty_Request__destroy(zero_req);
    std_srvs__srv__Empty_Response__destroy(zero_res);

    return 0;
}
