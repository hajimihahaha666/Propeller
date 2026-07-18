#!/bin/bash
# 后台守护：IMU 驱动 + 网页控制（systemd 用户服务，网线直连无需外网）

set -eo pipefail

WS="/home/hansing/imu_ws"
LOG_DIR="/tmp"
IMU_LOG="${LOG_DIR}/imu_serial.log"
WEB_LOG="${LOG_DIR}/imu_web.log"
CAM_LOG="${LOG_DIR}/cam.log"
CAM_SCRIPT="/home/hansing/camera_stream.py"
PI_IP="192.168.50.1"

log() {
    echo "[$(date '+%F %T')] $*" >> "${WEB_LOG}"
}

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

log "propeller 守护进程启动"

# 等待 eth0 固定直连地址（不依赖 WiFi / 外网 / network-online）
log "等待 eth0 直连网络 ${PI_IP}..."
eth_ready=0
for _ in $(seq 1 60); do
    if ip -4 addr show dev eth0 2>/dev/null | grep -q "${PI_IP}/"; then
        eth_ready=1
        log "eth0 已就绪: ${PI_IP}"
        break
    fi
    sleep 1
done
if [ "${eth_ready}" -eq 0 ]; then
    log "[WARN] eth0 未获得 ${PI_IP}，仍继续启动（请运行 setup_eth_direct.sh）"
fi

# SPI 设备（推进器经 STM32）
for _ in $(seq 1 20); do
    [ -e /dev/spidev0.0 ] && break
    sleep 1
done
if [ ! -e /dev/spidev0.0 ]; then
    log "[WARN] /dev/spidev0.0 未找到"
fi

# IMU USB（可选）
imu_ready=0
for _ in $(seq 1 45); do
    if [ -e /dev/ttyUSB0 ]; then
        imu_ready=1
        break
    fi
    sleep 1
done

pkill -f imu_serial_node 2>/dev/null || true
pkill -f imu_web_server.py 2>/dev/null || true
sleep 1

imu_pid=""
if [ "${imu_ready}" -eq 1 ]; then
    log "启动 IMU 串口驱动..."
    echo "[$(date)] 启动 IMU 串口驱动..." >> "${IMU_LOG}"
    ros2 run imu_serial_driver imu_serial_node >> "${IMU_LOG}" 2>&1 &
    imu_pid=$!
    sleep 2
else
    log "[WARN] /dev/ttyUSB0 未就绪，跳过 IMU（推进器网页仍可用）"
fi

log "启动网页服务 http://${PI_IP}:8080 ..."
python3 "${WS}/web_server/imu_web_server.py" >> "${WEB_LOG}" 2>&1 &
web_pid=$!

# 摄像头不自启（供电不足以同时扛 WiFi+摄像头），由网页按钮手动启动
cam_pid=""
log "摄像头未自启，请在网页点击「启动摄像头」"

sleep 3
if curl -sf -m 5 "http://127.0.0.1:8080/api/status" >/dev/null; then
    log "网页服务就绪"
else
    log "[ERROR] 网页未响应，查看 ${WEB_LOG}"
fi

cleanup() {
    log "停止 propeller 服务..."
    [ -n "${imu_pid}" ] && kill "${imu_pid}" 2>/dev/null || true
    [ -n "${cam_pid}" ] && kill "${cam_pid}" 2>/dev/null || true
    kill "${web_pid}" 2>/dev/null || true
    pkill -f imu_serial_node 2>/dev/null || true
    pkill -f imu_web_server.py 2>/dev/null || true
    pkill -f camera_stream.py 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM

# 只看守网页/IMU；摄像头由网页 API 管理，退出不拖垮整体服务
wait -n ${imu_pid} ${web_pid} 2>/dev/null || wait "${web_pid}"
cleanup
