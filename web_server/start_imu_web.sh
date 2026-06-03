#!/bin/bash
# 一键启动 IMU 驱动 + 网页控制服务

set -e

WS="/home/hansing/imu_ws"

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

echo "[1/2] 启动 IMU 串口驱动..."
ros2 run imu_serial_driver imu_serial_node &
IMU_PID=$!

sleep 2

echo "[2/2] 启动网页控制服务..."
python3 "${WS}/web_server/imu_web_server.py" &
WEB_PID=$!

cleanup() {
    echo
    echo "正在停止服务..."
    kill "$IMU_PID" "$WEB_PID" 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM

echo
echo "=========================================="
echo "  全部服务已启动"
echo "  请在电脑浏览器打开网页地址（见上方输出）"
echo "  按 Ctrl+C 可停止全部服务"
echo "=========================================="

wait
