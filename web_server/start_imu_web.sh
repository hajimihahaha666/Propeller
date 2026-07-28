#!/bin/bash
# 一键启动 IMU 驱动 + 网页控制服务

set -e

WS="/home/han/imu_ws"

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

# 等待 IMU 串口设备就绪（开机时 USB 可能尚未枚举）
for i in $(seq 1 30); do
    [ -e /dev/ttyUSB0 ] && break
    sleep 1
done

echo "[1/2] 启动 IMU 串口驱动..."
ros2 run imu_serial_driver imu_serial_node &
IMU_PID=$!

sleep 2

echo "[2/2] 启动网页控制服务..."
python3 "${WS}/web_server/imu_web_server.py" > /tmp/imu_web.log 2>&1 &
WEB_PID=$!

sleep 2

LOCAL_IP="$(hostname -I | awk '{print $1}')"
if curl -s -m 2 -o /dev/null "http://127.0.0.1:8080/"; then
    WEB_OK=1
else
    WEB_OK=0
    echo "[ERROR] 网页服务启动失败，日志: /tmp/imu_web.log"
    tail -20 /tmp/imu_web.log 2>/dev/null || true
fi

cleanup() {
    echo
    echo "正在停止服务..."
    kill "$IMU_PID" "$WEB_PID" 2>/dev/null || true
    pkill -f imu_serial_node 2>/dev/null || true
    pkill -f imu_web_server.py 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM

echo
echo "=========================================="
echo "  全部服务已启动"
if [ "$WEB_OK" -eq 1 ]; then
    echo "  网页: http://${LOCAL_IP}:8080"
    echo "  本机: http://127.0.0.1:8080"
else
    echo "  网页服务未就绪，请查看 /tmp/imu_web.log"
fi
echo "  按 Ctrl+C 可停止全部服务"
echo "=========================================="

wait
