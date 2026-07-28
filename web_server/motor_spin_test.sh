#!/bin/bash
# 绕过网页，直接测试：1-6 升沉Z，7-8 前进/转向
set -euo pipefail
SCRIPT="/home/han/imu_ws/web_server/spi_send_burst.py"

echo "=== 1) 中位解锁 3 秒 ==="
for _ in $(seq 1 15); do
  python3 "$SCRIPT" "[0,0,0,0,0,0,0,0]" 2
  sleep 0.2
done

echo "=== 2) 升沉电机 1-6 最大 5 秒（应上浮/下潜）==="
for _ in $(seq 1 25); do
  python3 "$SCRIPT" "[100,100,100,100,100,100,0,0]" 2
  sleep 0.2
done

echo "=== 3) 水平电机 7-8 最大 3 秒（应前进/可差动转向）==="
for _ in $(seq 1 15); do
  python3 "$SCRIPT" "[0,0,0,0,0,0,100,100]" 2
  sleep 0.2
done

echo "=== 4) 归零 ==="
python3 "$SCRIPT" "[0,0,0,0,0,0,0,0]" 3
echo "完成。通道 0-7 = 电机 1-8。"
