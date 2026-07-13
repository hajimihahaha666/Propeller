#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

echo "=== 编译 STM32 固件 ==="
make clean
make -j"$(nproc)"

BIN="$ROOT/build/pai_stm32f103c8t6.bin"
if [[ ! -f "$BIN" ]]; then
  echo "错误: 未找到 $BIN"
  exit 1
fi

echo "=== 停止 propeller 服务（释放 SPI/ST-Link）==="
echo "200655" | sudo -S systemctl stop propeller.service 2>/dev/null || true
sleep 1

echo "=== OpenOCD 烧录 ==="
echo "200655" | sudo -S openocd \
  -f /usr/share/openocd/scripts/interface/stlink.cfg \
  -f /usr/share/openocd/scripts/target/stm32f1x.cfg \
  -c "program $BIN 0x08000000 verify reset exit"

echo "=== 重启 propeller 服务 ==="
echo "200655" | sudo -S systemctl start propeller.service 2>/dev/null || true

echo "烧录完成: $BIN"
