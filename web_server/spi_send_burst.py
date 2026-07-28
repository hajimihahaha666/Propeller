#!/usr/bin/env python3
"""独立进程 SPI 下发（与 imu_web_server 分离，共用 esc_spi 的总线锁串行访问）。

帧格式、SPI 参数、总线锁全部复用 esc_spi，避免两处实现漂移 ——
历史上这里写死 500kHz 而服务用 200kHz，等于同一条总线上跑两套参数。
"""
from __future__ import annotations

import json
import sys
import time

try:
    from esc_spi import ESC_SPI, FRAME_GAP_SEC, SPI_SPEED_HZ, spi_bus_lock
except ImportError:
    print("no spidev", file=sys.stderr)
    sys.exit(2)


def main() -> int:
    raw = sys.argv[1] if len(sys.argv) > 1 else sys.stdin.read()
    channels = json.loads(raw)
    if not isinstance(channels, list) or len(channels) != 8:
        print("need 8 channels", file=sys.stderr)
        return 1

    burst = int(sys.argv[2]) if len(sys.argv) > 2 else 12

    # 整轮发送包在一把锁里：守护进程此刻不会插进来，STM32 收到的帧不会交错
    with spi_bus_lock():
        esc = ESC_SPI(bus=0, device=0, max_speed_hz=SPI_SPEED_HZ)
        try:
            esc.set_all(channels)
            for _ in range(max(1, burst)):
                esc.send_frame()
                time.sleep(max(FRAME_GAP_SEC, 0.003))
        finally:
            esc.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
