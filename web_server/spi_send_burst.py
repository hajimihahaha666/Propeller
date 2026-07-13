#!/usr/bin/env python3
"""独立进程 SPI 下发（与 imu_web_server 分离，fcntl 文件锁串行访问总线）。"""
from __future__ import annotations

import fcntl
import json
import struct
import sys
import time
from pathlib import Path

try:
    import spidev
except ImportError:
    print("no spidev", file=sys.stderr)
    sys.exit(2)

LOCK_FILE = Path("/tmp/propeller_spi.lock")


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(channels: list[int]) -> list[int]:
    frame = bytearray([0xAA, 0x01, 16])
    for v in channels:
        frame += struct.pack("<h", int(v))
    frame.extend([0] * 6)
    frame.append(crc8(frame))
    return list(frame)


def main() -> int:
    raw = sys.argv[1] if len(sys.argv) > 1 else sys.stdin.read()
    channels = json.loads(raw)
    if not isinstance(channels, list) or len(channels) != 8:
        print("need 8 channels", file=sys.stderr)
        return 1

    burst = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    LOCK_FILE.touch(exist_ok=True)
    with LOCK_FILE.open("w") as lockf:
        fcntl.flock(lockf, fcntl.LOCK_EX)
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 500000
        spi.mode = 0
        frame = build_frame(channels)
        for _ in range(max(1, burst)):
            spi.xfer2(frame)
            time.sleep(0.003)
        spi.close()
        fcntl.flock(lockf, fcntl.LOCK_UN)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
