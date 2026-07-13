"""JACK 水下机器人 8 路推进器混控（SPI 通道 0-7 与电机 1-8 一一对应）。

布局来自 ~/桌面/JACK/urdf/JACK.urdf，机体坐标：X 前、Y 左、Z 上。
前排（X≈+0.445）：电机 1（左）、电机 6（右）。
中排（X≈+0.115）：电机 2（左）、电机 5（右）。
后排（X≈−0.415）：电机 3（左）、电机 4（右）。
水平推进器：电机 7（Y≈−0.605）、电机 8（Y≈+0.605）。

电机 1–6：推力沿 URDF +Z，负责 heave / pitch / roll（上浮下潜与姿态）。
电机 7–8：水平推进，负责 surge / yaw（前进后退与转向）。
"""

from __future__ import annotations

MOTOR_LABELS = (
    "1 前左·升沉Z",
    "2 中左·升沉Z",
    "3 后左·升沉Z",
    "4 后右·升沉Z",
    "5 中右·升沉Z",
    "6 前右·升沉Z",
    "7 水平·右",
    "8 水平·左",
)


def mix_thrusters(
    h: float,
    p: float,
    r: float,
    s: float,
    y: float,
    limit: float,
) -> list[int]:
    """将归一化指令 (-1~1) 混控为 8 路电机百分比 (-limit~limit)。"""
    raw = [
        h - p + r,  # 电机 1 前左 · Z
        h + r,      # 电机 2 中左 · Z
        h + p + r,  # 电机 3 后左 · Z
        h + p - r,  # 电机 4 后右 · Z
        h - r,      # 电机 5 中右 · Z
        h - p - r,  # 电机 6 前右 · Z
        s - y,      # 电机 7 水平（Y 负侧）
        s + y,      # 电机 8 水平（Y 正侧）
    ]
    lim = float(limit)
    return [int(max(-lim, min(lim, v * lim))) for v in raw]
