"""IMU 反馈 PID 控制器：根据姿态误差计算推进器修正量。"""

from __future__ import annotations

import math


def wrap_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def angle_error_deg(setpoint: float, measurement: float) -> float:
    return wrap_angle_deg(setpoint - measurement)


class PIDAxis:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float = 0.5,
        output_limit: float = 1.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0

    def update(self, error: float, dt: float, rate: float = 0.0) -> float:
        if dt <= 0.0:
            return self.last_output

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        derivative = rate if rate != 0.0 else (error - self.last_error) / dt
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        self.last_output = output
        return output


class ImuThrusterController:
    """根据 IMU 姿态与角速度，输出 roll/pitch/yaw/heave 修正指令（-1~1）。"""

    def __init__(self) -> None:
        self.roll_pid = PIDAxis(kp=0.035, ki=0.002, kd=0.12, output_limit=1.0)
        self.pitch_pid = PIDAxis(kp=0.035, ki=0.002, kd=0.12, output_limit=1.0)
        self.yaw_pid = PIDAxis(kp=0.025, ki=0.001, kd=0.08, output_limit=1.0)
        self.heave_pid = PIDAxis(kp=0.02, ki=0.001, kd=0.05, output_limit=1.0)

        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.target_heave = 0.0

        self.last_cmd = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "heave": 0.0}

    def reset(self) -> None:
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.heave_pid.reset()
        self.last_cmd = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "heave": 0.0}

    def set_targets(
        self,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        heave: float | None = None,
    ) -> None:
        if roll is not None:
            self.target_roll = roll
        if pitch is not None:
            self.target_pitch = pitch
        if yaw is not None:
            self.target_yaw = yaw
        if heave is not None:
            self.target_heave = heave

    def compute(
        self,
        imu: dict[str, float],
        dt: float,
        hold_depth: bool = False,
    ) -> dict[str, float]:
        roll_err = angle_error_deg(self.target_roll, imu["roll"])
        pitch_err = angle_error_deg(self.target_pitch, imu["pitch"])
        yaw_err = angle_error_deg(self.target_yaw, imu["yaw"])

        roll_out = self.roll_pid.update(roll_err, dt, rate=-imu["gx"])
        pitch_out = self.pitch_pid.update(pitch_err, dt, rate=-imu["gy"])
        yaw_out = self.yaw_pid.update(yaw_err, dt, rate=-imu["gz"])

        heave_out = 0.0
        if hold_depth:
            heave_err = self.target_heave - imu.get("pos_z", 0.0)
            heave_out = self.heave_pid.update(heave_err, dt, rate=-imu.get("az", 0.0) * 0.1)

        self.last_cmd = {
            "roll": roll_out,
            "pitch": pitch_out,
            "yaw": yaw_out,
            "heave": heave_out,
        }
        return dict(self.last_cmd)
