#!/usr/bin/env python3
"""IMU 反馈闭环 + 网页推进器控制服务"""

from __future__ import annotations

import fcntl
import json
import math
import mimetypes
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from imu_controller import ImuThrusterController, wrap_angle_deg
from thruster_mixer import mix_thrusters

try:
    from esc_spi import ESC_SPI
except ImportError:
    ESC_SPI = None
    print("[WARN] spidev 未安装，SPI 推进器仅模拟输出（请: sudo apt install python3-spidev）")

STATIC_DIR = Path(__file__).resolve().parent / "static"
ZERO_PROFILES_FILE = Path(__file__).resolve().parent / "zero_profiles.json"
IMU_TCP_HOST = "127.0.0.1"
IMU_TCP_PORT = 8888
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

CONTROL_MODES = ("manual", "imu_hold", "hybrid")
individual_motor_active = False
individual_motor_values = [0.0] * 8
individual_motor_lock = threading.Lock()

raw_state = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
}

DEFAULT_ZERO_PROFILE = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
}


def load_zero_profiles() -> dict[str, dict[str, float]]:
    profiles = {
        name: dict(DEFAULT_ZERO_PROFILE)
        for name in ("A", "B", "C")
    }
    active = "A"
    if ZERO_PROFILES_FILE.is_file():
        try:
            data = json.loads(ZERO_PROFILES_FILE.read_text(encoding="utf-8"))
            for name in ("A", "B", "C"):
                if name in data.get("profiles", {}):
                    profiles[name].update(data["profiles"][name])
            active = data.get("active", "A")
            if active not in profiles:
                active = "A"
        except (json.JSONDecodeError, OSError, TypeError, ValueError) as exc:
            print(f"[WARN] 零位配置读取失败，使用默认值: {exc}")
    return profiles, active


def save_zero_profiles() -> None:
    payload = {
        "active": state["active_zero_profile"],
        "profiles": zero_profiles,
    }
    try:
        ZERO_PROFILES_FILE.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except OSError as exc:
        print(f"[WARN] 零位配置保存失败: {exc}")


zero_profiles, _active_profile = load_zero_profiles()

state: dict[str, object] = {
    "connected": False,
    "ax": 0.0, "ay": 0.0, "az": 0.0,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
    "active_zero_profile": _active_profile,
    "control_mode": "manual",
    "imu_cmd_roll": 0.0,
    "imu_cmd_pitch": 0.0,
    "imu_cmd_yaw": 0.0,
    "imu_cmd_heave": 0.0,
    "thruster_cmd_roll": 0.0,
    "thruster_cmd_pitch": 0.0,
    "thruster_cmd_yaw": 0.0,
    "thruster_cmd_heave": 0.0,
    "thruster_cmd_surge": 0.0,
    "spi_active": False,
    "spi_tx_count": 0,
    "spi_err_count": 0,
    "last_spi_channels": [0] * 8,
}

state_lock = threading.Lock()
imu_sock: socket.socket | None = None
imu_sock_lock = threading.Lock()

manual_cmd = {
    "heave": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "surge": 0.0,
    "yaw": 0.0,
    "speed_mode": "medium",
}
manual_lock = threading.Lock()
last_manual_time = time.time()
manual_holding = False  # 前端按键按住时为 True，松开即 False → 手动模式立即停转
# 仅在 holding 期间防止偶发零帧覆盖（松开时 holding=False 会清空 sticky）
manual_cmd_sticky: dict[str, tuple[float, float]] = {}
MANUAL_STICKY_SEC = 0.15

control_mode = "manual"
control_mode_lock = threading.Lock()

imu_controller = ImuThrusterController()
last_control_tick = time.time()

esc_instance = None
spi_tx_count = 0
spi_err_count = 0
last_spi_channels = [0] * 8
last_pushed_channels: list[int] | None = None
last_spi_send_time = 0.0
SPI_SPEED_HZ = 200_000
SPI_FRAME_GAP_SEC = 0.001
CONTROL_LOOP_SEC = 0.025  # 40Hz
IDLE_KEEPALIVE_SEC = 1.0
spi_lock = threading.Lock()  # 进程内串行

# 跨进程 SPI 总线锁：与 spi_send_burst.py 用同一把文件锁，保证守护进程与任何
# 测试脚本永不同时占用 /dev/spidev0.0。总线争用会让 STM32 收到交错帧、校验失败，
# 固件每累计 200 次失败自动切一次 SPI 模式 → 与 Pi 固定 mode 0 永久错位、无法自愈。
SPI_LOCK_FILE = Path("/tmp/propeller_spi.lock")
_spi_lock_fh = None


def _spi_bus_lock():
    global _spi_lock_fh
    if _spi_lock_fh is None:
        SPI_LOCK_FILE.touch(exist_ok=True)
        _spi_lock_fh = SPI_LOCK_FILE.open("w")
    return _spi_lock_fh


# 逻辑通道 -> 物理通道 的置换：物理[i] = 逻辑[CHANNEL_WIRING[i]]。
# 接线为自然映射（通道 index i → 电机 i+1）：A8→M1, A9→M2, A10→M3, A11→M4,
# B6→M5, B7→M6, B8→M7, B9→M8（用户确认 A9 接电机 2）。故不做任何换位。
# 若日后确实发现某两路接反，只需在此表对调对应下标。
CHANNEL_WIRING = [0, 1, 2, 3, 4, 5, 6, 7]


def apply_wiring(channels: list[int]) -> list[int]:
    return [channels[i] for i in CHANNEL_WIRING]
if ESC_SPI:
    try:
        Path("/dev/spidev0.0").resolve()
        state["spi_active"] = True
        print("[INFO] SPI 设备就绪 (40Hz 控制环 + 指令变化立即下发)")
    except OSError as exc:
        state["spi_active"] = False
        print(f"[WARN] SPI 设备不可用: {exc}")


def spi_get() -> "ESC_SPI | None":
    """持久 SPI 连接，避免每次 open/close 增加 tens of ms 延迟。"""
    global esc_instance
    if ESC_SPI is None:
        return None
    if esc_instance is None:
        esc_instance = ESC_SPI(bus=0, device=0, max_speed_hz=SPI_SPEED_HZ, mode=0)
    return esc_instance


def spi_close() -> None:
    global esc_instance
    if esc_instance is not None:
        try:
            esc_instance.close()
        except OSError:
            pass
        esc_instance = None


def arm_escs_at_boot() -> None:
    """服务启动后先发 3s 中位 PWM，帮助电调从自检/掉电状态完成解锁。"""
    if ESC_SPI is None:
        return
    print("[INFO] 电调解锁：发送 3s 中位 PWM ...")
    for _ in range(30):
        spi_push_channels([0] * 8, burst=2)
        time.sleep(0.1)
    print("[INFO] 电调解锁完成，可以控制")


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))


def compute_channels_now() -> list[int]:
    """按当前状态立即计算 8 路 SPI 通道值。"""
    speed_limits = {"slow": 25.0, "medium": 50.0, "fast": 80.0}
    with individual_motor_lock:
        if individual_motor_active:
            return [int(clamp(v, -100, 100)) for v in individual_motor_values]
    with manual_lock:
        limit = speed_limits.get(manual_cmd["speed_mode"], 50.0)
    h, p, r, s, y = compute_thruster_commands()
    return mix_thrusters(h, p, r, s, y, limit)


def spi_push_channels(channels: list[int], *, burst: int = 1) -> None:
    """向 STM32 下发 SPI 帧。持久连接 + 短帧间隔。"""
    global spi_tx_count, spi_err_count, last_spi_channels, last_spi_send_time, last_pushed_channels
    if ESC_SPI is None:
        return
    last_spi_channels = list(channels)          # 显示用：逻辑通道（重映射前）
    phys_channels = apply_wiring(channels)      # 实际下发：物理通道（重映射后）
    ok = False
    with spi_lock:
        lockf = _spi_bus_lock()
        fcntl.flock(lockf, fcntl.LOCK_EX)       # 跨进程独占总线
        try:
            esc = spi_get()
            if esc is None:
                return
            esc.set_all(phys_channels)
            for _ in range(max(1, burst)):
                esc.send_frame()
                if burst > 1:
                    time.sleep(SPI_FRAME_GAP_SEC)
            spi_tx_count += 1
            last_spi_send_time = time.time()
            last_pushed_channels = list(channels)
            ok = True
        except OSError as exc:
            ok = False
            spi_err_count += 1
            spi_close()
            if spi_err_count <= 5 or spi_err_count % 50 == 0:
                print(f"[ERROR] SPI 发送失败 #{spi_err_count}: {exc}")
        finally:
            fcntl.flock(lockf, fcntl.LOCK_UN)
    with state_lock:
        state["spi_tx_count"] = spi_tx_count
        state["spi_err_count"] = spi_err_count
        state["last_spi_channels"] = list(channels)
        if ok:
            state["spi_active"] = True


def push_control_spi_now() -> None:
    """HTTP 控制请求到达后立即混控并下发 SPI（不等待后台线程）。"""
    channels = compute_channels_now()
    prev = last_pushed_channels
    if prev is None or prev != channels:
        spi_push_channels(channels, burst=1)


def compute_thruster_commands() -> tuple[float, float, float, float, float]:
    """根据控制模式，融合 IMU 反馈与手动指令，输出 h,p,r,s,y。"""
    global last_control_tick

    now = time.time()
    dt = clamp(now - last_control_tick, 0.001, 0.1)
    last_control_tick = now

    with control_mode_lock:
        mode = control_mode

    with state_lock:
        imu_connected = bool(state["connected"])
        imu = {
            "roll": float(state["roll"]),
            "pitch": float(state["pitch"]),
            "yaw": float(state["yaw"]),
            "gx": float(state["gx"]),
            "gy": float(state["gy"]),
            "gz": float(state["gz"]),
            "az": float(state["az"]),
            "pos_z": float(state["pos_z"]),
        }

    with manual_lock:
        manual = dict(manual_cmd)
        manual_age = now - last_manual_time
        holding = manual_holding
        now_ts = now
        if holding:
            for key in ("heave", "pitch", "roll", "surge", "yaw"):
                sticky = manual_cmd_sticky.get(key)
                if sticky and now_ts < sticky[1] and abs(manual[key]) < 0.01:
                    manual[key] = sticky[0]

    h = manual["heave"]
    p = manual["pitch"]
    r = manual["roll"]
    s = manual["surge"]
    y = manual["yaw"]

    imu_cmd = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "heave": 0.0}

    if mode == "imu_hold" and imu_connected:
        imu_controller.set_targets(0.0, 0.0, 0.0, 0.0)
        imu_cmd = imu_controller.compute(imu, dt, hold_depth=False)
        p = imu_cmd["pitch"]
        r = imu_cmd["roll"]
        y = imu_cmd["yaw"]
        h = manual["heave"]

    elif mode == "hybrid" and imu_connected:
        imu_controller.set_targets(
            roll=manual["roll"] * 30.0,
            pitch=manual["pitch"] * 30.0,
            yaw=manual["yaw"] * 45.0,
            heave=manual["heave"] * 0.5,
        )
        imu_cmd = imu_controller.compute(imu, dt, hold_depth=abs(manual["heave"]) < 0.05)
        p = imu_cmd["pitch"]
        r = imu_cmd["roll"]
        y = imu_cmd["yaw"]
        h = imu_cmd["heave"] if abs(manual["heave"]) < 0.05 else manual["heave"]
        s = manual["surge"]

    elif mode == "manual":
        imu_controller.reset()
        if not holding:
            h = p = r = s = y = 0.0

    with state_lock:
        state["control_mode"] = mode
        state["imu_cmd_roll"] = imu_cmd["roll"]
        state["imu_cmd_pitch"] = imu_cmd["pitch"]
        state["imu_cmd_yaw"] = imu_cmd["yaw"]
        state["imu_cmd_heave"] = imu_cmd["heave"]
        state["thruster_cmd_roll"] = r
        state["thruster_cmd_pitch"] = p
        state["thruster_cmd_yaw"] = y
        state["thruster_cmd_heave"] = h
        state["thruster_cmd_surge"] = s

    return h, p, r, s, y


def update_thrusters() -> None:
    """混控计算 + SPI 下发：40Hz；通道变化或松键归零立即发送。"""
    global last_pushed_channels
    while True:
        channels = compute_channels_now()

        with individual_motor_lock:
            indi_active = individual_motor_active
            indi_values = list(individual_motor_values)
        with state_lock:
            state["individual_motor_active"] = indi_active
            state["individual_motor_values"] = indi_values
            state["last_spi_channels"] = list(channels)

        prev = last_pushed_channels
        changed = prev is None or prev != channels
        now = time.time()
        moving = any(ch != 0 for ch in channels)
        if changed:
            spi_push_channels(channels, burst=1)
        elif moving:
            spi_push_channels(channels, burst=1)
        elif now - last_spi_send_time >= IDLE_KEEPALIVE_SEC:
            spi_push_channels([0] * 8, burst=1)

        time.sleep(CONTROL_LOOP_SEC)


def update_state_from_line(line: str) -> None:
    parts = line.strip().split(",")
    if parts[0] != "IMU":
        return

    try:
        values = [
            0.0 if math.isnan(float(v)) or math.isinf(float(v)) else float(v)
            for v in parts[1:]
        ]
    except ValueError:
        return

    with state_lock:
        state["connected"] = True
        if len(values) >= 9:
            state["ax"], state["ay"], state["az"] = values[0:3]
            state["gx"], state["gy"], state["gz"] = values[3:6]
            raw_state["roll"], raw_state["pitch"], raw_state["yaw"] = values[6:9]
        if len(values) >= 12:
            raw_state["pos_x"], raw_state["pos_y"], raw_state["pos_z"] = values[9:12]

        prof = zero_profiles[state["active_zero_profile"]]
        state["roll"] = raw_state["roll"] - prof["roll"]
        state["pitch"] = raw_state["pitch"] - prof["pitch"]
        state["yaw"] = wrap_angle_deg(raw_state["yaw"] - prof["yaw"])
        state["pos_x"] = raw_state["pos_x"] - prof["pos_x"]
        state["pos_y"] = raw_state["pos_y"] - prof["pos_y"]
        state["pos_z"] = raw_state["pos_z"] - prof["pos_z"]


def send_hardware_zero() -> bool:
    """向 IMU 驱动发送硬件置零命令（加速度/角速度/姿态角）。"""
    with imu_sock_lock:
        if imu_sock is None:
            return False
        try:
            imu_sock.sendall(b"ZERO\n")
            return True
        except OSError as exc:
            print(f"[WARN] 硬件置零命令发送失败: {exc}")
            return False


def resolve_static_path(url_path: str) -> Path | None:
    rel = url_path.lstrip("/")
    if rel.startswith("static/"):
        rel = rel[len("static/"):]
    if not rel or ".." in Path(rel).parts:
        return None
    file_path = (STATIC_DIR / rel).resolve()
    if not file_path.is_relative_to(STATIC_DIR.resolve()):
        return None
    return file_path


def imu_reader_loop() -> None:
    global imu_sock
    while True:
        try:
            sock = socket.create_connection((IMU_TCP_HOST, IMU_TCP_PORT), timeout=5)
            sock.settimeout(1.0)
            with imu_sock_lock:
                imu_sock = sock
            print(f"[INFO] 已连接 IMU 驱动 {IMU_TCP_HOST}:{IMU_TCP_PORT}")

            buffer = ""
            while True:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if line.startswith("IMU,"):
                        update_state_from_line(line)
        except OSError as exc:
            with state_lock:
                state["connected"] = False
            with imu_sock_lock:
                if imu_sock is not None:
                    try:
                        imu_sock.close()
                    except OSError:
                        pass
                    imu_sock = None
            print(f"[WARN] 无法连接 IMU 驱动: {exc}")
            time.sleep(2)


class ImuWebHandler(BaseHTTPRequestHandler):
    server_version = "RobotWebServer/3.0"
    # 保持默认 HTTP/1.0：SSE /stream 无 Content-Length/分块编码，若开 HTTP/1.1 keep-alive
    # 会让浏览器 EventSource 无法确定消息边界 -> 面板显示“断开”。短连接 + 下面的 TCP_NODELAY
    # 在网线直连下延迟已足够低。

    def setup(self) -> None:
        super().setup()
        # 关闭 Nagle：小体积控制包立即发出，去掉最多 ~40ms 的合并等待。
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    def log_message(self, format: str, *args: object) -> None:
        if self.path.startswith(("/stream", "/api/control")):
            return
        print(f"[HTTP] {self.address_string()} {format % args}")

    def _read_json_body(self) -> dict | None:
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0:
            return None
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    def _send_json(self, payload: dict[str, object], status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, file_path: Path) -> None:
        if not file_path.is_file():
            self.send_error(404, "File not found")
            return
        content = file_path.read_bytes()
        content_type, _ = mimetypes.guess_type(str(file_path))
        self.send_response(200)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]

        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    with state_lock:
                        payload = json.dumps(state, ensure_ascii=False)
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return

        if path in ("/api/data", "/api/status"):
            with state_lock:
                self._send_json(dict(state))
            return

        if path == "/":
            self._send_file(STATIC_DIR / "index.html")
            return

        static_path = resolve_static_path(path)
        if static_path is not None:
            self._send_file(static_path)
            return

        self.send_error(404, "Not found")

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]

        if path == "/api/control":
            data = self._read_json_body()
            if data is None:
                self.send_error(400, "Bad Request")
                return
            with manual_lock:
                global last_manual_time, manual_holding
                now_ts = time.time()
                if "holding" in data:
                    holding = bool(data["holding"])
                else:
                    # 兼容未刷新的旧页面：有非零轴即视为按住
                    holding = any(
                        abs(float(data.get(k, 0.0))) > 0.01
                        for k in ("heave", "pitch", "roll", "surge", "yaw")
                    )
                manual_holding = holding
                if holding:
                    last_manual_time = now_ts
                    for key in ("heave", "pitch", "roll", "surge", "yaw"):
                        if key in data:
                            val = float(data[key])
                            manual_cmd[key] = val
                            if abs(val) > 0.01:
                                manual_cmd_sticky[key] = (val, now_ts + MANUAL_STICKY_SEC)
                            else:
                                manual_cmd_sticky.pop(key, None)
                else:
                    for key in ("heave", "pitch", "roll", "surge", "yaw"):
                        manual_cmd[key] = float(data.get(key, 0.0))
                    manual_cmd_sticky.clear()
                if "speed_mode" in data:
                    manual_cmd["speed_mode"] = str(data["speed_mode"])
            # 键盘/摇杆一介入(holding)就自动退出独立电机模式，避免刷新网页后
            # 服务器仍卡在 individual_motor_active=True 而键盘永久失灵。
            if holding:
                with individual_motor_lock:
                    global individual_motor_active
                    if individual_motor_active:
                        individual_motor_active = False
                        print("[INFO] 手动控制介入，自动退出独立电机模式")
            push_control_spi_now()
            self._send_json({"ok": True})
            return

        if path == "/api/control/mode":
            data = self._read_json_body()
            if not data or "mode" not in data:
                self.send_error(400, "Bad Request")
                return
            mode = str(data["mode"])
            if mode not in CONTROL_MODES:
                self.send_error(400, "Invalid mode")
                return
            with control_mode_lock:
                global control_mode
                control_mode = mode
            imu_controller.reset()
            print(f"[INFO] 控制模式切换为: {mode}")
            self._send_json({"ok": True, "mode": mode})
            return

        if path == "/api/zero/set":
            data = self._read_json_body()
            if not data:
                self.send_error(400, "Bad Request")
                return
            profile = data.get("profile", "A")
            if profile in zero_profiles:
                with state_lock:
                    zero_profiles[profile]["roll"] = raw_state["roll"]
                    zero_profiles[profile]["pitch"] = raw_state["pitch"]
                    zero_profiles[profile]["yaw"] = raw_state["yaw"]
                    zero_profiles[profile]["pos_x"] = raw_state["pos_x"]
                    zero_profiles[profile]["pos_y"] = raw_state["pos_y"]
                    zero_profiles[profile]["pos_z"] = raw_state["pos_z"]
                save_zero_profiles()
                hw_ok = send_hardware_zero()
                imu_controller.reset()
                msg = f"已设置零位 {profile}"
                if hw_ok:
                    msg += "（含硬件置零）"
                self._send_json({"ok": True, "message": msg, "hardware_zero": hw_ok})
                return
            self.send_error(400, "Bad Request")
            return

        if path == "/api/zero/use":
            data = self._read_json_body()
            if not data:
                self.send_error(400, "Bad Request")
                return
            profile = data.get("profile", "A")
            if profile in zero_profiles:
                with state_lock:
                    state["active_zero_profile"] = profile
                save_zero_profiles()
                imu_controller.reset()
                self._send_json({"ok": True, "message": f"已切换零位 {profile}"})
                return
            self.send_error(400, "Bad Request")
            return

        if path == "/api/motor/individual":
            data = self._read_json_body()
            if data is None:
                self.send_error(400, "Bad Request")
                return
            with individual_motor_lock:
                global individual_motor_values  # individual_motor_active 已在上方声明为 global
                if "active" in data:
                    individual_motor_active = bool(data["active"])
                if "channels" in data:
                    ch = data["channels"]
                    if len(ch) == 8:
                        individual_motor_values = [float(v) for v in ch]
            push_control_spi_now()
            self._send_json({"ok": True, "active": individual_motor_active})
            return

        self.send_error(404, "Not found")


ETH_DIRECT_IP = "192.168.50.1"


def get_local_ip() -> str:
    """优先返回网线直连地址，便于笔记本通过 eth 访问。"""
    try:
        with open("/sys/class/net/eth0/operstate", encoding="utf-8") as f:
            if f.read().strip() in ("up", "unknown"):
                out = subprocess.check_output(
                    ["ip", "-4", "-o", "addr", "show", "eth0"],
                    text=True,
                    stderr=subprocess.DEVNULL,
                )
                for token in out.split():
                    if token.startswith(f"{ETH_DIRECT_IP}/"):
                        return ETH_DIRECT_IP
    except (OSError, subprocess.CalledProcessError):
        pass

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return ETH_DIRECT_IP


def main() -> None:
    if not STATIC_DIR.is_dir():
        raise SystemExit(f"缺少网页目录: {STATIC_DIR}")

    threading.Thread(target=imu_reader_loop, daemon=True).start()
    threading.Thread(target=arm_escs_at_boot, daemon=True).start()
    threading.Thread(target=update_thrusters, daemon=True).start()

    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), ImuWebHandler)
    local_ip = get_local_ip()
    print("=" * 56)
    print("  水下机器人 IMU 闭环控制服务已启动")
    print(f"  网线直连: http://{ETH_DIRECT_IP}:{WEB_PORT}")
    print(f"  当前可用: http://{local_ip}:{WEB_PORT}")
    print(f"  SPI/STM32: {'已连接' if state.get('spi_active') else '模拟模式'}")
    print("  控制模式: manual / imu_hold / hybrid")
    print("=" * 56)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] 服务已停止")
    finally:
        server.server_close()
        spi_push_channels([0] * 8, burst=3)
        spi_close()


if __name__ == "__main__":
    main()
