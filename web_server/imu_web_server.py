#!/usr/bin/env python3
"""IMU 反馈闭环 + 网页推进器控制服务"""

from __future__ import annotations

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

control_mode = "manual"
control_mode_lock = threading.Lock()

imu_controller = ImuThrusterController()
last_control_tick = time.time()

esc_instance = None
if ESC_SPI:
    try:
        esc_instance = ESC_SPI(bus=0, device=0, max_speed_hz=5000000, mode=0)
        esc_instance.fill_center()
        state["spi_active"] = True
        print("[INFO] SPI 电调已连接 (STM32)")
    except Exception as exc:
        print(f"[ERROR] SPI 初始化失败: {exc}")
        esc_instance = None


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))


def mix_thrusters(h: float, p: float, r: float, s: float, y: float, limit: float) -> list[int]:
    t1 = h + p + r
    t2 = h + r
    t3 = h - p + r
    t4 = h + p - r
    t5 = h - r
    t6 = h - p - r
    t7 = s + y
    t8 = s - y
    channels = [t1, t2, t3, t4, t5, t6, t7, t8]
    return [int(clamp(ch * limit, -limit, limit)) for ch in channels]


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
        if manual_age > 1.0:
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
    speed_limits = {"slow": 25.0, "medium": 50.0, "fast": 80.0}

    while True:
        with individual_motor_lock:
            indi_active = individual_motor_active
            indi_values = list(individual_motor_values)

        if indi_active:
            channels = [int(clamp(v, -100, 100)) for v in indi_values]
        else:
            with manual_lock:
                limit = speed_limits.get(manual_cmd["speed_mode"], 50.0)
            h, p, r, s, y = compute_thruster_commands()
            channels = mix_thrusters(h, p, r, s, y, limit)

        with state_lock:
            state["individual_motor_active"] = indi_active
            state["individual_motor_values"] = list(indi_values)

        if esc_instance:
            esc_instance.set_all(channels)
            try:
                esc_instance.send_frame()
            except OSError:
                pass

        time.sleep(0.01)


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
                global last_manual_time
                last_manual_time = time.time()
                for key in ("heave", "pitch", "roll", "surge", "yaw"):
                    if key in data:
                        manual_cmd[key] = float(data[key])
                if "speed_mode" in data:
                    manual_cmd["speed_mode"] = str(data["speed_mode"])
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
                global individual_motor_active, individual_motor_values
                if "active" in data:
                    individual_motor_active = bool(data["active"])
                if "channels" in data:
                    ch = data["channels"]
                    if len(ch) == 8:
                        individual_motor_values = [float(v) for v in ch]
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
    threading.Thread(target=update_thrusters, daemon=True).start()

    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), ImuWebHandler)
    local_ip = get_local_ip()
    print("=" * 56)
    print("  水下机器人 IMU 闭环控制服务已启动")
    print(f"  网线直连: http://{ETH_DIRECT_IP}:{WEB_PORT}")
    print(f"  当前可用: http://{local_ip}:{WEB_PORT}")
    print(f"  SPI/STM32: {'已连接' if esc_instance else '模拟模式'}")
    print("  控制模式: manual / imu_hold / hybrid")
    print("=" * 56)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] 服务已停止")
    finally:
        server.server_close()
        if esc_instance:
            esc_instance.fill_center()
            esc_instance.send_frame()
            esc_instance.close()


if __name__ == "__main__":
    main()
