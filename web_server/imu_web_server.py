#!/usr/bin/env python3
"""IMU & Thruster Web Server"""
from __future__ import annotations

import json
import math
import mimetypes
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

# Import ESC_SPI if available
try:
    from esc_spi import ESC_SPI
except ImportError:
    ESC_SPI = None
    print("[WARN] esc_spi.py not found or spidev not installed. Thrusters will be simulated.")

STATIC_DIR = Path(__file__).resolve().parent / "static"
IMU_TCP_HOST = "127.0.0.1"
IMU_TCP_PORT = 8888
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

# IMU State and Zero Profiles
state: dict[str, object] = {
    "connected": False,
    "ax": 0.0, "ay": 0.0, "az": 0.0,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
    "active_zero_profile": "A",
}

# Store raw values for zeroing calculation
raw_state = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
}

zero_profiles = {
    "A": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0},
    "B": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0},
    "C": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0},
}

state_lock = threading.Lock()
imu_sock: socket.socket | None = None
imu_sock_lock = threading.Lock()

# Thruster Control State
thruster_state = {
    "heave": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "surge": 0.0,
    "yaw": 0.0,
    "speed_mode": "medium" # slow, medium, fast
}
thruster_lock = threading.Lock()
last_control_time = time.time()

# ESC Instance
esc_instance = None
if ESC_SPI:
    try:
        esc_instance = ESC_SPI(bus=0, device=0, max_speed_hz=100000, mode=0)
        esc_instance.fill_center()
    except Exception as e:
        print(f"[ERROR] Failed to init ESC_SPI: {e}")
        esc_instance = None

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def update_thrusters():
    """Background thread to continuously send SPI frames at 50Hz"""
    global last_control_time
    
    speed_limits = {
        "slow": 25.0,
        "medium": 50.0,
        "fast": 80.0
    }
    
    while True:
        with thruster_lock:
            # Auto-stop if no control received for 1 second
            if time.time() - last_control_time > 1.0:
                h, p, r, s, y = 0.0, 0.0, 0.0, 0.0, 0.0
            else:
                h = thruster_state["heave"]
                p = thruster_state["pitch"]
                r = thruster_state["roll"]
                s = thruster_state["surge"]
                y = thruster_state["yaw"]
                
            mode = thruster_state["speed_mode"]
            limit = speed_limits.get(mode, 50.0)

        # Calculate Top 6 Thrusters (Attitude + Heave)
        # 1: Left Front, 2: Left Mid, 3: Left Back
        # 4: Right Front, 5: Right Mid, 6: Right Back
        
        # Heave: all push up/down (assume positive = up, thrust needed = positive)
        # Pitch: positive = pitch forward (front up, back down) => 1,4 = +p, 3,6 = -p
        # Roll: positive = roll right (left up, right down) => 1,2,3 = +r, 4,5,6 = -r
        
        t1 = h + p + r
        t2 = h + r
        t3 = h - p + r
        t4 = h + p - r
        t5 = h - r
        t6 = h - p - r
        
        # Calculate Tail 2 Thrusters (Surge + Yaw)
        # 7: Tail Left, 8: Tail Right
        # Surge: positive = forward => 7,8 = +s
        # Yaw: positive = turn right => 7 = +y, 8 = -y
        t7 = s + y
        t8 = s - y
        
        channels = [t1, t2, t3, t4, t5, t6, t7, t8]
        
        # Scale and send
        if esc_instance:
            for i in range(8):
                val = clamp(channels[i] * limit, -limit, limit)
                esc_instance.set_channel(i, int(val))
            try:
                esc_instance.send_frame()
            except Exception as e:
                pass
                
        time.sleep(0.02) # 50Hz

def update_state_from_line(line: str) -> None:
    parts = line.strip().split(",")
    if parts[0] != "IMU":
        return

    try:
        values = [0.0 if math.isnan(float(v)) or math.isinf(float(v)) else float(v) for v in parts[1:]]
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
            
        # Apply current zero profile
        prof = zero_profiles[state["active_zero_profile"]]
        state["roll"] = raw_state["roll"] - prof["roll"]
        state["pitch"] = raw_state["pitch"] - prof["pitch"]
        state["yaw"] = raw_state["yaw"] - prof["yaw"]
        state["pos_x"] = raw_state["pos_x"] - prof["pos_x"]
        state["pos_y"] = raw_state["pos_y"] - prof["pos_y"]
        state["pos_z"] = raw_state["pos_z"] - prof["pos_z"]


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
            print(f"[WARN] 无法连接 IMU 驱动，2 秒后重试: {exc}")
            time.sleep(2)


class ImuWebHandler(BaseHTTPRequestHandler):
    server_version = "RobotWebServer/2.0"
    
    def log_message(self, format: str, *args: object) -> None:
        if self.path.startswith("/stream") or self.path.startswith("/api/control"):
            return
        print(f"[HTTP] {self.address_string()} {format % args}")
        
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

        if path == "/api/data":
            with state_lock:
                self._send_json(dict(state))
            return

        if path == "/":
            self._send_file(STATIC_DIR / "index.html")
            return

        self._send_file(STATIC_DIR / path.lstrip("/"))
        
    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        
        if path == "/api/control":
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    with thruster_lock:
                        global last_control_time
                        last_control_time = time.time()
                        if "heave" in data: thruster_state["heave"] = float(data["heave"])
                        if "pitch" in data: thruster_state["pitch"] = float(data["pitch"])
                        if "roll" in data: thruster_state["roll"] = float(data["roll"])
                        if "surge" in data: thruster_state["surge"] = float(data["surge"])
                        if "yaw" in data: thruster_state["yaw"] = float(data["yaw"])
                        if "speed_mode" in data: thruster_state["speed_mode"] = str(data["speed_mode"])
                    self._send_json({"ok": True})
                    return
                except Exception as e:
                    self.send_error(400, "Bad Request")
                    return
                    
        if path == "/api/zero/set":
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    profile = data.get("profile", "A")
                    if profile in zero_profiles:
                        with state_lock:
                            zero_profiles[profile]["roll"] = raw_state["roll"]
                            zero_profiles[profile]["pitch"] = raw_state["pitch"]
                            zero_profiles[profile]["yaw"] = raw_state["yaw"]
                            zero_profiles[profile]["pos_x"] = raw_state["pos_x"]
                            zero_profiles[profile]["pos_y"] = raw_state["pos_y"]
                            zero_profiles[profile]["pos_z"] = raw_state["pos_z"]
                        print(f"[INFO] 已设置零位配置 {profile}")
                        self._send_json({"ok": True, "message": f"已成功设置零位 {profile}"})
                        return
                except Exception as e:
                    pass
            self.send_error(400, "Bad Request")
            return
            
        if path == "/api/zero/use":
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    profile = data.get("profile", "A")
                    if profile in zero_profiles:
                        with state_lock:
                            state["active_zero_profile"] = profile
                        print(f"[INFO] 已切换至零位配置 {profile}")
                        self._send_json({"ok": True, "message": f"已切换至零位 {profile}"})
                        return
                except Exception as e:
                    pass
            self.send_error(400, "Bad Request")
            return
            
        self.send_error(404, "Not found")

def get_local_ip() -> str:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return "127.0.0.1"

def main() -> None:
    if not STATIC_DIR.is_dir():
        raise SystemExit(f"缺少网页目录: {STATIC_DIR}")
        
    threading.Thread(target=imu_reader_loop, daemon=True).start()
    threading.Thread(target=update_thrusters, daemon=True).start()
    
    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), ImuWebHandler)
    local_ip = get_local_ip()
    print("=" * 56)
    print("  机器人网页控制服务已启动")
    print(f"  在本机浏览器打开: http://127.0.0.1:{WEB_PORT}")
    print(f"  在电脑浏览器打开:   http://{local_ip}:{WEB_PORT}")
    print("=" * 56)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] 网页服务已停止")
    finally:
        server.server_close()
        if esc_instance:
            esc_instance.fill_center()
            esc_instance.send_frame()
            esc_instance.close()

if __name__ == "__main__":
    main()
