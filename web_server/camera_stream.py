#!/usr/bin/env python3
"""USB 摄像头 MJPEG 推流服务（端口 8081，路径 /stream）。

网页 index.html 通过 <img src="http://<pi>:8081/stream"> 拉流。
用 ffmpeg 抓取 /dev/video0：优先「原生 MJPEG 直通」（最省 CPU），
若摄像头不支持则回退「软件编码 MJPEG」，分辨率 1920x1080 失败再逐级降级。
逐帧解析 JPEG(FFD8..FFD9)，以 multipart/x-mixed-replace 输出。
"""

from __future__ import annotations

import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEVICE = "/dev/video0"
PORT = 8081
BOUNDARY = "frame"
# 依次尝试的采集参数：(是否原生直通, 宽, 高, 帧率)
ATTEMPTS = [
    (True, 1920, 1080, 30),
    (True, 1280, 720, 30),
    (False, 1920, 1080, 15),
    (False, 1280, 720, 20),
    (False, 640, 480, 20),
]


def build_cmd(native: bool, w: int, h: int, fps: int) -> list[str]:
    base = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2"]
    if native:
        # 摄像头原生输出 MJPEG，直接 copy，几乎不耗 CPU
        return base + [
            "-input_format", "mjpeg",
            "-video_size", f"{w}x{h}",
            "-framerate", str(fps),
            "-i", DEVICE,
            "-c:v", "copy",
            "-f", "mjpeg", "pipe:1",
        ]
    # 软件编码：兼容 YUYV 等非 MJPEG 摄像头
    return base + [
        "-video_size", f"{w}x{h}",
        "-framerate", str(fps),
        "-i", DEVICE,
        "-f", "mjpeg", "-q:v", "5", "pipe:1",
    ]


def open_ffmpeg():
    """按 ATTEMPTS 逐个尝试，返回存活的 ffmpeg 进程；全失败返回 None。"""
    for native, w, h, fps in ATTEMPTS:
        cmd = build_cmd(native, w, h, fps)
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
            )
        except FileNotFoundError:
            print("[CAM] 未找到 ffmpeg，请: sudo apt install -y ffmpeg", flush=True)
            return None
        # 读首个 JPEG 头判断是否真的出图
        head = proc.stdout.read(3)
        if head == b"\xff\xd8\xff":
            print(f"[CAM] ffmpeg 就绪 native={native} {w}x{h}@{fps}", flush=True)
            return proc, head
        # 失败：清理后试下一组
        try:
            proc.kill()
            err = proc.stderr.read(400).decode("utf-8", "ignore").strip()
        except Exception:
            err = ""
        print(f"[CAM] 尝试失败 native={native} {w}x{h}: {err[:200]}", flush=True)
    return None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *_a):
        pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/stream", "/stream.mjpg", "/"):
            self.stream_mjpeg()
        else:
            self.send_error(404)

    def stream_mjpeg(self):
        opened = open_ffmpeg()
        if not opened:
            self.send_error(503, "camera unavailable")
            return
        proc, head = opened
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )
        self.end_headers()

        buf = bytearray(head)
        try:
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf.extend(chunk)
                # 从缓冲区中切出完整 JPEG（FFD8..FFD9）
                while True:
                    start = buf.find(b"\xff\xd8\xff")
                    if start < 0:
                        break
                    end = buf.find(b"\xff\xd9", start + 3)
                    if end < 0:
                        if start > 0:
                            del buf[:start]
                        break
                    jpeg = bytes(buf[start : end + 2])
                    del buf[: end + 2]
                    self.wfile.write(b"--" + BOUNDARY.encode() + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:  # noqa: BLE001
            print(f"[CAM] 流异常: {exc}", flush=True)
        finally:
            try:
                proc.kill()
            except Exception:
                pass


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    if not shutil.which("ffmpeg"):
        print("[CAM] 缺少 ffmpeg，请: sudo apt install -y ffmpeg", flush=True)
    print(f"[CAM] 摄像头推流服务启动: http://0.0.0.0:{PORT}/stream (设备 {DEVICE})", flush=True)
    httpd = Server(("0.0.0.0", PORT), Handler)

    def _stop(*_a):
        httpd.shutdown()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
