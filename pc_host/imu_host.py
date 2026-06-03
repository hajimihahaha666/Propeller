#!/usr/bin/env python3
"""幻尔 IMU 上位机：连接树莓派 TCP 服务，显示数据并发送置零命令。"""

import socket
import threading
import tkinter as tk
from tkinter import messagebox, ttk


DEFAULT_HOST = "10.17.92.104"
DEFAULT_PORT = 8888


class ImuHostApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("幻尔 IMU 上位机")
        self.root.geometry("520x420")

        self.sock: socket.socket | None = None
        self.connected = False
        self.running = False
        self.recv_thread: threading.Thread | None = None

        self._build_ui()

    def _build_ui(self) -> None:
        conn_frame = ttk.LabelFrame(self.root, text="连接设置")
        conn_frame.pack(fill="x", padx=12, pady=8)

        ttk.Label(conn_frame, text="树莓派 IP:").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(conn_frame, textvariable=self.host_var, width=18).grid(row=0, column=1, padx=4, pady=8)

        ttk.Label(conn_frame, text="端口:").grid(row=0, column=2, padx=8, pady=8, sticky="w")
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        ttk.Entry(conn_frame, textvariable=self.port_var, width=8).grid(row=0, column=3, padx=4, pady=8)

        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=8, pady=8)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(conn_frame, textvariable=self.status_var).grid(row=0, column=5, padx=8, pady=8)

        data_frame = ttk.LabelFrame(self.root, text="实时数据")
        data_frame.pack(fill="both", expand=True, padx=12, pady=8)

        labels = [
            ("加速度 X (m/s²)", "ax"),
            ("加速度 Y (m/s²)", "ay"),
            ("加速度 Z (m/s²)", "az"),
            ("角速度 X (rad/s)", "gx"),
            ("角速度 Y (rad/s)", "gy"),
            ("角速度 Z (rad/s)", "gz"),
            ("Roll 角度 (°)", "roll"),
            ("Pitch 角度 (°)", "pitch"),
            ("Yaw 角度 (°)", "yaw"),
        ]

        self.value_vars: dict[str, tk.StringVar] = {}
        for row, (title, key) in enumerate(labels):
            ttk.Label(data_frame, text=title).grid(row=row, column=0, padx=10, pady=4, sticky="w")
            var = tk.StringVar(value="--")
            self.value_vars[key] = var
            ttk.Label(data_frame, textvariable=var, width=18).grid(row=row, column=1, padx=10, pady=4, sticky="w")

        ctrl_frame = ttk.Frame(self.root)
        ctrl_frame.pack(fill="x", padx=12, pady=8)

        ttk.Button(ctrl_frame, text="三轴置零", command=self.send_zero).pack(side="left", padx=8)
        ttk.Label(
            ctrl_frame,
            text="置零后当前姿态将作为 Roll/Pitch/Yaw 的 0° 原点",
        ).pack(side="left", padx=8)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def toggle_connection(self) -> None:
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        host = self.host_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            messagebox.showerror("错误", "端口必须是数字")
            return

        try:
            self.sock = socket.create_connection((host, port), timeout=3)
            self.sock.settimeout(1.0)
        except OSError as exc:
            messagebox.showerror("连接失败", f"无法连接到 {host}:{port}\n{exc}")
            self.sock = None
            return

        self.connected = True
        self.running = True
        self.connect_btn.config(text="断开")
        self.status_var.set(f"已连接 {host}:{port}")
        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()

    def disconnect(self) -> None:
        self.running = False
        self.connected = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.connect_btn.config(text="连接")
        self.status_var.set("未连接")

    def recv_loop(self) -> None:
        buffer = ""
        while self.running and self.sock is not None:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="ignore")
            except socket.timeout:
                continue
            except OSError:
                break

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if line.startswith("IMU,"):
                    self.root.after(0, self.update_values, line)

        self.root.after(0, self.handle_disconnect)

    def update_values(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) != 10:
            return

        keys = ["ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw"]
        for key, value in zip(keys, parts[1:], strict=False):
            try:
                self.value_vars[key].set(f"{float(value):.3f}")
            except ValueError:
                self.value_vars[key].set(value)

    def send_zero(self) -> None:
        if not self.connected or self.sock is None:
            messagebox.showwarning("提示", "请先连接树莓派")
            return

        try:
            self.sock.sendall(b"ZERO\n")
        except OSError as exc:
            messagebox.showerror("发送失败", str(exc))

    def handle_disconnect(self) -> None:
        if self.connected:
            self.disconnect()
            messagebox.showinfo("提示", "与树莓派的连接已断开")

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ImuHostApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
