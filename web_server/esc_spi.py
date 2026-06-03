import argparse
import os
import struct
import time

try:
    import spidev
except ImportError:
    spidev = None


def crc8(data: bytes) -> int:
    """计算 CRC8，多项式 0x07，初始值 0，无反射、无异或。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class ESC_SPI:
    FRAME_HEADER = 0xAA
    CMD = 0x01
    DATA_LEN = 16  # 8 * int16
    PADDING_LEN = 6
    TOTAL_FRAME_LEN = 26
    CHANNEL_COUNT = 8

    def __init__(self, bus=0, device=0, max_speed_hz=1000000, mode=0, bits_per_word=8):
        if spidev is None:
            raise RuntimeError("spidev is required on Raspberry Pi to use SPI")

        self.spi = spidev.SpiDev()
        try:
            self.spi.open(bus, device)
        except FileNotFoundError as exc:
            dev = f"/dev/spidev{bus}.{device}"
            spidev_list = []
            if os.path.isdir('/dev'):
                spidev_list = [n for n in os.listdir('/dev') if n.startswith('spidev')]
            raise RuntimeError(
                f"SPI device {dev} not found. Please enable SPI in raspi-config and verify the device exists. "
                f"Also confirm the correct bus/device and that the SPI kernel module is loaded. "
                f"Available SPI devices: {', '.join(sorted(spidev_list)) or 'none'}"
            ) from exc
        self.spi.max_speed_hz = max_speed_hz
        self.spi.mode = mode
        self.spi.bits_per_word = bits_per_word
        self.channels = [0] * self.CHANNEL_COUNT

    def close(self):
        self.spi.close()

    def build_frame(self, channels):
        if len(channels) != self.CHANNEL_COUNT:
            raise ValueError(f"channels must be {self.CHANNEL_COUNT} values")

        frame = bytearray()
        frame.append(self.FRAME_HEADER)
        frame.append(self.CMD)
        frame.append(self.DATA_LEN)
        for v in channels:
            frame += struct.pack("<h", int(v))
        
        # 添加6个字节的填充
        frame.extend([0] * self.PADDING_LEN)

        frame.append(crc8(frame))
        # 验证帧长度
        if len(frame) != self.TOTAL_FRAME_LEN:
            raise RuntimeError(f"构建的帧长度 ({len(frame)}) 与预期的 ({self.TOTAL_FRAME_LEN}) 不符")
        return frame

    def send_frame(self, channels=None):
        if channels is None:
            channels = self.channels
        frame = self.build_frame(channels)
        rx = self.spi.xfer2(list(frame))
        return frame, bytes(rx)

    @staticmethod
    def parse_frame(data: bytes):
        """从 SPI 回传中解析完整帧，返回 (values, error)。"""
        if not data:
            return None, "no data"
        if len(data) < ESC_SPI.TOTAL_FRAME_LEN:
            return None, f"len {len(data)} < {ESC_SPI.TOTAL_FRAME_LEN}"

        if data[0] != ESC_SPI.FRAME_HEADER:
            return None, f"bad header 0x{data[0]:02X}"
        if data[1] != ESC_SPI.CMD:
            return None, f"bad cmd 0x{data[1]:02X}"
        if data[2] != ESC_SPI.DATA_LEN:
            return None, f"bad len 0x{data[2]:02X}"

        frame_without_crc = data[:ESC_SPI.TOTAL_FRAME_LEN - 1]
        received_crc = data[ESC_SPI.TOTAL_FRAME_LEN - 1]
        calculated_crc = crc8(frame_without_crc)
        if calculated_crc != received_crc:
            return None, f"crc mismatch {calculated_crc:02X}!={received_crc:02X}"

        values = []
        for i in range(ESC_SPI.CHANNEL_COUNT):
            offset = 3 + i * 2
            values.append(int.from_bytes(data[offset : offset + 2], "little", signed=True))
        return values, "ok"

    @staticmethod
    def format_channel_value(value: int) -> str:
        if -100 <= value <= 100:
            us = 1500 + value * 5
            return f"{value:+4d}% / {us:4d}us"
        return f"{value:5d}us"

    def set_channel(self, index, value):
        if not 0 <= index < self.CHANNEL_COUNT:
            raise IndexError("channel index out of range")
        self.channels[index] = int(value)

    def set_all(self, values):
        if len(values) != self.CHANNEL_COUNT:
            raise ValueError(f"values must contain {self.CHANNEL_COUNT} channels")
        self.channels = [int(v) for v in values]

    def fill_center(self):
        self.channels = [0] * self.CHANNEL_COUNT


def draw_screen(stdscr, esc, selected, actual_values, frame_hex, rx_valid, parse_status):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    def add_line(y, x, text):
        if y < 0 or y >= max_y:
            return
        text = text[:max_x - x]
        try:
            stdscr.addstr(y, x, text)
        except Exception:
            pass

    add_line(0, 0, "ESC SPI 控制面板 (q 退出, 1-8 选通道, 上下改通道, 左右改大小, c 置中)")
    add_line(1, 0, "按键: q 退出, c 中位, r 重新发送")
    add_line(2, 0, "发送频率: 50Hz, SPI mode 0, 100kHz (测试用), 8 通道")
    add_line(3, 0, f"当前选中通道: {selected + 1}")
    add_line(4, 0, "发送通道值:")
    for idx, value in enumerate(esc.channels):
        mark = ">" if idx == selected else " "
        add_line(5 + idx, 0, f"{mark} 通道 {idx + 1}: {esc.format_channel_value(value)}")
    add_line(14, 0, "回传实际速度:")
    if actual_values is None:
        add_line(15, 0, "  等待有效回传数据...")
    else:
        for idx, value in enumerate(actual_values):
            add_line(15 + idx, 0, f"  实际 {idx + 1}: {esc.format_channel_value(value)}")
    add_line(24, 0, f"RX len {len(frame_hex.split()) if frame_hex else 0}, status: {'有效' if rx_valid else '无效'}")
    add_line(25, 0, f"最后一次回传帧: {frame_hex}")
    add_line(26, 0, f"解析状态: {parse_status}")
    stdscr.refresh()


def tui_main(stdscr):
    curses = __import__("curses")
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    esc = ESC_SPI(bus=0, device=0, max_speed_hz=100000, mode=0)
    esc.fill_center()
    selected = 0
    actual_values = None
    frame_hex = ""
    rx_valid = False
    parse_status = "等待发送"
    last_send = 0.0
    send_interval = 0.02

    try:
        while True:
            now = time.time()
            if now - last_send >= send_interval:
                frame, rx_bytes = esc.send_frame()
                frame_hex = " ".join(f"{b:02X}" for b in rx_bytes)
                actual_values, parse_status = esc.parse_frame(rx_bytes)
                rx_valid = actual_values is not None
                last_send = now
            draw_screen(stdscr, esc, selected, actual_values, frame_hex, rx_valid, parse_status)
            key = stdscr.getch()
            if key == -1:
                time.sleep(0.01)
                continue
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("c"), ord("C")):
                esc.fill_center()
            if key in (ord("r"), ord("R")):
                frame, rx_bytes = esc.send_frame()
                frame_hex = " ".join(f"{b:02X}" for b in rx_bytes)
                actual_values, parse_status = esc.parse_frame(rx_bytes)
                rx_valid = actual_values is not None
            if key == curses.KEY_UP:
                selected = (selected - 1) % esc.CHANNEL_COUNT
            if key == curses.KEY_DOWN:
                selected = (selected + 1) % esc.CHANNEL_COUNT
            if key == curses.KEY_LEFT:
                esc.channels[selected] = max(-2000, esc.channels[selected] - 5)
            if key == curses.KEY_RIGHT:
                esc.channels[selected] = min(2000, esc.channels[selected] + 5)
            if ord("1") <= key <= ord("8"):
                selected = key - ord("1")
    finally:
        esc.close()

def main():
    try:
        import curses
    except ImportError:
        print("无法导入 curses，无法启动 TUI。请在支持 curses 的终端环境中运行。")
        return
    curses.wrapper(tui_main)


if __name__ == "__main__":
    main()
