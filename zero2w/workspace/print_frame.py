import struct


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


FRAME_HEADER = 0xAA
CMD = 0x01
DATA_LEN = 16
PADDING_LEN = 6
CHANNELS = [20] + [0] * 7  # match your TUI: channel1 = +20

frame = bytearray()
frame.append(FRAME_HEADER)
frame.append(CMD)
frame.append(DATA_LEN)
for v in CHANNELS:
    frame += struct.pack("<h", int(v))
frame.extend([0] * PADDING_LEN)
frame.append(crc8(frame))

print("len=", len(frame))
print("hex:", " ".join(f"{b:02X}" for b in frame))
