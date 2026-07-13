ESC SPI slave usage

Frame format (SPI master -> STM32 slave):
- Byte 0: Header = 0xAA
- Byte 1: CMD    = 0x01 (write channels)
- Byte 2: LEN    = N (number of payload bytes, for 8 channels use 16)
- Bytes 3..(3+N-1): Payload = 8 little-endian int16_t values (channel0..channel7)
    - Each int16_t can be either:
        - signed percentage in range -100..100 (common with the Python UI), or
        - direct microsecond pulse width (1000..2000)
- Last byte: CRC8 over all previous bytes (simple CRC-8 poly 0x07)

Examples (Python master sender):

# Build example frame with percentages (-100..100) and send via spidev
import spidev

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x07
            else:
                crc = (crc << 1) & 0xFF
    return crc

channels = [0]*8  # percent values
payload = []
for v in channels:
    lv = v & 0xFFFF
    payload.append(lv & 0xFF)
    payload.append((lv >> 8) & 0xFF)

frame = [0xAA, 0x01, len(payload)] + payload
frame.append(crc8(frame))

spi = spidev.SpiDev()
spi.open(0, 0)  # select bus/device
spi.max_speed_hz = 500000
spi.mode = 0b00
spi.xfer2(frame)

Notes and mapping:
- PWM channel mapping in this project:\n  channel0..3 -> TIM2 CH1..CH4 (PA0..PA3)\n  channel4..5 -> TIM3 CH1..CH2 (PA6..PA7)\n  channel6..7 -> TIM4 CH1..CH2 (PB6..PB7 or depending on board; TIM4 CH pins configured in tim.c)
- The code assumes timers were configured with 1MHz tick (prescaler 71) and period=19999 for 50Hz.
- On boot, all channels are initialized to 0 (interpreted as mid 1500us in ESC_ApplyToPWM). You can change default values in esc_spi.c

Troubleshooting:
- Ensure SPI pins are not used by other peripherals. Adjust pin configuration in spi_slave.c if needed.
- If data seems shifted or garbage, verify SPI mode (CPOL/CPHA) and NSS handling. This implementation uses software NSS; hardware NSS may be necessary depending on master.
- To capture received frames, add debug via UART or toggle a GPIO in HAL_SPI_RxCpltCallback.
