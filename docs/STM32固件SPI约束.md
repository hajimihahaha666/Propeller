# STM32 固件 SPI 约束（Pi 侧适配依据）

> 来源：**已烧录进芯片的最终固件**，`My_Core/spi_slave.c`、`My_Core/esc_spi.c`、`My_Core/App.c`、`Core/Src/tim.c`。
> **固件不可改动**，Pi 侧必须去迁就它。换固件时请先按本文逐条比对。

## 1. 物理层与 SPI 参数

| 项 | 值 | 固件出处 |
|---|---|---|
| 角色 | STM32 = SPI2 **从机**；Pi = 主机（`/dev/spidev0.0`, CE0） | `SPI_Slave_Init` |
| 模式 | **mode 0**（CPOL=LOW, CPHA=1EDGE） | `hspi2.Init.CLKPolarity/CLKPhase` |
| 位序 / 字长 | **MSB first**，8 bit | `SPI_FIRSTBIT_MSB` / `SPI_DATASIZE_8BIT` |
| 片选 | 硬件 NSS，PB12 输入上拉（空闲高） | `SPI_NSS_HARD_INPUT` |
| 方向 | `SPI_DIRECTION_2LINES_RXONLY`，**PB14(MISO) 配成浮空输入** | `SPI_Slave_Init` |
| 速率 | 已验证 **100k–500k**，本项目用 **200k** | 实测 |

### 接线（务必共地）

| 信号 | Pi 物理脚 | STM32 |
|---|---|---|
| GND | 6/9/14/20/25 | GND |
| CE0 | 24 (GPIO8) | PB12 (NSS) |
| SCLK | 23 (GPIO11) | PB13 |
| MOSI | 19 (GPIO10) | PB15 |
| MISO | 21 (GPIO9) | PB14 —— **固件不驱动，可不接** |

## 2. 帧格式（固定 26 字节）

```
[0]      0xAA   帧头
[1]      0x01   命令（写 8 路通道）
[2]      16     数据长度 = 8 通道 × 2 字节
[3..18]  8 × int16_t 小端（通道 0..7）
[19..24] 6 × 0x00 填充
[25]     CRC8
```

CRC8：多项式 **0x07**，初值 0，无反射无异或，覆盖**前 25 字节**。

参考帧（全通道 0 = 全停）：
```
AA 01 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 93
```

## 3. 通道值语义

固件 `ESC_ApplyToPWM`：

```
-100 ≤ v ≤ 100  →  百分比，us = 1500 + v*5   （-100→1000us，0→1500us 停，+100→2000us）
其它值           →  直接当微秒脉宽
最终一律钳位到 1000..2000us
```

⚠ **101..999 这类中间值会被固件钳到 1000us**（对双向电调 = 全速反转）。`esc_spi.validate_channel_value()`
因此只接受 `-100..100` 或 `1000..2000`，其余直接报错而不是静默钳位。

本仓库网页服务全程走**百分比**：`thruster_mixer` 输出 -100..100，`arm_escs_at_boot` 发 0（=1500us 停）。

### 通道 → PWM 引脚

以 `esc_spi.c` 为准（`README_ESC.md` 已过时）：

- 通道 0-3 → TIM1 CH1-4 = **PA8 / PA9 / PA10 / PA11** = 电机 1-4
- 通道 4-7 → TIM4 CH1-4 = **PB6 / PB7 / PB8 / PB9** = 电机 5-8

PWM：prescaler 71 → 1MHz tick，period 19999 → 50Hz，比较值即脉宽 µs，1500 = 中位。

## 4. ★ 三条必须让 Pi 迁就的硬约束

### A. 帧头必须落在接收缓冲区第 0 字节

固件预挂"恰好 26 字节"的中断接收，收满即调 `spi_validate_frame`，而它**只认 `buf[0..2] == AA 01 10`**。
`build_aligned_view()` 那套"窗口内找帧头再循环对齐"**仅写入调试变量，不进 ESC 更新路径**。

→ **Pi 必须一次 `xfer2` 恰好发一帧 26 字节，帧间留 ≥数百 µs 间隔**，绝不能把多帧首尾相连塞进同一次 CS。
本仓库 `FRAME_GAP_SEC = 0.001`。

### B. 校验失败每满 200 次，固件自动切一次 CPOL/CPHA，且无法自愈

`spi_slave.c`：

```c
if ((spi_validate_fail_count % 200u) == 0u) {
    spi_mode_index = (spi_mode_index + 1u) & 0x03u;   /* 0→1→2→3 循环 */
    spi_pending_mode_switch = 1u;
}
```

切换由主循环里 500ms 一次的 `SPI_Slave_Poll()` 应用。一旦 Pi 侧持续失步，STM32 就会在 4 个模式间乱跳，
与 Pi 固定的 mode 0 永久错位 —— **全部电机不转，且固件自己回不来，只能复位 STM32**。

→ 所以**任何总线争用都是致命的**。所有发送方（守护进程、`spi_send_burst.py`、`esc_spi.py` 的 TUI）
必须共用 `esc_spi.spi_bus_lock()`（进程内 RLock + `/tmp/propeller_spi.lock` 跨进程 flock）。
→ 速率也不能拔高：1MHz 时每字节仅 8µs，而固件是**逐字节中断**接收，容易溢出丢字节 → 失步 → 触发本条。

### C. STM32 侧没有失控保护

`ESC_TimeoutHandler()` 有实现，但 **App.c 里 `Millisecond_Task` / `Millisecond_50_Task` 都是空的，从未被调用**。

→ **Pi 一旦停发，最后一次油门会被 PWM 永久保持，电机不会自己停。**
→ 失控保护只能由 Pi 负责：`imu_web_server.py` 注册了 SIGTERM/SIGINT handler，
退出前经 `stop_thrusters_and_close()` 连发中位帧；`start_propeller_daemon.sh` 的 `cleanup` 会等它发完再退出
（否则 systemd `KillMode=mixed` 的 SIGKILL 会把兜底帧掐掉）。

## 5. 链路确认：软件层拿不到任何回执

固件是 RXONLY 且 **PB14(MISO) 不驱动**，所以 Pi `xfer2` 的读回全是悬空噪声。

> **绝不能用读回判断链路是否通。** 历史上就因为拿读回当回执而长期误判：
> 若读回和发出完全一致（如 `AA 01 10 … 93`），那是 MOSI→MISO 回环假信号，
> 不能证明 STM32 收到了。

确认链路只有三条路：

1. 电调上电自检音 / 电机反应；
2. **ST-Link + OpenOCD 读 STM32 RAM**（唯一铁证）：
   `spi_interrupt_count`（有没有收到时钟）、`spi_validate_ok_count`（CRC 过了几帧）、
   `spi_pwm_update_count`、`debug_esc_throttles[8]`（解码出的油门）、`spi_mode_index`（有没有被切走）；
3. 示波器。

⚠ `openocd halt` 本身会打断 STM32 正在收的帧、造成失步（无帧内重对齐）。调试读 RAM 要少用；
读完若发现 mode 漂移 / ok 停涨，`openocd -c init -c "reset run" -c shutdown` 复位一次即恢复。
判断链路是否真健康，要在**不碰 openocd** 的情况下让守护进程独自跑一段再读一次。

## 6. 上电时序注意

- 固件 `ESC_PWM_Init()` 在上电时把 8 路全部置 **1500µs**（中位/停）。
- `SPI_Slave_Init()` 之前有 PWM 自检延时，STM32 上电约数秒后才挂上接收。
- Pi 侧 `arm_escs_at_boot()` 启动后连发 3s 中位帧，兼作电调解锁与帧对齐预热。
