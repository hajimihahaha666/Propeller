# Propeller - 水下机器人控制系统

基于树莓派的 IMU 姿态监控 + 网页推进器控制项目。

## 项目结构

```
imu_ws/
├── src/imu_serial_driver/   # ROS2 IMU 串口驱动（C）
├── web_server/              # 网页控制服务（Python）
│   ├── imu_web_server.py    # 主服务：IMU 数据 + 推进器控制
│   ├── esc_spi.py             # SPI 电调通信
│   └── static/index.html      # 网页界面
├── pc_host/                 # 旧版桌面上位机（可选）
└── web_server/start_imu_web.sh  # 一键启动脚本
```

## IMU 闭环控制（新增）

三种控制模式（网页左侧切换）：

| 模式 | 说明 |
|------|------|
| **manual** | 手动：摇杆直接控制 8 路推进器 |
| **imu_hold** | IMU 保持：PID 根据姿态误差自动稳姿，维持当前零位 |
| **hybrid** | 混合：摇杆设定目标姿态，IMU PID 自动跟踪修正 |

数据流：`IMU → PID(imu_controller.py) → 8路混控 → SPI → STM32 → 推进器`


- 幻尔 IMU 实时读取：加速度、角速度、姿态角
- PID 滤波 + 坐标积分，减少数据漂移
- 网页 3D 姿态显示
- 多零位配置（A / B / C）
- 8 路推进器控制（6 路姿态 + 2 路前进/转向）
- 虚拟摇杆 + 速度档位（慢/中/快）

## 快速启动

```bash
/home/hansing/imu_ws/web_server/start_imu_web.sh
```

浏览器打开：`http://<树莓派IP>:8080`

## 依赖

- ROS 2 Jazzy
- Python 3
- IMU 串口：`/dev/ttyUSB0`
- SPI 电调（可选）：`spidev`

## 协作者

克隆仓库后：

```bash
cd imu_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imu_serial_driver
source install/setup.bash
./web_server/start_imu_web.sh
```
