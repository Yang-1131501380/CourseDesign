# Software

该目录用于存放项目的软件工程与联调相关代码，当前主工程为 `F407_TEST/`。

## Main Project

- `F407_TEST/`
  当前整车主控集成工程，负责视觉数据接入、云台跟踪、姿态采样、底盘跟随和状态显示。

## Control Flow

```text
K230 target detection
-> USART6 target frame input
-> STM32 target parsing
-> gimbal yaw/pitch tracking
-> IMU / compass feedback
-> chassis follow control
-> LCD / serial monitoring
```

## Runtime Tasks

`Core/Src/app.c` 是应用入口，当前核心任务基于 RT-Thread 组织：

- `ctrl`
  10 ms 周期处理 K230 数据解析、云台控制与追踪状态更新
- `mon`
  推进 LCD DMA、输出调试信息、处理串口调参命令
- `imu`
  周期采样 `ICM42688`，并在 I2C 异常时执行恢复
- `chassis`
  根据目标偏差和距离执行底盘低速跟随与目标丢失刹车

## Key Modules

- `F407_TEST/Core/Src/app.c`
  应用层入口与线程调度
- `F407_TEST/HARDWARE/Emm_V5/`
  K230 协议解析、云台驱动通信、目标跟踪控制
- `F407_TEST/HARDWARE/ICM42688/`
  IMU 采样、姿态处理与数学辅助
- `F407_TEST/HARDWARE/QMC5883/`
  航向参考数据获取
- `F407_TEST/HARDWARE/Motor/`
  TB6612 驱动、编码器反馈、底盘电机控制
- `F407_TEST/HARDWARE/UART_MULTI/`
  多串口 DMA + FIFO 通信封装
- `F407_TEST/HARDWARE/ST7735/`
  LCD 驱动与状态文本显示

## Key Interfaces

- `USART1`
  调试日志与手动调参
- `USART2`
  `EMM_V5` 云台驱动器总线
- `USART6`
  `K230` 目标检测数据输入
- `I2C1`
  `ICM42688` 等姿态传感器通信

## Tracking Strategy

- K230 输出目标中心、偏移量与距离
- STM32 解析后驱动云台 yaw / pitch 对准目标
- 根据目标距离分段控制底盘前进速度
- 根据目标横向偏差和车体姿态执行转向修正
- 目标超时丢失时立即停止底盘运动

## Notes

- `software/K230/` 存放项目实际使用的 K230 侧脚本与说明
- 历史参考资料和第三方配套包不再作为 GitHub 主展示内容
