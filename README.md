# STM32 Real-Time UAV Tracking Car

基于 `STM32F407` 与 `K230` 视觉模组的无人车实时追踪系统。项目聚焦嵌入式控制链路设计与整车联调，核心目标是实现对空中目标的实时检测、方向跟踪、姿态感知与地面跟随控制。

## Highlights

- `STM32F407` 负责整车控制、姿态解算、云台驱动与底盘跟随
- `K230` 输出目标中心、偏移量与距离信息
- `EMM_V5` 二维云台完成目标方位跟踪
- `ICM42688 + QMC5883` 提供车体姿态与航向参考
- `TB6612 + 编码器` 实现双轮底盘速度控制与低速跟随
- `RT-Thread` 多线程任务架构支撑控制、监测、采样与显示链路

## System Flow

```text
K230 target detection
-> USART6 data input
-> STM32 target parsing
-> EMM_V5 gimbal tracking
-> IMU / compass attitude feedback
-> chassis follow control
-> LCD / serial monitoring
```

## Repository Layout

- `software/`
  STM32 主控工程、驱动代码、控制逻辑与联调相关软件
- `hardware/`
  嘉立创EDA硬件设计文件与硬件说明

Quick entry:

- `software/README.md` for software architecture and module layout
- `hardware/README.md` for hardware design scope and system connection overview

## Core Software

- K230 视觉串口协议解析
- 云台 yaw / pitch 跟踪控制
- IMU 与航向数据融合参考
- 编码器反馈与底盘双轮控制
- 基于目标距离与偏差的低速跟随策略
- LCD 状态显示与串口调试输出
- I2C 异常检测与恢复机制

## Core Hardware

- Main controller: `STM32F407`
- Vision module: `K230`
- Gimbal actuator: `EMM_V5`
- IMU: `ICM42688`
- Compass: `QMC5883`
- Motor driver: `TB6612`
- Display: `ST7735 LCD`

## My Role

负责整车硬件设计与搭建、STM32 底层驱动开发、视觉通信协议对接、云台与底盘控制逻辑实现，以及整车联调与功能验证。
