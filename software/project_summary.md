# 工程总览与接手说明

## 当前目标

当前工作区用于 K230 视觉、STM32F407 云台追踪、IMU 姿态和底盘低速跟随的
整车联合调试。主修改目录是 `F407_TEST/`。

```text
K230视觉检测 -> STM32 USART6 -> 云台追踪 -> 底盘低速跟随 -> LCD/USART1观察
```

`F407_K230_GIMBAL_TEST/` 是历史专项联调工程。除非任务明确点名它，否则当前排查、
参数调整和文档同步都落到 `F407_TEST/`。

## 共享硬件事实

```text
USART1 / UART_MULTI_CH1  调试日志、手动命令
USART2 / UART_MULTI_CH2  EMM_V5 云台驱动器总线，256000
USART6 / UART_MULTI_CH6  K230 输入，460800，PC7
I2C1 PB6/PB7             ICM42688
云台地址                 yaw=1，pitch=2
```

K230 协议保持不变：

```text
$K230,valid,cx,cy,dx,dy,dist,w,h*CS\r\n
```

## F407_TEST 当前实现

`F407_TEST/Core/Src/app.c` 是应用层入口。它创建四个 RT-Thread 线程：

```text
ctrl     10 ms 周期处理 K230 解析、云台运动、S_CPOS 和追踪控制
mon      10 ms 推进 LCD DMA、处理控制事件、USART1 调参，200 ms 刷新状态
imu      25 ms 姿态采样，I2C 失败后应用层恢复
chassis  10 ms 底盘跟随，目标有效时低速运动，目标丢失时刹车
```

关键文件：

- `F407_TEST/Core/Src/app.c`
- `F407_TEST/HARDWARE/Emm_V5/emm_v5_ctrl.c`
- `F407_TEST/HARDWARE/Emm_V5/k230_parser.c`
- `F407_TEST/HARDWARE/Emm_V5/track_controller.c`
- `F407_TEST/HARDWARE/Emm_V5/emm_motor_driver.c`
- `F407_TEST/HARDWARE/Motor/`
- `F407_TEST/HARDWARE/UART_MULTI/`

底层 `UART_MULTI` 使用 DMA + kfifo。RX ISR/回调只推 FIFO 并释放信号量，业务解析和
控制都在线程内完成。`Emm_V5` 只保留当前链路需要的原始读写、`S_CPOS` 查询和
CPOS/错误/ACK 解析。LCD 文本刷新采用 DMA 队列，SPI 完成回调只更新状态，后续
文本启动由 `mon` 线程内的 `UiText_Process()` 推进。

## 关键参数

云台追踪：

```text
target timeout     300 ms
track period       20 ms
track velocity     45 rpm
deadzone           50 px
output range       80..420 clk
pos period         100 ms
```

底盘跟随：

```text
0..25 cm           stop forward
25..50 cm          0.4 rps
>50 cm             0.8 rps
max speed          0.8 rps
yaw deadzone       5 deg
dx deadzone        50 px
turn default       130%
```

云台 `S_CPOS` 按 `51200 clk = 360 deg` 换算。LCD 的 `g/p` 是云台 yaw/pitch 位置
反馈换算角，`by` 是底盘 IMU 上电稳定后的相对 yaw。

`USART1` 可在线调底盘转向激进度：

```text
ag?      查询当前百分比
ag 100   基础转向
ag 130   默认略激进
ag 180   更激进
```

## 联调顺序

1. 只接 STM32，上电看 USART1 是否输出 `MAIN: app start`。
2. 确认 `USART2=256000`、`USART6=460800`。
3. 确认底盘在无目标时不会乱动。
4. 接 K230 到 `USART6_RX/PC7` 并共地，运行 `K230/main.py`。
5. 看 LCD/日志：`v=1`、`dx/dy` 合理、`g/p` 有反馈。
6. 左右移动目标确认 yaw 方向，上下移动目标确认 pitch 方向。
7. 目标有效时观察底盘低速跟随，目标丢失时必须刹车。
8. 长时间运行观察 I2C 恢复日志，先不改底层 I2C/CubeMX。

更细的现场检查项见 `F407_TEST/docs/joint_debug_checklist.md`。
