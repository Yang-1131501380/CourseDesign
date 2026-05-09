# F407_TEST 协作说明

## 1. 当前仓库定位

`F407_TEST` 是当前整车联合调试主工程。默认目标不是云台专项，而是：

```text
K230视觉输入 -> 二维云台追踪 -> IMU姿态参考 -> 底盘低速跟随 -> LCD/USART1观察
```

## 2. 当前实际运行状态

主流程：

- `Core/Src/main.c`
  负责 HAL、时钟、GPIO、DMA、UART、I2C、SPI、定时器等基础初始化，然后进入
  `app_main()`。
- `Core/Src/app.c`
  应用层入口，创建 `ctrl`、`mon`、`imu`、`chassis` 四个 RT-Thread 线程。
- `HARDWARE/Emm_V5/emm_v5_ctrl.c`
  云台控制协调器，只编排 K230 解析、跟踪控制、电机驱动和状态导出。
- `HARDWARE/Emm_V5/k230_parser.c`
  只从 `USART6 / UART_MULTI_CH6` 取 FIFO 数据并解析 K230 帧。
- `HARDWARE/Emm_V5/track_controller.c`
  把最新目标偏差转换成覆盖式云台运动命令。
- `HARDWARE/Emm_V5/emm_motor_driver.c`
  是 `USART2 / UART_MULTI_CH2` 的唯一云台电机总线使用者。

线程分工：

```text
ctrl     -> 10 ms 周期处理 K230、云台电机、S_CPOS 和追踪控制
mon      -> 10 ms 推进 LCD DMA + 控制事件日志 + 200 ms LCD 刷新 + USART1 调参
imu      -> 25 ms 姿态采样，I2C 失败后应用层恢复
chassis  -> 10 ms 低速跟随，目标丢失时刹车
```

## 3. 串口、总线与地址

```text
USART1 / UART_MULTI_CH1  调试日志、手动命令，115200
USART2 / UART_MULTI_CH2  EMM_V5 云台驱动器总线，256000
USART6 / UART_MULTI_CH6  真实 K230 输入，256000
I2C1 PB6/PB7             ICM42688/QMC5883P 等 I2C 设备
```

注意：

- K230 真实输入走 `USART6`，不要接到 `USART2`。
- `USART2` 固定给 EMM_V5 云台驱动器。
- `USART1` 只保留调试和命令用途。
- `USART6_RX / PC7` 当前有上拉，用于减少 K230 未稳定时的串口噪声。
- 没有明确要求时，不要改底层 I2C 初始化或 CubeMX 配置。

云台地址固定：

- `addr=1`：yaw
- `addr=2`：pitch

一圈脉冲定义：

- `51200 clk = 360 deg`

## 4. 当前控制事实

- K230 帧格式为 `$K230,valid,cx,cy,dx,dy,dist,w,h*CS`。
- `CS` 按两位大写十六进制 XOR 校验。
- 每个新 K230 帧都会更新目标快照和 `frame_seq`。
- `track_controller.c` 只消费新 K230 坐标，没有新帧时不重复使用旧 `dx/dy`。
- K230 发来 `valid=0` 的新帧时，追踪控制会重置 PID 并触发一次停止事件。
- 目标帧超过 `300 ms` 后停止继续跟踪。
- 跟踪周期为 `20 ms`。
- 跟踪速度为 `45 rpm`。
- PID 默认值为 yaw/pitch `kp=0.6, kd=0.12`。
- 死区 `50 px`，输出最大 `420 clk`。
- 云台真实位置通过 `S_CPOS` 查询。
- LCD 的 `g/p` 是云台 yaw/pitch 绝对位置反馈换算角。
- LCD 的 `by` 是底盘 IMU 上电稳定后的相对 yaw。
- 底盘转向使用云台 yaw 相对上电零点角，云台接近中位后再用视觉 `dx` 微调。
- `USART1` 可用 `ag?`、`ag 100`、`ag 130`、`ag 180` 调底盘转向激进度。
- 目标无效时底盘必须调用 `MotorCtrl_Stop()`。
- K230、TRACK、MOVE 调试日志在 `app.c` 的 `mon` 线程节流。
- ST7735 的 SPI 完成回调只更新 DMA 状态，不在回调内启动下一条文本。

## 5. IMU 与 I2C 当前状态

接上 K230 后，系统运行一段时间可能出现 I2C 通信失败。当前应用层处理：

- `imu` 线程每 `25 ms` 调用 `IMU_getYawPitchRoll()`。
- 通过 `bsp_Icm42688GetErrorCount()` 判断是否发生 I2C 错误。
- 错误计数变化时，不发布新的 `by`。
- 连续失败超过 `2 s` 后恢复 I2C1。
- 恢复后延时 `4 s`，再执行 `IMU_init()`。

相关文件优先看：

1. `Core/Src/app.c`
2. `HARDWARE/ICM42688/icm42688.c`
3. `HARDWARE/ICM42688/IMU.c`
4. `Core/Src/i2c.c`

## 6. 读代码顺序

第一次接手建议按下面顺序看：

1. `Core/Src/main.c`
2. `Core/Src/app.c`
3. `HARDWARE/Emm_V5/emm_v5_ctrl.c`
4. `HARDWARE/Emm_V5/k230_parser.c`
5. `HARDWARE/Emm_V5/track_controller.c`
6. `HARDWARE/Emm_V5/emm_motor_driver.c`
7. `Core/Src/usart.c`
8. `HARDWARE/UART_MULTI/`
9. `HARDWARE/Motor/`
10. `HARDWARE/ICM42688/`
11. `HARDWARE/ST7735/`

现场联调步骤见 `docs/joint_debug_checklist.md`。

## 7. 当前怎么配合工作

- 默认围绕整车联合调试回答和修改。
- 当前链路相关修改优先落在 `app.c`、`HARDWARE/Emm_V5/`、`HARDWARE/Motor/`。
- 不要随意改 `USART2` 所有权。
- 不要把底盘 yaw 和云台 yaw/pitch 混成一个状态值。
- 新的或修改过的 C 代码按 `docs/autostar_coding_standard.md` 执行，但只约束本次
  修改到的行，不批量重排历史代码。
