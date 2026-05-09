# 工作区协作说明

## 1. 当前目标

这个工作区用于整车联合调试。当前主修改目标是 `F407_TEST/`，不要再把
`F407_K230_GIMBAL_TEST/` 当成默认调试入口。

当前链路：

```text
K230视觉检测 -> STM32 USART6解析 -> EMM_V5二维云台追踪 ->
IMU姿态参考 -> 底盘低速跟随 -> LCD/USART1观察
```

子目录分工：

- `K230/`：视觉检测、距离估计、串口发送
- `F407_TEST/`：当前主集成工程，默认跑 K230、云台、IMU、底盘、LCD
- `F407_K230_GIMBAL_TEST/`：历史专项联调工程，只在明确要求排查云台专项时再看

## 2. 先读什么

开始改代码、读架构、或者回答工程问题前，按下面顺序读：

1. `project_summary.md`
2. 本文件
3. `F407_TEST/AGENTS.md`
4. `F407_TEST/docs/autostar_coding_standard.md`
5. `F407_TEST/docs/joint_debug_checklist.md`

如果任务明确点名 `F407_K230_GIMBAL_TEST/`，再读该目录自己的 `AGENTS.md` 和
`K230_GIMBAL_TEST_README.md`。

## 3. 当前串口和硬件分工

这些分工是当前联合调试的前提：

- `USART1 / UART_MULTI_CH1`
  调试日志和手动命令口
- `USART2 / UART_MULTI_CH2`
  EMM_V5 二维云台驱动器总线，`256000`
- `USART6 / UART_MULTI_CH6`
  K230 输入，`256000`，`USART6_RX/PC7`
- `I2C1 / PB6 PB7`
  ICM42688 等 I2C 设备
- 云台电机地址固定：
  - `addr=1`：yaw
  - `addr=2`：pitch

不要把 K230 改接到 `USART2`，也不要改 K230 协议：

```text
$K230,valid,cx,cy,dx,dy,dist,w,h*CS\r\n
```

## 4. F407_TEST 当前事实

- 应用层入口是 `F407_TEST/Core/Src/app.c`
- 云台协调入口是 `F407_TEST/HARDWARE/Emm_V5/emm_v5_ctrl.c`
- K230 解析在 `F407_TEST/HARDWARE/Emm_V5/k230_parser.c`
- 云台电机总线在 `F407_TEST/HARDWARE/Emm_V5/emm_motor_driver.c`
- 底盘接口在 `F407_TEST/HARDWARE/Motor/`
- LCD 显示当前关注 `t/by/s/v/dx/dy/l/r/g/p`

当前线程结构：

```text
k230     USART6 RX 唤醒，20 ms 超时兜底
motor    TRACK 事件唤醒，10 ms 超时兜底
track    K230 新帧唤醒，30 ms 控制节流
mon      事件队列 + 200 ms LCD 快照
imu      25 ms 姿态采样，I2C 失败后应用层恢复
chassis  10 ms 底盘低速跟随，目标丢失刹车
lcd      阻塞等待状态快照，只负责显示
```

## 5. 当前控制范围

- K230 数据从 `USART6` 进入 STM32
- 云台总线固定走 `USART2`
- 真实位置反馈优先看 `S_CPOS`
- `S_CPOS` 角度换算按 `51200 clk = 360 deg`
- 目标帧超过 `300 ms` 后视为无效
- 云台跟踪速度当前为 `25 rpm`
- 死区 `10 px`，单次输出 `80 clk` 到 `300 clk`
- 底盘默认低速前进，目标丢失时必须 `MotorCtrl_Stop()`
- `USART1` 可用 `ag?`、`ag 100`、`ag 130`、`ag 180` 调底盘转向激进度

底盘 IMU 的 `by` 是相对上电零点 yaw，底盘转向用云台 yaw 相对上电零点角把云台
拉回中位。不要把底盘 yaw 和云台 yaw/pitch 混成一个状态量。

## 6. I2C 当前问题

已知现象：接上 K230 后，系统运行一段时间可能出现 I2C 通信失败。这不是上电
初始化失败。

当前处理方向：

- IMU 错误计数变化时，不发布新的姿态结果
- 连续失败超过 `2 s` 后，由应用层 `HAL_I2C_DeInit(&hi2c1)` +
  `MX_I2C1_Init()` 恢复 I2C1
- 恢复后延时 `4 s`，再执行 `IMU_init()`
- 后续排查先看 `F407_TEST/Core/Src/app.c` 的 `imu` 线程和 ICM42688 错误计数路径
- 不要先改底层 I2C 引脚、CubeMX 配置或生成初始化代码

## 7. 修改原则

- 先看真实本地文件，再下判断
- 当前默认围绕 `F407_TEST/` 整车联合调试展开
- 只改和当前任务直接相关的文件
- 不改无关第三方、生成代码、历史稳定代码
- 不要为了“更优雅”重写已跑通链路
- 如果同时涉及 `K230` 和 STM32，必须按真实串口协议和 UART 分工描述
