# F407_TEST 整车联合调试检查清单

## 1. 上电前确认

- K230 TX 接 STM32 `USART6_RX / PC7`，两端共地。
- EMM_V5 云台驱动器接 `USART2 / PA2 PA3`，波特率 `256000`。
- `USART1` 接调试串口，波特率 `115200`。
- 云台地址保持 `yaw=1`、`pitch=2`。
- 底盘电机先架空或离地测试，避免首次方向错误直接冲出。

## 2. STM32 单独启动

只给 STM32 上电，不接 K230 目标输入，先看：

```text
MAIN: app start
GIMBAL: core ready
```

检查项：

- LCD 能刷新时间 `t` 和 IMU 相对 yaw `by`。
- K230 目标无效时 `v=0`。
- 底盘无目标时不转，电机速度显示接近 `l0.00 r0.00`。
- 若出现 `IMU: i2c recover`，先记录出现时间，不先改底层 I2C。

## 3. K230 输入检查

运行 `K230/main.py` 后，观察 USART1 或 LCD：

```text
K230: seq=... valid=1 dx=... dy=...
```

LCD 重点看：

```text
s: 帧序号
v: 目标有效标志
dx/dy: 目标相对画面中心偏差
```

判断：

- `s` 持续增加，说明 STM32 收到了 K230 帧。
- `v=1` 且 `dx/dy` 随目标移动变化，说明视觉链路有效。
- `s` 不变时，先查 K230 程序、TX/RX 接线、共地、`USART6=460800`。
- `v=0` 但 `s` 增加时，先查 K230 目标识别阈值和画面目标。

## 4. 云台方向检查

先让底盘离地或断开底盘动力，只测云台：

- 目标向画面左侧移动，观察 yaw 是否把目标拉回中心。
- 目标向画面右侧移动，观察 yaw 是否反向拉回中心。
- 目标向画面上方移动，观察 pitch 是否把目标拉回中心。
- 目标向画面下方移动，观察 pitch 是否反向拉回中心。

日志重点看：

```text
TRACK: seq=... dx=... dy=... yaw=... pitch=...
MOVE: addr=... clk=... vel=...
```

LCD 重点看：

```text
g: yaw S_CPOS 换算角
p: pitch S_CPOS 换算角
```

如果方向反了，只改对应轴方向，不同时改 K230 坐标和底盘转向。

## 5. 底盘低速跟随检查

确认云台方向正确后，再接底盘动力。默认参数：

```text
0..25 cm      stop forward
25..50 cm     0.4 rps
>50 cm        0.8 rps
max speed     0.8 rps
yaw deadzone  5 deg
dx deadzone   50 px
turn percent  130
```

检查项：

- 目标有效且距离小于等于 `25 cm` 时，底盘停止前进。
- 目标有效且距离在 `25-50 cm` 时，底盘约 `0.4 rps` 前进。
- 目标有效且距离大于 `50 cm` 时，底盘约 `0.8 rps` 前进。
- 目标丢失或 `v=0` 时，底盘刹车。
- 目标偏左/偏右时，底盘应配合云台 yaw 做差速转向。
- LCD 的 `l/r` 显示左右轮速度，方向判断以实际车轮为准。

`USART1` 在线调参：

```text
ag?      查询当前底盘转向百分比
ag 100   降低转向激进度
ag 130   当前默认值
ag 180   增强转向
```

如果底盘转向方向反了，优先只翻 `APP_CHASSIS_TURN_SIGN` 或
`APP_CHASSIS_DX_TURN_SIGN`，不要同时改云台方向。

## 6. 长时间运行观察

连续运行时记录：

- K230 帧序号是否持续增加。
- `v` 是否频繁从 1 掉到 0。
- `g/p` 是否持续有 S_CPOS 反馈。
- `l/r` 是否在目标丢失后回到 0。
- 是否出现 `K230: uart err recover`。
- 是否出现 `IMU: i2c recover`，以及出现前是否刚接入 K230 或电机动作。

## 7. 常见现象定位

```text
无 MAIN: app start
  -> 先查固件是否烧录到 F407_TEST，USART1 是否 115200

s 不增加
  -> 先查 K230 串口、PC7 接线、共地、USART6=460800

v=0 但 s 增加
  -> 先查 K230 识别阈值、目标颜色、画面目标大小

有 TRACK 但无 MOVE
  -> 查 track 到 motor 事件、motor 线程、命令队列

有 MOVE 但 g/p 不变
  -> 查 USART2=256000、EMM 地址、S_CPOS 回包

云台动但底盘不动
  -> 查目标 valid、距离是否小于等于 25 cm、MotorCtrl 输出和底盘供电

底盘无目标仍动
  -> 查 chassis 线程无效目标分支是否调用 MotorCtrl_Stop()

I2C 长时间失败
  -> 先记录 K230/电机供电和共地状态，再考虑硬件干扰
```
