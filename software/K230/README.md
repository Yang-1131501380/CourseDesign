# K230 红色目标追踪

这是恢复后的新版 K230 工程，用于识别无人机上的红色目标，并通过串口把
中心误差和距离估计发送给 `F407_TEST`。

## 当前版本

- 入口文件：`main.py`
- 检测方法：`Sensor.RGB565` + `img.find_blobs()` + LAB 红色阈值
- 串口：K230 `UART2`，FPIOA `GPIO11 TX / GPIO12 RX`
- 波特率：`256000`，对应 STM32 `USART6 / UART_MULTI_CH6`
- 协议：

```text
$K230,valid,cx,cy,dx,dy,dist,w,h*CS\r\n
```

`CS` 是从 `K230` 到最后一个数据字段的逐字节 XOR 校验。脚本会把发送的
完整串口包同时 `print` 到终端，便于联调时确认 K230 实际输出。

## 测距说明

目标按 `6.0 cm x 6.0 cm` 红色块处理。当前版本不再强依赖矩形角点 PnP，
而是使用 blob 宽、高、面积做单目近似测距，再套用实测线性修正：

```text
fixed_cm = 0.736 * raw_cm + 2.45
```

运行时会打印：

```text
raw: <raw_cm> fixed: <fixed_cm>
```

用于确认屏幕显示和串口发送使用的是修正后的距离。

## 与 STM32 对接

接线建议：

- K230 GPIO11 / UART2_TXD -> STM32 PC7 / USART6_RX
- K230 GPIO12 / UART2_RXD -> STM32 PC6 / USART6_TX
- K230 GND -> STM32 GND

`F407_TEST` 侧解析入口是 `HARDWARE/Emm_V5/k230_parser.c`，云台总线仍走
`USART2`，不要把 K230 改接到 `USART2`。

## 调试顺序

1. 先在 K230 画面确认红色目标能被稳定框住。
2. 再观察终端是否持续打印 `$K230,...*CS`。
3. 再看 STM32 `USART6` 是否收到新帧。
4. 距离偏差明显时，优先用新实测点调整 `correct_distance_cm()`。
