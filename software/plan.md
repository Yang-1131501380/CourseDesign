- # F407_TEST 重构计划（详细实施版）

  ## 当前优化版状态（2026-05-09）

  当前 Git 基线已保存为 `baseline-current`，开发分支为
  `feature/next-tracking-version`。旧计划中的工具函数、Emm_V5 通道拆分、
  `EmmV5Ctrl_Init(callback)`、ST7735 让出 CPU、ICM42688/AHRS 状态封装、
  底盘结构体封装和线程合并已经基本落到当前版本。

  本轮优化只补当前仍有价值的边界行为：

  - `valid=0` 的 K230 新帧会让追踪控制重置 PID，并按帧序号只触发一次停止事件。
  - 保留 `300 ms` 目标超时停止逻辑。
  - 同步 `project_summary.md` 和 `F407_TEST/AGENTS.md` 的当前四线程描述。
  - 不改变 K230 协议、USART6 输入、USART2 云台总线或底盘/IMU 主逻辑。

  ## 背景

  对 `F407_TEST/` 进行重构，参考 xrobot 组件化设计思想，利用 RT-Thread 特性：

  - 减少全局变量，优先用结构体+参数传递
  - 减少代码行数，能合并的合并，能内联的内联
  - 消除阻塞点（spin-wait / HAL_Delay）
  - 不改变现有功能和行为

  ---

  ## Phase 1: 工具函数统一

  ### 1.1 fast_math.h 新增工具函数

  **文件**: `HARDWARE/ICM42688/fast_math.h`

  在现有 `fast_math_clampf` 之后添加:

  ```c
  static inline float fast_math_deadzone(float v, float dz)
  {
      return ((v) > -(dz) && (v) < (dz)) ? 0.0f : (v);
  }
  
  static inline float fast_math_normalize_deg(float a)
  {
      while (a > 180.0f) {
          a -= 360.0f;
      }
      while (a < -180.0f) {
          a += 360.0f;
      }
      return a;
  }
  ```

  ### 1.2 motor.c 使用统一 clamp

  **文件**: `HARDWARE/Motor/motor.c`

  改动:

  - 添加 `#include "fast_math.h"`
  - `Motor_Clamp()` 实现改为调用 `fast_math_clampf()`

  ```c
  // Motor_Clamp 改为:
  static float Motor_Clamp(float value, float min_value, float max_value)
  {
      return fast_math_clampf(value, min_value, max_value);
  }
  ```

  ### 1.3 tb6612.c 使用统一 clamp

  **文件**: `HARDWARE/Motor/tb6612.c`

  改动:

  - 添加 `#include "fast_math.h"`
  - `TB6612_Clamp()` 实现改为调用 `fast_math_clampf()`

  ```c
  static int16_t TB6612_Clamp(int16_t value, int16_t limit)
  {
      return (int16_t)fast_math_clampf((float)value, (float)-limit, (float)limit);
  }
  ```

  ### 1.4 track_controller.c 使用统一 clamp

  **文件**: `HARDWARE/Emm_V5/track_controller.c`

  改动:

  - 添加 `#include "fast_math.h"`
  - `track_limit_clk()` 改为调用 `fast_math_clampf()`

  ```c
  static int32_t track_limit_clk(int32_t clk, int32_t limit)
  {
      return (int32_t)fast_math_clampf((float)clk, (float)-limit, (float)limit);
  }
  ```

  ---

  ## Phase 2: ICM42688 驱动层结构体化

  ### 2.1 ICM42688 状态封装

  **文件**: `HARDWARE/ICM42688/icm42688.c`

  将文件顶部:

  ```c
  // 现状:
  static float accSensitivity  = 0.244f;
  static float gyroSensitivity = 32.8f;
  static volatile uint8_t g_icm42688LastStatus = HAL_OK;
  static volatile uint32_t g_icm42688ErrorCount = 0U;
  ```

  改为:

  ```c
  typedef struct {
      float acc_sensitivity;
      float gyro_sensitivity;
      volatile uint8_t last_status;
      volatile uint32_t error_count;
  } ICM42688_STATE_S;
  
  static ICM42688_STATE_S g_icm_state = {
      .acc_sensitivity = 0.244f,
      .gyro_sensitivity = 32.8f,
      .last_status = HAL_OK,
      .error_count = 0U
  };
  ```

  所有引用处: `accSensitivity` → `g_icm_state.acc_sensitivity`，`gyroSensitivity` → `g_icm_state.gyro_sensitivity`，`g_icm42688LastStatus` → `g_icm_state.last_status`，`g_icm42688ErrorCount` → `g_icm_state.error_count`。

  ### 2.2 AHRS 状态封装

  **文件**: `HARDWARE/ICM42688/IMU.c`

  将文件顶部:

  ```c
  // 现状（file-static）:
  static float exInt = 0.0f;
  static float eyInt = 0.0f;
  static float ezInt = 0.0f;
  static float q0 = 1.0f;
  static float q1 = 0.0f;
  static float q2 = 0.0f;
  static float q3 = 0.0f;
  static uint32_t lastUpdate = 0;
  static uint32_t now = 0;
  static float TTangles_gyro[3] = {0.0f, 0.0f, 0.0f};
  static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
  static float Gyro_fill[3][20];
  static float Gyro_total[3] = {0.0f, 0.0f, 0.0f};
  static float sqrGyro_total[3] = {0.0f, 0.0f, 0.0f};
  static uint8_t GyroinitFlag = 0;
  static uint8_t GyroCount = 0;
  static uint8_t CalCount = 0;
  ```

  改为:

  ```c
  typedef struct {
      float q0, q1, q2, q3;
      float ex_int, ey_int, ez_int;
      uint32_t last_update;
      uint32_t now;
      float gyro_angles[3];
      float gyro_offset[3];
      float gyro_buf[3][20];
      float gyro_total[3];
      float sqr_total[3];
      uint8_t gyro_init_flag;
      uint8_t gyro_count;
      uint8_t cal_count;
  } AHRS_STATE_S;
  
  static AHRS_STATE_S g_ahrs = {
      .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
      .ex_int = 0.0f, .ey_int = 0.0f, .ez_int = 0.0f,
      .last_update = 0, .now = 0,
      .gyro_angles = {0}, .gyro_offset = {0},
      .gyro_buf = {{0}}, .gyro_total = {0}, .sqr_total = {0},
      .gyro_init_flag = 0, .gyro_count = 0, .cal_count = 0
  };
  ```

  所有引用处全字替换。

  ### 2.3 删除 IMU.h 中的 extern 声明

  **文件**: `HARDWARE/ICM42688/IMU.h`

  删除第38-39行:

  ```c
  extern volatile float yaw[5];
  extern float motion6[7];
  ```

  ### 2.4 HAL_Delay → rt_thread_mdelay

  **文件**: `HARDWARE/ICM42688/icm42688.c`

  删除:

  ```c
  static void icm42688_delay_ms(uint32_t delay_ms)
  {
      HAL_Delay(delay_ms);
  }
  ```

  第187行 `icm42688_delay_ms(100U);` → `rt_thread_mdelay(100U);`
  第206行 `icm42688_delay_ms(1U);` → `rt_thread_mdelay(1U);`

  ---

  ## Phase 3: Emm_V5 模块清理

  ### 3.1 删除 g_emm_v5_channel 全局

  **文件**: `HARDWARE/Emm_V5/Emm_V5.c`

  删除:

  ```c
  static UartMultiChannelId g_emm_v5_channel = UART_MULTI_CH2;
  ```

  删除 `Emm_V5_SetChannel()` 函数实现。

  `Emm_V5_Read_Sys_Params()` 改为直接使用参数传入的 channel:

  ```c
  // 现状:
  void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
  {
      ...
      (void)Emm_V5_WriteRawOn(g_emm_v5_channel, cmd, sizeof(cmd));
  }
  
  // 改为: 增加 channel 参数
  void Emm_V5_Read_Sys_Params(UartMultiChannelId ch, uint8_t addr, SysParams_t s)
  {
      ...
      (void)Emm_V5_WriteRawOn(ch, cmd, sizeof(cmd));
  }
  ```

  对应修改 `Emm_V5.h` 声明和 `emm_motor_driver.c` 的调用处（`emm_motor_poll_position()` 调用 `Emm_V5_Read_Sys_Params(pDriver->pos_wait_addr, S_CPOS)` 改为 `Emm_V5_Read_Sys_Params(pDriver->uart_ch, pDriver->pos_wait_addr, S_CPOS)`）。

  **文件**: `HARDWARE/Emm_V5/Emm_V5.h`

  删除:

  ```c
  void Emm_V5_SetChannel(UartMultiChannelId id);
  ```

  修改 `Emm_V5_Read_Sys_Params` 声明为增加 channel 参数。

  ### 3.2 EmmV5Ctrl 回调改为 Init 参数

  **文件**: `HARDWARE/Emm_V5/emm_v5_ctrl.h`

  ```c
  // 现状:
  void EmmV5Ctrl_Init(void);
  void EmmV5Ctrl_SetEventCallback(EmmV5CtrlEventCallback callback);
  
  // 改为:
  void EmmV5Ctrl_Init(EmmV5CtrlEventCallback event_cb);
  // 删除 EmmV5Ctrl_SetEventCallback
  ```

  **文件**: `HARDWARE/Emm_V5/emm_v5_ctrl.c`

  删除 `g_event_cb` 全局变量。

  修改 `EmmV5Ctrl_Init()`:

  ```c
  // 现状:
  static EmmV5CtrlEventCallback g_event_cb;  // 删除
  
  void EmmV5Ctrl_Init(void)
  {
      ...
  }
  
  void EmmV5Ctrl_SetEventCallback(EmmV5CtrlEventCallback callback)  // 删除
  {
      g_event_cb = callback;
  }
  
  // 改为:
  void EmmV5Ctrl_Init(EmmV5CtrlEventCallback event_cb)
  {
      ...
      ctrl_push_event = event_cb;  // 直接使用
  }
  ```

  同时将 `ctrl_push_event()` 改为直接使用函数指针参数，删除全局 `g_event_cb`。

  ### 3.3 删除 enter/exit critical (emm_v5_ctrl.c)

  **文件**: `HARDWARE/Emm_V5/emm_v5_ctrl.c`

  删除:

  ```c
  static uint32_t ctrl_enter_critical(void)
  {
      uint32_t primask;
      primask = __get_PRIMASK();
      __disable_irq();
      return primask;
  }
  
  static void ctrl_exit_critical(uint32_t primask)
  {
      if (primask == 0U) {
          __enable_irq();
      }
  }
  ```

  调用处改为:

  ```c
  uint32_t level = rt_hw_interrupt_disable();
  // ...访问共享数据...
  rt_hw_interrupt_enable(level);
  ```

  ### 3.4 删除 enter/exit critical (emm_motor_driver.c)

  **文件**: `HARDWARE/Emm_V5/emm_motor_driver.c`

  删除:

  ```c
  static uint32_t emm_motor_enter_critical(void) { ... }
  static void emm_motor_exit_critical(uint32_t primask) { ... }
  ```

  调用处 `level = emm_motor_enter_critical();` → `level = rt_hw_interrupt_disable();`
  调用处 `emm_motor_exit_critical(level);` → `rt_hw_interrupt_enable(level);`

  ---

  ## Phase 4: 底盘驱动结构体化

  ### 4.1 CHASSIS_S 结构体

  **文件**: `HARDWARE/Motor/motor_ctrl.c`

  将文件顶部:

  ```c
  static Encoder_Handle_t g_left_encoder = {
      .htim = &htim5,
      .direction = 1,
      .pulses_per_rev = MOTOR_CTRL_DEFAULT_PULSES_PER_REV
  };
  static Encoder_Handle_t g_right_encoder = {
      .htim = &htim3,
      .direction = -1,
      .pulses_per_rev = MOTOR_CTRL_DEFAULT_PULSES_PER_REV
  };
  static Motor_t g_left_motor;
  static Motor_t g_right_motor;
  static uint8_t g_motor_ctrl_ready;
  ```

  改为:

  ```c
  typedef struct {
      Encoder_Handle_t left_enc;
      Encoder_Handle_t right_enc;
      Motor_t left_motor;
      Motor_t right_motor;
      uint8_t ready;
  } CHASSIS_S;
  
  static CHASSIS_S g_chassis = {
      .left_enc = {
          .htim = &htim5,
          .direction = 1,
          .pulses_per_rev = MOTOR_CTRL_DEFAULT_PULSES_PER_REV
      },
      .right_enc = {
          .htim = &htim3,
          .direction = -1,
          .pulses_per_rev = MOTOR_CTRL_DEFAULT_PULSES_PER_REV
      },
      .ready = 0
  };
  ```

  `MotorCtrl_Init()` 改为:

  ```c
  void MotorCtrl_Init(void)
  {
      if (g_chassis.ready != 0U) {
          return;
      }
      Motor_Init(&g_chassis.left_motor, Motor_GetLeftChannel(),
                  &g_chassis.left_enc, ...);
      Motor_Init(&g_chassis.right_motor, Motor_GetRightChannel(),
                  &g_chassis.right_enc, ...);
      g_chassis.ready = 1U;
  }
  ```

  其余 `MotorCtrl_*` 函数同样将 `g_motor_ctrl_ready` 改为 `g_chassis.ready`，`g_left_motor` 改为 `g_chassis.left_motor` 等。

  ---

  ## Phase 5: 线程合并（7→5）

  ### 5.1 合并 k230 + motor + track 为 ctrl_thread

  **文件**: `Core/Src/app.c`

  **删除的全局变量**:

  ```c
  static struct rt_thread g_k230_thread;
  static struct rt_thread g_motor_thread;
  static struct rt_thread g_track_thread;
  static rt_uint8_t g_k230_stack[APP_K230_STACK_SIZE];
  static rt_uint8_t g_motor_stack[APP_MOTOR_STACK_SIZE];
  static rt_uint8_t g_track_stack[APP_TRACK_STACK_SIZE];
  static struct rt_semaphore g_k230_rx_sem;
  static rt_semaphore g_track_sem;
  static rt_semaphore g_motor_sem;
  static APP_WORKER_S g_k230_worker;
  static APP_WORKER_S g_motor_worker;
  static APP_WORKER_S g_track_worker;
  ```

  **新增的全局变量**:

  ```c
  static struct rt_thread g_ctrl_thread;
  static rt_uint8_t g_ctrl_stack[4096U];
  ```

  **新增线程入口**:

  ```c
  static void ctrl_thread_entry(void *pArg)
  {
      (void)pArg;
      while (1) {
          uint32_t now = app_tick_ms();
          EmmV5Ctrl_K230Process(now);
          EmmV5Ctrl_MotorProcess(now);
          EmmV5Ctrl_TrackProcess(now);
          rt_thread_delay(app_ms_to_tick(APP_CTRL_PERIOD_MS));
      }
  }
  ```

  其中 `APP_CTRL_PERIOD_MS` 定义为 `10U`。

  **删除的函数**:

  - `process_thread_entry(void *pArg)` - 替换为 `ctrl_thread_entry()`
  - `app_k230_rx_callback()` - 不再需要，K230 解析在 ctrl_thread 内完成

  **app_main() 改动**:

  ```c
  // 删除这三个信号量的 rt_sem_init:
  #if 0
  (void)rt_sem_init(&g_k230_rx_sem, "k230rx", 0U, RT_IPC_FLAG_PRIO);
  (void)rt_sem_init(&g_track_sem, "trkev", 0U, RT_IPC_FLAG_PRIO);
  (void)rt_sem_init(&g_motor_sem, "motorev", 0U, RT_IPC_FLAG_PRIO);
  #endif
  
  // EmmV5Ctrl_Init 改为传入 event_cb 参数:
  // 现状: EmmV5Ctrl_Init(); EmmV5Ctrl_SetEventCallback(app_publish_event);
  // 改为:
  EmmV5Ctrl_Init(app_publish_event);
  
  // 删除:
  // UartMulti_SetRxCallback(UART_MULTI_CH6, app_k230_rx_callback, RT_NULL);
  
  // 启动合并后的 ctrl_thread:
  app_thread_start(&g_ctrl_thread, "ctrl", ctrl_thread_entry, RT_NULL,
                   g_ctrl_stack, sizeof(g_ctrl_stack), 10U);
  
  // 删除原三个线程的启动:
  #if 0
  app_thread_start(&g_k230_thread, APP_K230_THREAD_NAME, process_thread_entry,
                   &g_k230_worker, g_k230_stack, sizeof(g_k230_stack), 10U);
  app_thread_start(&g_motor_thread, APP_MOTOR_THREAD_NAME, process_thread_entry,
                   &g_motor_worker, g_motor_stack, sizeof(g_motor_stack), 11U);
  app_thread_start(&g_track_thread, APP_TRACK_THREAD_NAME, process_thread_entry,
                   &g_track_worker, g_track_stack, sizeof(g_track_stack), 12U);
  #endif
  ```

  **删除 APP_WORKER_S 相关**:

  ```c
  typedef struct {
      struct rt_semaphore *p_sem;
      uint32_t period_ms;
      APP_PROCESS_FUNC process;
  } APP_WORKER_S;
  ```

  ---

  ## Phase 6: 消除阻塞

  ### 6.1 ST7735 spin-wait 加 rt_thread_yield

  **文件**: `HARDWARE/ST7735/st7735.c`

  第136-141行:

  ```c
  // 现状:
  static void ST7735_WaitIdle(void)
  {
      while ((g_st7735AsyncText.dmaBusy != 0U) ||
             (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)) {
      }
  }
  
  // 改为:
  static void ST7735_WaitIdle(void)
  {
      while ((g_st7735AsyncText.dmaBusy != 0U) ||
             (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)) {
          rt_thread_yield();
      }
  }
  ```

  ### 6.2 ST7735 HAL_Delay → rt_thread_mdelay

  **文件**: `HARDWARE/ST7735/st7735.c`

  第128行: `HAL_Delay(20U);` → `rt_thread_mdelay(20U);`
  第132行: `HAL_Delay(20U);` → `rt_thread_mdelay(20U);`
  第194行: `HAL_Delay(120U);` → `rt_thread_mdelay(120U);`
  第197行: `HAL_Delay(120U);` → `rt_thread_mdelay(120U);`
  第299行: `HAL_Delay(20U);` → `rt_thread_mdelay(20U);`

  ---

  ## Phase 7: app.c 精简

  ### 7.1 删除 g_shared

  **文件**: `Core/Src/app.c`

  删除:

  ```c
  typedef struct {
      float body_yaw;
      float body_yaw_zero;
      uint8_t body_yaw_zero_ready;
      float gimbal_yaw_zero;
      uint8_t gimbal_yaw_zero_ready;
      float chassis_left_rps;
      float chassis_right_rps;
      uint16_t chassis_turn_percent;
  } APP_SHARED_S;
  
  static APP_SHARED_S g_shared;
  ```

  body_yaw_zero / gimbal_yaw_zero 相关逻辑改为线程本地变量。mon 线程自行采样构建 APP_STATE_S。

  ### 7.2 删除/简化辅助函数

  **删除的函数**（改用 fast_math）:

  ```c
  // 删除:
  static float app_clamp_float(float value, float minValue, float maxValue)
  {
      if (value > maxValue) return maxValue;
      if (value < minValue) return minValue;
      return value;
  }
  
  // 删除:
  static float app_normalize_deg(float angle) { ... }
  
  // 删除:
  static float app_deadzone_float(float value, float deadzone) { ... }
  
  // 删除:
  static uint32_t app_enter_critical(void) { ... }
  static void app_exit_critical(uint32_t primask) { ... }
  ```

  **app_float_scale** 内联到调用处（lcd_thread_entry 中两处）。

  ### 7.3 lcd_show_line 简化

  ```c
  // 现状: 变参版本
  static void lcd_show_line(uint8_t row, uint16_t color, const char *pFormat, ...) {
      char line[APP_LCD_LINE_LEN];
      va_list args;
      va_start(args, pFormat);
      (void)vsnprintf(line, sizeof(line), pFormat, args);
      va_end(args);
      UiText_ShowString(0U, (uint16_t)row * 16U, line, color, ST7735_COLOR_BLACK);
  }
  
  // 调用: lcd_show_line(0U, ST7735_COLOR_WHITE, "t%lu by%ld.%01ld", ...);
  
  // 改为: 预格式化版本
  static void lcd_show_line(uint8_t row, uint16_t color, const char *pStr)
  {
      char line[APP_LCD_LINE_LEN];
      snprintf(line, sizeof(line), "%-16s", pStr);
      UiText_ShowString(0U, (uint16_t)row * 16U, line, color, ST7735_COLOR_BLACK);
  }
  
  // 调用处改为 snprintf 到临时变量:
  // char line0[APP_LCD_LINE_LEN];
  // snprintf(line0, sizeof(line0), "t%lu by%ld.%01ld", ...);
  // lcd_show_line(0U, ST7735_COLOR_WHITE, line0);
  ```

  ---

  ## 文件修改清单（汇总）

  | 序号 | 文件                                  | 修改内容                                                     |
  | ---- | ------------------------------------- | ------------------------------------------------------------ |
  | 1    | `HARDWARE/ICM42688/fast_math.h`       | 添加 `fast_math_deadzone`, `fast_math_normalize_deg` inline  |
  | 2    | `HARDWARE/Motor/motor.c`              | `Motor_Clamp` 改用 `fast_math_clampf`；添加 `#include "fast_math.h"` |
  | 3    | `HARDWARE/Motor/tb6612.c`             | `TB6612_Clamp` 改用 `fast_math_clampf`；添加 `#include "fast_math.h"` |
  | 4    | `HARDWARE/Emm_V5/track_controller.c`  | `track_limit_clk` 改用 `fast_math_clampf`；添加 `#include "fast_math.h"` |
  | 5    | `HARDWARE/ICM42688/icm42688.c`        | 封装 `ICM42688_STATE_S`；删除 `icm42688_delay_ms`；`HAL_Delay` → `rt_thread_mdelay` |
  | 6    | `HARDWARE/ICM42688/IMU.c`             | 封装 `AHRS_STATE_S`；所有 file-static 变量结构体化           |
  | 7    | `HARDWARE/ICM42688/IMU.h`             | 删除 `extern yaw[5]`, `extern motion6[7]`                    |
  | 8    | `HARDWARE/Emm_V5/Emm_V5.c`            | 删除 `g_emm_v5_channel`；`Emm_V5_Read_Sys_Params` 增加 channel 参数 |
  | 9    | `HARDWARE/Emm_V5/Emm_V5.h`            | 删除 `Emm_V5_SetChannel`；修改 `Emm_V5_Read_Sys_Params` 声明 |
  | 10   | `HARDWARE/Emm_V5/emm_v5_ctrl.c`       | `g_event_cb` 改为 Init 参数；删除 `ctrl_enter/exit_critical` |
  | 11   | `HARDWARE/Emm_V5/emm_v5_ctrl.h`       | `EmmV5Ctrl_Init()` 签名增加 event_cb 参数；删除 `EmmV5Ctrl_SetEventCallback` |
  | 12   | `HARDWARE/Emm_V5/emm_motor_driver.c`  | 删除 `emm_motor_enter/exit_critical`；更新 `Emm_V5_Read_Sys_Params` 调用 |
  | 13   | `HARDWARE/Motor/motor_ctrl.c`         | 封装 `CHASSIS_S`；删除 `g_left/right_encoder/motor`、`g_motor_ctrl_ready` |
  | 14   | `HARDWARE/ST7735/st7735.c`            | `HAL_Delay` → `rt_thread_mdelay`；`ST7735_WaitIdle` 加 `rt_thread_yield` |
  | 15   | `Core/Src/app.c`                      | 线程 7→5；删除 `g_shared`/`g_worker`/信号量/`process_thread_entry`；简化辅助函数 |
  | 16   | `Core/Src/main.h` / `Core/Src/main.c` | 检查是否需要配合修改（如有回调变更）                         |

  ---

  ## 实施顺序（依赖关系）

  ```
  Phase 1 (工具函数统一)
      ↓
  Phase 3 (Emm_V5 清理，依赖 Phase 1 的 rt_hw_interrupt_*)
      ↓
  Phase 2 (ICM42688 结构体化)
      ↓
  Phase 4 (底盘结构体化)
      ↓
  Phase 6 (ST7735)
      ↓
  Phase 7 (app.c 精简)
      ↓
  Phase 5 (线程合并，最后一步)
  ```

  **关键**: Phase 5 是最后一步，因为它依赖前面所有 phase 对 `EmmV5Ctrl_*` 接口的修改（特别是 EmmV5Ctrl_Init 的新签名）。

  ---

  ## 功能验证

  1. **编译**: EIDE 编译无 error/warning
  2. **K230**: USART1 输出 `K230: seq=X valid=Y dx=Z dy=W`
  3. **云台**: LCD 显示 `gX.X pY.Y` 更新，`S_CPOS` 反馈正常
  4. **底盘**: `ag 130` 改变转向，`ag?` 查询当前值
  5. **目标丢失**: 底盘自动刹车（`MotorCtrl_Stop()`）
  6. **IMU恢复**: 长时运行后 I2C 错误计数变化时日志输出 `IMU: i2c recover err=X`
