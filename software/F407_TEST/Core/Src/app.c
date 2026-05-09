/*==============================================================================
 * F407_TEST 联调入口：K230 -> 云台 -> IMU 车体 yaw -> 底盘跟随 -> LCD/日志
 *============================================================================*/

#include "main.h"
#include "rtthread.h"
#include "rthw.h"
#include "uart_multi.h"
#include "emm_v5_ctrl.h"
#include "ui_text.h"
#include "motor_ctrl.h"
#include "IMU.h"
#include "icm42688.h"
#include "fast_math.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>

/*=============================== Macro Define ===============================*/
#define APP_CTRL_STACK_SIZE          4096U /**< K230/云台控制线程栈 */
#define APP_MONITOR_STACK_SIZE       4096U /**< 日志和 LCD 显示线程栈 */
#define APP_IMU_STACK_SIZE           4096U /**< ICM42688 采样线程栈 */
#define APP_CHASSIS_STACK_SIZE       4096U /**< 底盘跟随线程栈 */

#define APP_CTRL_PERIOD_MS           10U  /**< K230/云台轮询周期 */
#define APP_MONITOR_PERIOD_MS        10U  /**< 日志/LCD 推进周期 */
#define APP_LCD_PERIOD_MS            200U /**< LCD 状态刷新周期 */
#define APP_CHASSIS_PERIOD_MS        10U  /**< 底盘控制周期 */

#define APP_LCD_LINE_LEN             17U /**< ST7735 单行 16 字符+\0 */
#define APP_EVENT_MSG_NUM            12U /**< 控制事件队列深度 */
#define APP_MQ_ITEM_SIZE(type)       (RT_ALIGN(sizeof(type), RT_ALIGN_SIZE) + sizeof(void *))

#define APP_IMU_PERIOD_MS            25U   /**< IMU 采样周期 */
#define APP_IMU_FAIL_RECOVER_MS      2000U /**< I2C 连续失败恢复阈值 */
#define APP_IMU_RECOVER_DELAY_MS     4000U /**< I2C 重建后稳定等待 */
#define APP_IMU_ZERO_DELAY_MS        500U  /**< 上电后 yaw 置零等待 */
#define APP_IMU_YAW_OFFSET_DEG       24.0f /**< 当前安装姿态 yaw 修正 */

#define APP_DEBUG_LOG_PERIOD_MS      500U    /**< 成功类串口日志限速 */
#define APP_DROP_LOG_PERIOD_MS       1000U   /**< 事件丢弃日志限速 */
#define APP_CHASSIS_MID_FORWARD_RPS  0.4f    /**< 中距离跟随速度 */
#define APP_CHASSIS_FAR_FORWARD_RPS  0.8f    /**< 远距离跟随速度 */
#define APP_CHASSIS_MAX_SPEED_RPS    0.8f    /**< 左右轮最大速度限制 */
#define APP_CHASSIS_STOP_DIST_CM     25.0f   /**< 小于等于该距离停止前进 */
#define APP_CHASSIS_FAR_DIST_CM      50.0f   /**< 大于该距离使用远距离速度 */
#define APP_CHASSIS_YAW_DEADZONE_DEG 5.0f    /**< 云台 yaw 回中死区 */
#define APP_CHASSIS_TURN_GAIN_DEG    0.015f  /**< 云台 yaw 差速增益 */
#define APP_CHASSIS_TURN_SIGN        -1.0f   /**< 云台 yaw 转向方向 */
#define APP_CHASSIS_DX_DEADZONE_PX   50.0f   /**< 视觉 dx 微调死区 */
#define APP_CHASSIS_DX_TURN_GAIN     0.0008f /**< 视觉 dx 差速增益 */
#define APP_CHASSIS_DX_TURN_SIGN     -1.0f   /**< 视觉 dx 转向方向 */
#define APP_CHASSIS_TURN_DEFAULT     130U    /**< 默认转向百分比 */
#define APP_CHASSIS_TURN_MIN         50U     /**< 最小转向百分比 */
#define APP_CHASSIS_TURN_MAX         250U    /**< 最大转向百分比 */
#define APP_CMD_LINE_LEN             16U     /**< USART1 调参命令长度 */

/*=============================== Type Define ================================*/
typedef struct {
    uint32_t uptime_ms;         /**< 系统运行时间 */
    float body_yaw;             /**< IMU 底盘相对 yaw */
    float chassis_left_rps;     /**< 左轮速度 */
    float chassis_right_rps;    /**< 右轮速度 */
    EmmV5CtrlTrackStatus track; /**< K230 和云台状态 */
} APP_STATE_S;

/*============================== Global Variable =============================*/
static struct rt_thread g_ctrl_thread;
static struct rt_thread g_monitor_thread;
static struct rt_thread g_imu_thread;
static struct rt_thread g_chassis_thread;
static rt_uint8_t g_ctrl_stack[APP_CTRL_STACK_SIZE];
static rt_uint8_t g_monitor_stack[APP_MONITOR_STACK_SIZE];
static rt_uint8_t g_imu_stack[APP_IMU_STACK_SIZE];
static rt_uint8_t g_chassis_stack[APP_CHASSIS_STACK_SIZE];
static struct rt_messagequeue g_event_mq;
static uint8_t g_event_mq_pool[APP_MQ_ITEM_SIZE(EmmV5CtrlEvent) *
                               APP_EVENT_MSG_NUM];
static float g_body_yaw;
static float g_chassis_left_rps;
static float g_chassis_right_rps;
static uint16_t g_chassis_turn_percent = APP_CHASSIS_TURN_DEFAULT;
static uint8_t g_body_yaw_zero_reset = 1U;
static uint32_t g_event_drop_count;

/*============================ Internal Prototype ============================*/
static uint32_t app_tick_ms(void);
static int32_t app_float_scale(float value, float scale);
static void app_thread_start(struct rt_thread *pThread,
                             const char *pName,
                             void (*entry)(void *parameter),
                             void *pArg,
                             void *pStack,
                             rt_uint32_t stackSize,
                             rt_uint8_t priority);
static void app_publish_event(const EmmV5CtrlEvent *pEvent);
static uint32_t app_get_event_drop_count(void);
static void app_set_body_yaw(float bodyYaw);
static void app_reset_body_yaw_zero(void);
static void app_set_chassis_speed(float leftRps, float rightRps);
static uint16_t app_get_chassis_turn_percent(void);
static void app_set_chassis_turn_percent(int32_t percent);
static void app_fill_state(APP_STATE_S *pState);
static void app_log_event(const EmmV5CtrlEvent *pEvent);
static void app_log_event_drop(uint32_t nowTick);
static void app_poll_debug_cmd(void);
static void lcd_show_state(const APP_STATE_S *pState);
static void ctrl_thread_entry(void *pArg);
static void monitor_thread_entry(void *pArg);
static void imu_thread_entry(void *pArg);
static void chassis_thread_entry(void *pArg);
static void app_init_threads(void);

/*============================ Internal Functions ============================*/
static uint32_t app_tick_ms(void)
{
    return (uint32_t)(((uint64_t)rt_tick_get() * 1000ULL) /
                      RT_TICK_PER_SECOND);
}

static int32_t app_float_scale(float value, float scale)
{
    float scaled;

    scaled = value * scale;

    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void app_thread_start(struct rt_thread *pThread,
                             const char *pName,
                             void (*entry)(void *parameter),
                             void *pArg,
                             void *pStack,
                             rt_uint32_t stackSize,
                             rt_uint8_t priority)
{
    if (rt_thread_init(pThread, pName, entry, pArg, pStack, stackSize,
                       priority, 10U) == RT_EOK) {
        rt_thread_startup(pThread);
    } else {
        UartMulti_Printf(UART_MULTI_CH1, "%s: thread init failed\r\n",
                         pName);
    }
}

static void app_publish_event(const EmmV5CtrlEvent *pEvent)
{
    EmmV5CtrlEvent drop;
    rt_base_t level;

    if (pEvent == NULL) {
        return;
    }

    if (rt_mq_send(&g_event_mq, (void *)pEvent, sizeof(*pEvent)) != RT_EOK) {
        (void)rt_mq_recv(&g_event_mq, &drop, sizeof(drop), 0U);
        level = rt_hw_interrupt_disable();
        g_event_drop_count++;
        rt_hw_interrupt_enable(level);
        (void)rt_mq_send(&g_event_mq, (void *)pEvent, sizeof(*pEvent));
    }
}

static uint32_t app_get_event_drop_count(void)
{
    rt_base_t level;
    uint32_t count;

    level = rt_hw_interrupt_disable();
    count = g_event_drop_count;
    rt_hw_interrupt_enable(level);

    return count;
}

static void app_set_body_yaw(float bodyYaw)
{
    rt_base_t level;
    static float bodyYawZero;
    static uint8_t bodyYawZeroReady;

    level = rt_hw_interrupt_disable();
    if (g_body_yaw_zero_reset != 0U) {
        bodyYawZeroReady      = 0U;
        g_body_yaw_zero_reset = 0U;
        g_body_yaw            = 0.0f;
    }
    if (bodyYawZeroReady == 0U) {
        bodyYawZero      = bodyYaw;
        bodyYawZeroReady = 1U;
    }
    g_body_yaw = fast_math_normalize_deg(bodyYaw - bodyYawZero);
    rt_hw_interrupt_enable(level);
}

static void app_reset_body_yaw_zero(void)
{
    rt_base_t level;

    level                 = rt_hw_interrupt_disable();
    g_body_yaw            = 0.0f;
    g_body_yaw_zero_reset = 1U;
    rt_hw_interrupt_enable(level);
}

static void app_set_chassis_speed(float leftRps, float rightRps)
{
    rt_base_t level;

    level               = rt_hw_interrupt_disable();
    g_chassis_left_rps  = leftRps;
    g_chassis_right_rps = rightRps;
    rt_hw_interrupt_enable(level);
}

static uint16_t app_get_chassis_turn_percent(void)
{
    rt_base_t level;
    uint16_t percent;

    level = rt_hw_interrupt_disable();
    percent = g_chassis_turn_percent;
    rt_hw_interrupt_enable(level);

    return (percent == 0U) ? APP_CHASSIS_TURN_DEFAULT : percent;
}

static void app_set_chassis_turn_percent(int32_t percent)
{
    rt_base_t level;
    uint16_t savePercent;

    if (percent < APP_CHASSIS_TURN_MIN) {
        savePercent = APP_CHASSIS_TURN_MIN;
    } else if (percent > APP_CHASSIS_TURN_MAX) {
        savePercent = APP_CHASSIS_TURN_MAX;
    } else {
        savePercent = (uint16_t)percent;
    }

    level = rt_hw_interrupt_disable();
    g_chassis_turn_percent = savePercent;
    rt_hw_interrupt_enable(level);
}

static void app_fill_state(APP_STATE_S *pState)
{
    rt_base_t level;

    pState->uptime_ms = app_tick_ms();
    EmmV5Ctrl_GetTrackStatus(&pState->track, pState->uptime_ms);
    level                     = rt_hw_interrupt_disable();
    pState->body_yaw          = g_body_yaw;
    pState->chassis_left_rps  = g_chassis_left_rps;
    pState->chassis_right_rps = g_chassis_right_rps;
    rt_hw_interrupt_enable(level);
}

static void app_log_event(const EmmV5CtrlEvent *pEvent)
{
    uint32_t nowTick;
    static uint32_t lastK230Tick;
    static uint32_t lastMoveTick;
    static uint32_t lastTrackTick;

    if (pEvent == NULL) {
        return;
    }

    nowTick = app_tick_ms();
    switch (pEvent->type) {
        case EMM_V5_CTRL_EVENT_READY:
            UartMulti_Printf(UART_MULTI_CH1, "GIMBAL: core ready\r\n");
            break;
        case EMM_V5_CTRL_EVENT_MOVE:
            if ((int32_t)(nowTick - lastMoveTick) >=
                (int32_t)APP_DEBUG_LOG_PERIOD_MS) {
                lastMoveTick = nowTick;
                UartMulti_Printf(UART_MULTI_CH1,
                                 "MOVE: addr=%u clk=%ld vel=%u\r\n",
                                 pEvent->addr, (long)pEvent->clk,
                                 pEvent->vel);
            }
            break;
        case EMM_V5_CTRL_EVENT_K230_FRAME:
            if ((int32_t)(nowTick - lastK230Tick) >=
                (int32_t)APP_DEBUG_LOG_PERIOD_MS) {
                lastK230Tick = nowTick;
                UartMulti_Printf(UART_MULTI_CH1,
                                 "K230: seq=%lu valid=%u dx=%ld dy=%ld\r\n",
                                 (unsigned long)pEvent->seq, pEvent->valid,
                                 (long)pEvent->dx, (long)pEvent->dy);
            }
            break;
        case EMM_V5_CTRL_EVENT_TRACK:
            if ((int32_t)(nowTick - lastTrackTick) >=
                (int32_t)APP_DEBUG_LOG_PERIOD_MS) {
                lastTrackTick = nowTick;
                UartMulti_Printf(UART_MULTI_CH1,
                                 "TRACK: seq=%lu dx=%ld dy=%ld "
                                 "yaw=%ld pitch=%ld\r\n",
                                 (unsigned long)pEvent->seq,
                                 (long)pEvent->dx, (long)pEvent->dy,
                                 (long)pEvent->yaw_clk,
                                 (long)pEvent->pitch_clk);
            }
            break;
        case EMM_V5_CTRL_EVENT_STOP:
            UartMulti_Printf(UART_MULTI_CH1, "TRACK: stop code=%u\r\n",
                             pEvent->code);
            break;
        case EMM_V5_CTRL_EVENT_K230_UART:
            UartMulti_Printf(UART_MULTI_CH1,
                             "K230: uart err recover err=%lu\r\n",
                             (unsigned long)pEvent->err);
            break;
        case EMM_V5_CTRL_EVENT_MOTOR_ERROR:
            UartMulti_Printf(UART_MULTI_CH1,
                             "MOTOR: error addr=%u cmd=0x%02X\r\n",
                             pEvent->addr, pEvent->cmd);
            break;
        default:
            break;
    }
}

static void app_log_event_drop(uint32_t nowTick)
{
    static uint32_t lastDropTick;
    static uint32_t lastDropCount;
    uint32_t dropCount;

    dropCount = app_get_event_drop_count();
    if ((dropCount != lastDropCount) &&
        ((int32_t)(nowTick - lastDropTick) >=
         (int32_t)APP_DROP_LOG_PERIOD_MS)) {
        lastDropTick = nowTick;
        lastDropCount = dropCount;
        UartMulti_Printf(UART_MULTI_CH1, "EVT: drop=%lu\r\n",
                         (unsigned long)dropCount);
    }
}

static void app_poll_debug_cmd(void)
{
    static char line[APP_CMD_LINE_LEN];
    static uint8_t pos;
    char ch;
    long value;

    while (UartMulti_Read(UART_MULTI_CH1, (uint8_t *)&ch, 1U) == 1U) {
        if ((ch == '\r') || (ch == '\n')) {
            line[pos] = '\0';
            if (strcmp(line, "ag?") == 0) {
                UartMulti_Printf(UART_MULTI_CH1, "AG:%u\r\n",
                                 (unsigned int)app_get_chassis_turn_percent());
            } else if (sscanf(line, "ag %ld", &value) == 1) {
                app_set_chassis_turn_percent((int32_t)value);
                UartMulti_Printf(UART_MULTI_CH1, "AG:%u\r\n",
                                 (unsigned int)app_get_chassis_turn_percent());
            }
            pos = 0U;
        } else if (pos < (APP_CMD_LINE_LEN - 1U)) {
            line[pos] = ch;
            pos++;
        } else {
            pos = 0U;
        }
    }
}

static void lcd_show_state(const APP_STATE_S *pState)
{
    int32_t bodyYaw10;
    int32_t leftRps100;
    int32_t rightRps100;
    int32_t gimbalYaw10;
    int32_t gimbalPitch10;
    char line[APP_LCD_LINE_LEN];

    if ((pState == NULL) || (UiText_IsBusy() != 0U)) {
        return;
    }

    bodyYaw10     = app_float_scale(pState->body_yaw, 10.0f);
    leftRps100    = app_float_scale(pState->chassis_left_rps, 100.0f);
    rightRps100   = app_float_scale(pState->chassis_right_rps, 100.0f);
    gimbalYaw10   = app_float_scale(pState->track.gimbal_yaw_deg, 10.0f);
    gimbalPitch10 = app_float_scale(pState->track.gimbal_pitch_deg, 10.0f);

    (void)snprintf(line, sizeof(line), "t%lu by%ld.%01ld",
                   (unsigned long)(pState->uptime_ms / 1000U),
                   (long)(bodyYaw10 / 10),
                   (long)((bodyYaw10 < 0) ? -(bodyYaw10 % 10) : (bodyYaw10 % 10)));
    if (UiText_TryShowString(0U, 0U, line, ST7735_COLOR_WHITE,
                             ST7735_COLOR_BLACK) == 0U) {
        return;
    }
    (void)snprintf(line, sizeof(line), "s:%-9lu v:%u",
                   (unsigned long)pState->track.frame_seq,
                   pState->track.valid);
    if (UiText_TryShowString(0U, 16U, line, ST7735_COLOR_GREEN,
                             ST7735_COLOR_BLACK) == 0U) {
        return;
    }
    (void)snprintf(line, sizeof(line), "dx:%-4ld dy:%-4ld",
                   (long)pState->track.dx, (long)pState->track.dy);
    if (UiText_TryShowString(0U, 32U, line, ST7735_COLOR_CYAN,
                             ST7735_COLOR_BLACK) == 0U) {
        return;
    }
    (void)snprintf(line, sizeof(line), "l%ld.%02ld r%ld.%02ld",
                   (long)(leftRps100 / 100),
                   (long)((leftRps100 < 0) ? -(leftRps100 % 100) : (leftRps100 % 100)),
                   (long)(rightRps100 / 100),
                   (long)((rightRps100 < 0) ? -(rightRps100 % 100) : (rightRps100 % 100)));
    if (UiText_TryShowString(0U, 48U, line, ST7735_COLOR_YELLOW,
                             ST7735_COLOR_BLACK) == 0U) {
        return;
    }
    (void)snprintf(line, sizeof(line), "g%ld.%01ld p%ld.%01ld",
                   (long)(gimbalYaw10 / 10),
                   (long)((gimbalYaw10 < 0) ? -(gimbalYaw10 % 10) : (gimbalYaw10 % 10)),
                   (long)(gimbalPitch10 / 10),
                   (long)((gimbalPitch10 < 0) ? -(gimbalPitch10 % 10) : (gimbalPitch10 % 10)));
    (void)UiText_TryShowString(0U, 64U, line, ST7735_COLOR_MAGENTA,
                               ST7735_COLOR_BLACK);
}

static void ctrl_thread_entry(void *pArg)
{
    uint32_t nowMs;

    (void)pArg;
    while (1) {
        nowMs = app_tick_ms();
        EmmV5Ctrl_K230Process(nowMs);
        EmmV5Ctrl_MotorProcess(nowMs);
        EmmV5Ctrl_TrackProcess(nowMs);
        rt_thread_mdelay(APP_CTRL_PERIOD_MS);
    }
}

static void monitor_thread_entry(void *pArg)
{
    APP_STATE_S state;
    EmmV5CtrlEvent event;
    uint32_t lastLcdTick;

    (void)pArg;
    lastLcdTick = 0U;
    UiText_Init();
    UiText_Clear(ST7735_COLOR_BLACK);
    while (1) {
        UiText_Process();
        if (rt_mq_recv(&g_event_mq, &event, sizeof(event), 0U) == RT_EOK) {
            app_log_event(&event);
        }
        app_poll_debug_cmd();
        app_log_event_drop(app_tick_ms());
        if ((int32_t)(app_tick_ms() - lastLcdTick) >=
            (int32_t)APP_LCD_PERIOD_MS) {
            app_fill_state(&state);
            lcd_show_state(&state);
            lastLcdTick = app_tick_ms();
        }
        rt_thread_mdelay(APP_MONITOR_PERIOD_MS);
    }
}

static void imu_thread_entry(void *pArg)
{
    float angles[3];
    uint32_t lastErrorCount;
    uint32_t errorCount;
    uint32_t failStartTick;
    uint32_t imuStartTick;
    uint32_t nowTick;

    (void)pArg;
    IMU_init();
    app_reset_body_yaw_zero();
    lastErrorCount = bsp_Icm42688GetErrorCount();
    failStartTick  = 0U;
    imuStartTick   = app_tick_ms();
    while (1) {
        IMU_getYawPitchRoll(angles);
        errorCount = bsp_Icm42688GetErrorCount();
        nowTick    = app_tick_ms();
        if (errorCount == lastErrorCount) {
            failStartTick = 0U;
            if ((int32_t)(nowTick - imuStartTick) >=
                (int32_t)APP_IMU_ZERO_DELAY_MS) {
                app_set_body_yaw(angles[0] - APP_IMU_YAW_OFFSET_DEG);
            }
        } else {
            lastErrorCount = errorCount;
            if (failStartTick == 0U) {
                failStartTick = nowTick;
            } else if ((int32_t)(nowTick - failStartTick) >=
                       (int32_t)APP_IMU_FAIL_RECOVER_MS) {
                UartMulti_Printf(UART_MULTI_CH1,
                                 "IMU: i2c recover err=%lu\r\n",
                                 (unsigned long)errorCount);
                (void)HAL_I2C_DeInit(&hi2c1);
                MX_I2C1_Init();
                rt_thread_mdelay(APP_IMU_RECOVER_DELAY_MS);
                IMU_init();
                app_reset_body_yaw_zero();
                lastErrorCount = bsp_Icm42688GetErrorCount();
                failStartTick  = 0U;
                imuStartTick   = app_tick_ms();
            }
        }
        rt_thread_mdelay(APP_IMU_PERIOD_MS);
    }
}

static void chassis_thread_entry(void *pArg)
{
    EmmV5CtrlTrackStatus status;
    float gimbalYawZero;
    float yawError;
    float dxError;
    float forwardRps;
    float turnRps;
    float leftRps;
    float rightRps;
    float turnScale;
    uint8_t zeroReady;

    (void)pArg;
    gimbalYawZero = 0.0f;
    zeroReady     = 0U;
    MotorCtrl_Init();
    MotorCtrl_Stop();
    while (1) {
        EmmV5Ctrl_GetTrackStatus(&status, app_tick_ms());

        if (status.valid == 0U) {
            MotorCtrl_Stop();
            app_set_chassis_speed(0.0f, 0.0f);
            rt_thread_mdelay(APP_CHASSIS_PERIOD_MS);
            continue;
        }

        if (zeroReady == 0U) {
            gimbalYawZero = status.gimbal_yaw_deg;
            zeroReady     = 1U;
        }

        yawError = fast_math_normalize_deg(status.gimbal_yaw_deg -
                                           gimbalYawZero);
        yawError = fast_math_deadzone(yawError,
                                      APP_CHASSIS_YAW_DEADZONE_DEG);
        turnScale = (float)app_get_chassis_turn_percent() * 0.01f;
        turnRps = yawError * APP_CHASSIS_TURN_GAIN_DEG *
                  APP_CHASSIS_TURN_SIGN * turnScale;
        if (turnRps == 0.0f) {
            dxError = fast_math_deadzone((float)status.dx,
                                         APP_CHASSIS_DX_DEADZONE_PX);
            turnRps = dxError * APP_CHASSIS_DX_TURN_GAIN *
                      APP_CHASSIS_DX_TURN_SIGN * turnScale;
        }

        if (status.dist_cm <= 0.0f) {
            forwardRps = APP_CHASSIS_MID_FORWARD_RPS;
        } else if (status.dist_cm <= APP_CHASSIS_STOP_DIST_CM) {
            forwardRps = 0.0f;
        } else if (status.dist_cm <= APP_CHASSIS_FAR_DIST_CM) {
            forwardRps = APP_CHASSIS_MID_FORWARD_RPS;
        } else {
            forwardRps = APP_CHASSIS_FAR_FORWARD_RPS;
        }

        leftRps  = fast_math_clampf(forwardRps - turnRps,
                                    -APP_CHASSIS_MAX_SPEED_RPS,
                                    APP_CHASSIS_MAX_SPEED_RPS);
        rightRps = fast_math_clampf(forwardRps + turnRps,
                                    -APP_CHASSIS_MAX_SPEED_RPS,
                                    APP_CHASSIS_MAX_SPEED_RPS);
        MotorCtrl_SetDifferentialSpeed((leftRps + rightRps) * 0.5f,
                                       (rightRps - leftRps) * 0.5f);
        MotorCtrl_Update();
        MotorCtrl_GetSpeed(&leftRps, &rightRps);
        app_set_chassis_speed(leftRps, rightRps);
        rt_thread_mdelay(APP_CHASSIS_PERIOD_MS);
    }
}

static void app_init_threads(void)
{
    app_thread_start(&g_ctrl_thread, "ctrl", ctrl_thread_entry, RT_NULL,
                     g_ctrl_stack, sizeof(g_ctrl_stack), 10U);
    app_thread_start(&g_monitor_thread, "mon", monitor_thread_entry, RT_NULL,
                     g_monitor_stack, sizeof(g_monitor_stack), 13U);
    app_thread_start(&g_imu_thread, "imu", imu_thread_entry, RT_NULL,
                     g_imu_stack, sizeof(g_imu_stack), 14U);
    app_thread_start(&g_chassis_thread, "chassis", chassis_thread_entry,
                     RT_NULL, g_chassis_stack, sizeof(g_chassis_stack), 15U);
}

/*============================== Public Function =============================*/
void app_main(void)
{
    (void)UartMulti_WriteString(UART_MULTI_CH1, "MAIN: app start\r\n");
    (void)rt_mq_init(&g_event_mq, "evtmq", g_event_mq_pool,
                     sizeof(EmmV5CtrlEvent), sizeof(g_event_mq_pool),
                     RT_IPC_FLAG_PRIO);
    EmmV5Ctrl_Init(app_publish_event);
    app_init_threads();

    while (1) {
        rt_thread_mdelay(1000U);
    }
}
