#include "rtthread.h"
#include "rthw.h"

#include "emm_v5_ctrl.h"
#include "main.h"
#include "emm_motor_driver.h"
#include "k230_parser.h"
#include "track_controller.h"
#include "uart_multi.h"

#include <string.h>

/*=============================== Macro Define ===============================*/
#define EMM_K230_CH                 UART_MULTI_CH6
#define EMM_MOTOR_CH                UART_MULTI_CH2
#define EMM_YAW_ADDR                1U
#define EMM_PITCH_ADDR              2U
#define EMM_TARGET_TIMEOUT_MS       300U

/*=============================== Type Define ================================*/
typedef struct {
    K230_PARSER_S k230;
    EMM_MOTOR_DRIVER_S motor;
    TRACK_CONTROLLER_S track;
    track_target_t target;
    motor_pos_t pos;
    EmmV5CtrlEventCallback event_cb;
} EMM_CTRL_S;

/*============================== Global Variable =============================*/
static EMM_CTRL_S g_ctrl;

static void ctrl_push_event(const EmmV5CtrlEvent *pEvent)
{
    if ((pEvent == NULL) || (g_ctrl.event_cb == NULL))
    {
        return;
    }

    g_ctrl.event_cb(pEvent);
}

static void ctrl_on_k230_target(const track_target_t *pTarget, void *pUser)
{
    EmmV5CtrlEvent event;
    track_target_t target;
    rt_base_t level;

    (void)pUser;
    if (pTarget == NULL)
    {
        return;
    }

    target = *pTarget;
    level = rt_hw_interrupt_disable();
    g_ctrl.target = *pTarget;
    rt_hw_interrupt_enable(level);

    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_K230_FRAME;
    event.valid = target.valid;
    event.dx = target.dx;
    event.dy = target.dy;
    event.seq = target.seq;
    ctrl_push_event(&event);
}

static void ctrl_on_k230_uart(uint32_t err, void *pUser)
{
    EmmV5CtrlEvent event;

    (void)pUser;
    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_K230_UART;
    event.err = err;
    ctrl_push_event(&event);
}

static void ctrl_on_motor_move(uint8_t addr,
                               int32_t clk,
                               uint16_t vel,
                               void *pUser)
{
    EmmV5CtrlEvent event;

    (void)pUser;
    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_MOVE;
    event.addr = addr;
    event.clk = clk;
    event.vel = vel;
    ctrl_push_event(&event);
}

static void ctrl_on_motor_error(uint8_t addr, uint8_t cmd, void *pUser)
{
    EmmV5CtrlEvent event;

    (void)pUser;
    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_MOTOR_ERROR;
    event.addr = addr;
    event.cmd = cmd;
    ctrl_push_event(&event);
}

static void ctrl_on_track_cmd(const motor_cmd_t *pCmd,
                              const track_target_t *pTarget,
                              void *pUser)
{
    EmmV5CtrlEvent event;

    (void)pUser;
    if ((pCmd == NULL) || (pTarget == NULL))
    {
        return;
    }

    EmmMotorDriver_Submit(&g_ctrl.motor, pCmd);

    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_TRACK;
    event.seq = pTarget->seq;
    event.dx = pTarget->dx;
    event.dy = pTarget->dy;
    event.yaw_clk = pCmd->yaw_clk;
    event.pitch_clk = pCmd->pitch_clk;
    ctrl_push_event(&event);
}

static void ctrl_on_track_stop(uint8_t code, void *pUser)
{
    EmmV5CtrlEvent event;

    (void)pUser;
    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_STOP;
    event.code = code;
    ctrl_push_event(&event);
}

/*============================ Interface Functions ===========================*/
void EmmV5Ctrl_Init(EmmV5CtrlEventCallback callback)
{
    EmmV5CtrlEvent event;

    memset(&g_ctrl, 0, sizeof(g_ctrl));
    g_ctrl.event_cb = callback;
    K230Parser_Init(&g_ctrl.k230, EMM_K230_CH, ctrl_on_k230_target,
                    ctrl_on_k230_uart, NULL);
    EmmMotorDriver_Init(&g_ctrl.motor, EMM_MOTOR_CH, EMM_YAW_ADDR,
                        EMM_PITCH_ADDR, ctrl_on_motor_move,
                        ctrl_on_motor_error, NULL);
    TrackController_Init(&g_ctrl.track, ctrl_on_track_cmd,
                         ctrl_on_track_stop, NULL);

    memset(&event, 0, sizeof(event));
    event.type = EMM_V5_CTRL_EVENT_READY;
    ctrl_push_event(&event);
}

void EmmV5Ctrl_K230Process(uint32_t nowMs)
{
    K230Parser_Process(&g_ctrl.k230, nowMs);
}

void EmmV5Ctrl_MotorProcess(uint32_t nowMs)
{
    motor_pos_t pos;
    rt_base_t level;

    EmmMotorDriver_Process(&g_ctrl.motor, nowMs);
    EmmMotorDriver_GetPos(&g_ctrl.motor, &pos);
    level = rt_hw_interrupt_disable();
    g_ctrl.pos = pos;
    rt_hw_interrupt_enable(level);
}

void EmmV5Ctrl_TrackProcess(uint32_t nowMs)
{
    track_target_t target;
    motor_pos_t pos;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    target = g_ctrl.target;
    pos = g_ctrl.pos;
    rt_hw_interrupt_enable(level);

    TrackController_Process(&g_ctrl.track, &target, &pos, nowMs);
}

void EmmV5Ctrl_GetTrackStatus(EmmV5CtrlTrackStatus *pStatus, uint32_t nowMs)
{
    track_target_t target;
    motor_pos_t pos;
    rt_base_t level;

    if (pStatus == NULL)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    target = g_ctrl.target;
    pos = g_ctrl.pos;
    rt_hw_interrupt_enable(level);

    pStatus->valid = target.valid;
    if ((target.valid != 0U) &&
        ((int32_t)(nowMs - target.tick_ms) >
         (int32_t)EMM_TARGET_TIMEOUT_MS))
    {
        pStatus->valid = 0U;
    }
    pStatus->cx = target.cx;
    pStatus->cy = target.cy;
    pStatus->dx = target.dx;
    pStatus->dy = target.dy;
    pStatus->width = target.width;
    pStatus->height = target.height;
    pStatus->dist_cm = target.dist_cm;
    pStatus->frame_seq = target.seq;
    pStatus->target_tick_ms = target.tick_ms;
    pStatus->gimbal_yaw_clk = pos.yaw_clk;
    pStatus->gimbal_pitch_clk = pos.pitch_clk;
    pStatus->gimbal_yaw_deg = pos.yaw_deg;
    pStatus->gimbal_pitch_deg = pos.pitch_deg;
}
