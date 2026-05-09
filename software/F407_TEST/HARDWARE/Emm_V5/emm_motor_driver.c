#include "rtthread.h"
#include "rthw.h"

#include "emm_motor_driver.h"
#include "main.h"

#include <string.h>

#define EMM_MOTOR_POS_LEN           13U
#define EMM_MOTOR_ENABLE_LEN        6U
#define EMM_MOTOR_CMD_GAP_MS        4U
#define EMM_MOTOR_POS_PERIOD_MS     100U
#define EMM_MOTOR_POS_TIMEOUT_MS    100U
#define EMM_MOTOR_ENABLE_ON_BOOT    1U
#define EMM_MOTOR_ENABLE_STATE      1U
#define EMM_MOTOR_SINGLE_SYNC       0U
#define EMM_MOTOR_TRACK_ACC         0U
#define EMM_MOTOR_CIRCLE_CLK        51200L

#define EMM_MOTOR_STAGE_READY       0U
#define EMM_MOTOR_STAGE_ENABLE_YAW  1U
#define EMM_MOTOR_STAGE_ENABLE_PITCH 2U

static float emm_motor_clk_to_deg(int32_t clk)
{
    return ((float)clk * 360.0f) / (float)EMM_MOTOR_CIRCLE_CLK;
}

static uint8_t emm_motor_init_done(const EMM_MOTOR_DRIVER_S *pDriver)
{
    return (pDriver->init_stage == EMM_MOTOR_STAGE_READY) ? 1U : 0U;
}

static int emm_motor_send_enable(EMM_MOTOR_DRIVER_S *pDriver,
                                 uint8_t addr,
                                 uint32_t nowMs)
{
    uint8_t frame[EMM_MOTOR_ENABLE_LEN];

    if ((int32_t)(nowMs - pDriver->last_cmd_tick) <
        (int32_t)EMM_MOTOR_CMD_GAP_MS)
    {
        return -1;
    }

    frame[0] = addr;
    frame[1] = 0xF3U;
    frame[2] = 0xABU;
    frame[3] = EMM_MOTOR_ENABLE_STATE;
    frame[4] = EMM_MOTOR_SINGLE_SYNC;
    frame[5] = 0x6BU;

    if (Emm_V5_WriteRawOn(pDriver->uart_ch, frame, sizeof(frame)) !=
        EMM_V5_IO_OK)
    {
        return -1;
    }

    pDriver->last_cmd_tick = nowMs;

    return 0;
}

static int emm_motor_send_fd(EMM_MOTOR_DRIVER_S *pDriver,
                             uint8_t addr,
                             int32_t clk,
                             uint16_t vel)
{
    uint8_t frame[EMM_MOTOR_POS_LEN];
    uint32_t absClk;

    absClk = (clk < 0) ? (uint32_t)(-clk) : (uint32_t)clk;
    frame[0] = addr;
    frame[1] = 0xFDU;
    frame[2] = (clk < 0) ? 1U : 0U;
    frame[3] = (uint8_t)(vel >> 8);
    frame[4] = (uint8_t)vel;
    frame[5] = EMM_MOTOR_TRACK_ACC;
    frame[6] = (uint8_t)(absClk >> 24);
    frame[7] = (uint8_t)(absClk >> 16);
    frame[8] = (uint8_t)(absClk >> 8);
    frame[9] = (uint8_t)absClk;
    frame[10] = 0U;
    frame[11] = EMM_MOTOR_SINGLE_SYNC;
    frame[12] = 0x6BU;

    return (Emm_V5_WriteRawOn(pDriver->uart_ch, frame, sizeof(frame)) ==
            EMM_V5_IO_OK) ? 0 : -1;
}

static int emm_motor_send_pos(EMM_MOTOR_DRIVER_S *pDriver,
                              uint8_t addr,
                              int32_t clk,
                              uint16_t vel,
                              uint32_t nowMs)
{
    if ((int32_t)(nowMs - pDriver->last_cmd_tick) <
        (int32_t)EMM_MOTOR_CMD_GAP_MS)
    {
        return -1;
    }

    if (emm_motor_send_fd(pDriver, addr, clk, vel) != 0)
    {
        return -1;
    }

    pDriver->last_cmd_tick = nowMs;
    if (pDriver->move_cb != NULL)
    {
        pDriver->move_cb(addr, clk, vel, pDriver->p_user);
    }

    return 0;
}

static void emm_motor_process_init(EMM_MOTOR_DRIVER_S *pDriver,
                                   uint32_t nowMs)
{
    switch (pDriver->init_stage)
    {
        case EMM_MOTOR_STAGE_READY:
            return;

        case EMM_MOTOR_STAGE_ENABLE_YAW:
#if (EMM_MOTOR_ENABLE_ON_BOOT != 0U)
            if (emm_motor_send_enable(pDriver, pDriver->yaw_addr,
                                      nowMs) == 0)
            {
                pDriver->init_stage = EMM_MOTOR_STAGE_ENABLE_PITCH;
            }
#else
            pDriver->init_stage = EMM_MOTOR_STAGE_ENABLE_PITCH;
#endif
            break;

        case EMM_MOTOR_STAGE_ENABLE_PITCH:
#if (EMM_MOTOR_ENABLE_ON_BOOT != 0U)
            if (emm_motor_send_enable(pDriver, pDriver->pitch_addr,
                                      nowMs) == 0)
            {
                pDriver->init_stage = EMM_MOTOR_STAGE_READY;
            }
#else
            pDriver->init_stage = EMM_MOTOR_STAGE_READY;
#endif
            break;

        default:
            pDriver->init_stage = EMM_MOTOR_STAGE_READY;
            break;
    }
}

static void emm_motor_restore_pending(EMM_MOTOR_DRIVER_S *pDriver,
                                      const motor_cmd_t *pCmd)
{
    rt_base_t level;

    if ((pDriver == NULL) || (pCmd == NULL))
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    if (pDriver->pending_valid == 0U)
    {
        pDriver->pending_cmd = *pCmd;
        pDriver->pending_valid = 1U;
    }
    rt_hw_interrupt_enable(level);
}

static void emm_motor_send_pending(EMM_MOTOR_DRIVER_S *pDriver,
                                   uint32_t nowMs)
{
    motor_cmd_t cmd;
    rt_base_t level;

    if ((emm_motor_init_done(pDriver) == 0U) ||
        (pDriver->pending_valid == 0U))
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    cmd = pDriver->pending_cmd;
    pDriver->pending_valid = 0U;
    rt_hw_interrupt_enable(level);

    if (cmd.type == MOTOR_CMD_REL_FD)
    {
        if (cmd.yaw_clk != 0)
        {
            if (emm_motor_send_pos(pDriver, pDriver->yaw_addr,
                                   cmd.yaw_clk, cmd.vel, nowMs) != 0)
            {
                emm_motor_restore_pending(pDriver, &cmd);
                return;
            }
            cmd.yaw_clk = 0;
        }

        if (cmd.pitch_clk != 0)
        {
            if (emm_motor_send_pos(pDriver, pDriver->pitch_addr,
                                   cmd.pitch_clk, cmd.vel, nowMs) != 0)
            {
                emm_motor_restore_pending(pDriver, &cmd);
                return;
            }
            cmd.pitch_clk = 0;
        }
    }
}

static void emm_motor_poll_response(EMM_MOTOR_DRIVER_S *pDriver,
                                    uint32_t nowMs)
{
    uint8_t raw[EMM_MOTOR_DRIVER_RX_CHUNK];
    uint16_t room;
    uint16_t len;
    EmmV5Frame frame;
    EmmV5ParseResult res;

    room = (uint16_t)(EMM_V5_PARSER_BUFFER_SIZE - pDriver->parser.length);
    if (room > sizeof(raw))
    {
        room = sizeof(raw);
    }

    len = Emm_V5_ReadRawFrom(pDriver->uart_ch, raw, room);
    if (len > 0U)
    {
        memcpy(&pDriver->parser.buffer[pDriver->parser.length], raw, len);
        pDriver->parser.length = (uint16_t)(pDriver->parser.length + len);
    }

    do
    {
        res = Emm_V5_ParserPoll(&pDriver->parser, &frame);
        if ((res == EMM_V5_PARSE_OK) &&
            (frame.type == EMM_V5_FRAME_TYPE_SYS_PARAM) &&
            (frame.data.sys_param.param == S_CPOS))
        {
            if (frame.addr == pDriver->yaw_addr)
            {
                pDriver->pos.yaw_clk = frame.data.sys_param.value_i32;
                pDriver->pos.yaw_deg =
                    emm_motor_clk_to_deg(pDriver->pos.yaw_clk);
                pDriver->pos.yaw_ready = 1U;
            }
            else if (frame.addr == pDriver->pitch_addr)
            {
                pDriver->pos.pitch_clk = frame.data.sys_param.value_i32;
                pDriver->pos.pitch_deg =
                    emm_motor_clk_to_deg(pDriver->pos.pitch_clk);
                pDriver->pos.pitch_ready = 1U;
            }

            pDriver->pos.tick_ms = nowMs;
            if (frame.addr == pDriver->pos_wait_addr)
            {
                pDriver->pos_wait_addr = 0U;
            }
        }
        else if ((res == EMM_V5_PARSE_OK) &&
                 (frame.type == EMM_V5_FRAME_TYPE_ERROR))
        {
            if (pDriver->error_cb != NULL)
            {
                pDriver->error_cb(frame.addr, frame.command,
                                  pDriver->p_user);
            }
        }
    } while (res == EMM_V5_PARSE_OK);
}

static void emm_motor_poll_position(EMM_MOTOR_DRIVER_S *pDriver,
                                    uint32_t nowMs)
{
    if ((emm_motor_init_done(pDriver) == 0U) ||
        (pDriver->pending_valid != 0U) ||
        ((int32_t)(nowMs - pDriver->last_cmd_tick) <
         (int32_t)EMM_MOTOR_CMD_GAP_MS))
    {
        return;
    }

    if (pDriver->pos_wait_addr != 0U)
    {
        if ((int32_t)(nowMs - pDriver->pos_deadline) >= 0)
        {
            pDriver->pos_wait_addr = 0U;
        }
        return;
    }

    if ((int32_t)(nowMs - pDriver->last_pos_tick) >=
        (int32_t)EMM_MOTOR_POS_PERIOD_MS)
    {
        pDriver->pos_wait_addr = pDriver->next_pos_addr;
        pDriver->next_pos_addr =
            (pDriver->next_pos_addr == pDriver->yaw_addr) ?
            pDriver->pitch_addr : pDriver->yaw_addr;
        Emm_V5_Read_Sys_Params(pDriver->uart_ch, pDriver->pos_wait_addr,
                               S_CPOS);
        pDriver->last_cmd_tick = nowMs;
        pDriver->pos_deadline = nowMs + EMM_MOTOR_POS_TIMEOUT_MS;
        pDriver->last_pos_tick = nowMs;
    }
}

void EmmMotorDriver_Init(EMM_MOTOR_DRIVER_S *pDriver,
                         UartMultiChannelId uartCh,
                         uint8_t yawAddr,
                         uint8_t pitchAddr,
                         EmmMotorMoveCallback moveCb,
                         EmmMotorErrorCallback errorCb,
                         void *pUser)
{
    if (pDriver == NULL)
    {
        return;
    }

    memset(pDriver, 0, sizeof(*pDriver));
    pDriver->uart_ch = uartCh;
    pDriver->yaw_addr = yawAddr;
    pDriver->pitch_addr = pitchAddr;
    pDriver->next_pos_addr = yawAddr;
    pDriver->init_stage = EMM_MOTOR_STAGE_ENABLE_YAW;
    pDriver->move_cb = moveCb;
    pDriver->error_cb = errorCb;
    pDriver->p_user = pUser;
    Emm_V5_ParserInit(&pDriver->parser, uartCh);
}

void EmmMotorDriver_Submit(EMM_MOTOR_DRIVER_S *pDriver,
                            const motor_cmd_t *pCmd)
{
    rt_base_t level;

    if ((pDriver == NULL) || (pCmd == NULL))
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    pDriver->pending_cmd = *pCmd;
    pDriver->pending_valid =
        ((pCmd->type != MOTOR_CMD_NONE) &&
         ((pCmd->yaw_clk != 0) || (pCmd->pitch_clk != 0))) ? 1U : 0U;
    rt_hw_interrupt_enable(level);
}

void EmmMotorDriver_Process(EMM_MOTOR_DRIVER_S *pDriver, uint32_t nowMs)
{
    if (pDriver == NULL)
    {
        return;
    }

    emm_motor_process_init(pDriver, nowMs);
    emm_motor_send_pending(pDriver, nowMs);
    emm_motor_poll_response(pDriver, nowMs);
    emm_motor_poll_position(pDriver, nowMs);
}

void EmmMotorDriver_GetPos(const EMM_MOTOR_DRIVER_S *pDriver,
                           motor_pos_t *pPos)
{
    if ((pDriver == NULL) || (pPos == NULL))
    {
        return;
    }

    *pPos = pDriver->pos;
}
