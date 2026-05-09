#ifndef __EMM_MOTOR_DRIVER_H
#define __EMM_MOTOR_DRIVER_H

#include "Emm_V5.h"
#include "emm_v5_msg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EMM_MOTOR_DRIVER_RX_CHUNK       32U

typedef void (*EmmMotorMoveCallback)(uint8_t addr,
                                     int32_t clk,
                                     uint16_t vel,
                                     void *pUser);
typedef void (*EmmMotorErrorCallback)(uint8_t addr,
                                      uint8_t cmd,
                                      void *pUser);

typedef struct {
    EmmV5Parser parser;
    UartMultiChannelId uart_ch;
    uint8_t yaw_addr;
    uint8_t pitch_addr;
    uint8_t pos_wait_addr;
    uint8_t next_pos_addr;
    uint8_t init_stage;
    uint8_t pending_valid;
    uint32_t last_cmd_tick;
    uint32_t last_pos_tick;
    uint32_t pos_deadline;
    motor_cmd_t pending_cmd;
    motor_pos_t pos;
    EmmMotorMoveCallback move_cb;
    EmmMotorErrorCallback error_cb;
    void *p_user;
} EMM_MOTOR_DRIVER_S;

void EmmMotorDriver_Init(EMM_MOTOR_DRIVER_S *pDriver,
                         UartMultiChannelId uartCh,
                         uint8_t yawAddr,
                         uint8_t pitchAddr,
                         EmmMotorMoveCallback moveCb,
                         EmmMotorErrorCallback errorCb,
                         void *pUser);
void EmmMotorDriver_Submit(EMM_MOTOR_DRIVER_S *pDriver,
                            const motor_cmd_t *pCmd);
void EmmMotorDriver_Process(EMM_MOTOR_DRIVER_S *pDriver, uint32_t nowMs);
void EmmMotorDriver_GetPos(const EMM_MOTOR_DRIVER_S *pDriver,
                           motor_pos_t *pPos);

#ifdef __cplusplus
}
#endif

#endif
