#ifndef __EMM_V5_CTRL_H
#define __EMM_V5_CTRL_H

#include "emm_v5_msg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EMM_V5_CTRL_EVENT_NONE = 0,
    EMM_V5_CTRL_EVENT_READY,
    EMM_V5_CTRL_EVENT_MOVE,
    EMM_V5_CTRL_EVENT_TRACK,
    EMM_V5_CTRL_EVENT_K230_FRAME,
    EMM_V5_CTRL_EVENT_STOP,
    EMM_V5_CTRL_EVENT_K230_UART,
    EMM_V5_CTRL_EVENT_MOTOR_ERROR
} EmmV5CtrlEventType;

typedef struct {
    EmmV5CtrlEventType type;
    uint8_t code;
    uint8_t addr;
    uint8_t cmd;
    uint8_t valid;
    uint16_t vel;
    int32_t clk;
    int32_t yaw_clk;
    int32_t pitch_clk;
    int32_t dx;
    int32_t dy;
    uint32_t seq;
    uint32_t err;
} EmmV5CtrlEvent;

typedef struct {
    uint8_t valid;
    int32_t cx;
    int32_t cy;
    int32_t dx;
    int32_t dy;
    int32_t width;
    int32_t height;
    float dist_cm;
    uint32_t frame_seq;
    uint32_t target_tick_ms;
    int32_t gimbal_yaw_clk;
    int32_t gimbal_pitch_clk;
    float gimbal_yaw_deg;
    float gimbal_pitch_deg;
} EmmV5CtrlTrackStatus;

typedef void (*EmmV5CtrlEventCallback)(const EmmV5CtrlEvent *event);

void EmmV5Ctrl_Init(EmmV5CtrlEventCallback callback);
void EmmV5Ctrl_K230Process(uint32_t nowMs);
void EmmV5Ctrl_MotorProcess(uint32_t nowMs);
void EmmV5Ctrl_TrackProcess(uint32_t nowMs);
void EmmV5Ctrl_GetTrackStatus(EmmV5CtrlTrackStatus *status, uint32_t nowMs);

#ifdef __cplusplus
}
#endif

#endif
