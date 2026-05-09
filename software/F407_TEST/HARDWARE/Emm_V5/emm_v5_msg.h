#ifndef __EMM_V5_MSG_H
#define __EMM_V5_MSG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t valid;
    int32_t cx;
    int32_t cy;
    int32_t dx;
    int32_t dy;
    int32_t width;
    int32_t height;
    float dist_cm;
    uint32_t seq;
    uint32_t tick_ms;
} track_target_t;

typedef struct {
    int32_t yaw_clk;
    int32_t pitch_clk;
    float yaw_deg;
    float pitch_deg;
    uint8_t yaw_ready;
    uint8_t pitch_ready;
    uint32_t tick_ms;
} motor_pos_t;

typedef enum {
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_ENABLE,
    MOTOR_CMD_REL_FD,
    MOTOR_CMD_ABS_FD,
    MOTOR_CMD_STOP
} motor_cmd_type_t;

typedef struct {
    motor_cmd_type_t type;
    int32_t yaw_clk;
    int32_t pitch_clk;
    uint16_t vel;
    uint8_t acc;
    uint32_t seq;
    uint32_t tick_ms;
} motor_cmd_t;

#ifdef __cplusplus
}
#endif

#endif
