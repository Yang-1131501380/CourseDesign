#ifndef __TRACK_CONTROLLER_H
#define __TRACK_CONTROLLER_H

#include "emm_v5_msg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*TrackControllerCmdCallback)(const motor_cmd_t *pCmd,
                                           const track_target_t *pTarget,
                                           void *pUser);
typedef void (*TrackControllerStopCallback)(uint8_t code, void *pUser);

typedef struct {
    float kp;
    float kd;
    float prev;
} TRACK_PID_S;

typedef struct {
    uint32_t last_track_tick;
    uint32_t used_seq;
    uint32_t stop_seq;
    int32_t yaw_target;
    int32_t pitch_target;
    TRACK_PID_S yaw_pid;
    TRACK_PID_S pitch_pid;
    TrackControllerCmdCallback cmd_cb;
    TrackControllerStopCallback stop_cb;
    void *p_user;
} TRACK_CONTROLLER_S;

void TrackController_Init(TRACK_CONTROLLER_S *pCtrl,
                          TrackControllerCmdCallback cmdCb,
                          TrackControllerStopCallback stopCb,
                          void *pUser);
void TrackController_Process(TRACK_CONTROLLER_S *pCtrl,
                             const track_target_t *pTarget,
                             const motor_pos_t *pPos,
                             uint32_t nowMs);

#ifdef __cplusplus
}
#endif

#endif
