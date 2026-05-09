#include "track_controller.h"
#include "fast_math.h"

#include <string.h>

#define TRACK_TIMEOUT_MS           300U
#define TRACK_PERIOD_MS            30U
#define TRACK_VEL                  25U
#define TRACK_ACC                  0U
#define TRACK_DEADZONE             10L
#define TRACK_STEP_MAX_CLK         300L
#define TRACK_LIMIT_CLK            3200L
#define TRACK_YAW_SIGN             1L
#define TRACK_PITCH_SIGN           1L

static int32_t track_limit_clk(int32_t clk, int32_t limit)
{
    return (int32_t)fast_math_clampf((float)clk, (float)-limit,
                                     (float)limit);
}

static int32_t track_pid_step(TRACK_PID_S *pPid, int32_t error)
{
    float e;
    float out;

    if (pPid == NULL)
    {
        return 0;
    }

    if ((error <= TRACK_DEADZONE) && (error >= -TRACK_DEADZONE))
    {
        pPid->prev = (float)error;
        return 0;
    }

    e = (float)error;
    out = (pPid->kp * e) + (pPid->kd * (e - pPid->prev));
    pPid->prev = e;

    return track_limit_clk((out >= 0.0f) ? (int32_t)(out + 0.5f) :
                           (int32_t)(out - 0.5f),
                           TRACK_STEP_MAX_CLK);
}

void TrackController_Init(TRACK_CONTROLLER_S *pCtrl,
                          TrackControllerCmdCallback cmdCb,
                          TrackControllerStopCallback stopCb,
                          void *pUser)
{
    if (pCtrl == NULL)
    {
        return;
    }

    memset(pCtrl, 0, sizeof(*pCtrl));
    pCtrl->yaw_pid.kp = 0.6f;
    pCtrl->yaw_pid.kd = 0.12f;
    pCtrl->pitch_pid.kp = 0.6f;
    pCtrl->pitch_pid.kd = 0.12f;
    pCtrl->cmd_cb = cmdCb;
    pCtrl->stop_cb = stopCb;
    pCtrl->p_user = pUser;
}

static void track_reset_pid(TRACK_CONTROLLER_S *pCtrl)
{
    if (pCtrl == NULL)
    {
        return;
    }

    pCtrl->yaw_pid.prev = 0.0f;
    pCtrl->pitch_pid.prev = 0.0f;
}

void TrackController_Process(TRACK_CONTROLLER_S *pCtrl,
                             const track_target_t *pTarget,
                             const motor_pos_t *pPos,
                             uint32_t nowMs)
{
    int32_t yawDelta;
    int32_t pitchDelta;
    int32_t nextYaw;
    int32_t nextPitch;
    motor_cmd_t cmd;

    (void)pPos;
    if ((pCtrl == NULL) || (pTarget == NULL))
    {
        return;
    }

    if (pTarget->valid == 0U)
    {
        return;
    }

    if ((int32_t)(nowMs - pTarget->tick_ms) > (int32_t)TRACK_TIMEOUT_MS)
    {
        if ((pCtrl->stop_seq != pTarget->seq) && (pCtrl->stop_cb != NULL))
        {
            pCtrl->stop_seq = pTarget->seq;
            track_reset_pid(pCtrl);
            pCtrl->stop_cb(1U, pCtrl->p_user);
        }
        return;
    }

    if ((pTarget->seq == pCtrl->used_seq) ||
        ((int32_t)(nowMs - pCtrl->last_track_tick) < (int32_t)TRACK_PERIOD_MS))
    {
        return;
    }

    pCtrl->last_track_tick = nowMs;
    pCtrl->used_seq = pTarget->seq;
    pCtrl->stop_seq = 0U;
    yawDelta = track_pid_step(&pCtrl->yaw_pid,
                              (int32_t)(TRACK_YAW_SIGN * pTarget->dx));
    pitchDelta = track_pid_step(&pCtrl->pitch_pid,
                                (int32_t)(TRACK_PITCH_SIGN * pTarget->dy));
    nextYaw = track_limit_clk(pCtrl->yaw_target + yawDelta,
                              TRACK_LIMIT_CLK);
    nextPitch = track_limit_clk(pCtrl->pitch_target + pitchDelta,
                                TRACK_LIMIT_CLK);
    yawDelta = nextYaw - pCtrl->yaw_target;
    pitchDelta = nextPitch - pCtrl->pitch_target;
    pCtrl->yaw_target = nextYaw;
    pCtrl->pitch_target = nextPitch;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = MOTOR_CMD_REL_FD;
    cmd.yaw_clk = yawDelta;
    cmd.pitch_clk = pitchDelta;
    cmd.vel = TRACK_VEL;
    cmd.acc = TRACK_ACC;
    cmd.seq = pTarget->seq;
    cmd.tick_ms = nowMs;

    if (pCtrl->cmd_cb != NULL)
    {
        pCtrl->cmd_cb(&cmd, pTarget, pCtrl->p_user);
    }
}
