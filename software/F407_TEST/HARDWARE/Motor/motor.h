#ifndef __MOTOR_H
#define __MOTOR_H

#include "encoder.h"
#include "tb6612.h"
#include <rtthread.h>

typedef struct {
    TB6612_Handle_t tb6612;
    Encoder_Handle_t encoder;
    float kp;
    float ki;
    float kd;
    float control_output;
    float prev_error;
    float output_min;
    float output_max;
    float target_speed;
    float current_speed;
    uint32_t control_period_ms;
    rt_tick_t last_control_tick;
} Motor_t;

void Motor_Init(Motor_t *motor, TB6612_Handle_t *tb_cfg, Encoder_Handle_t *enc_cfg, float kp, float ki, float kd);
void Motor_SetSpeed(Motor_t *motor, float speed_rps);
void Motor_Update(Motor_t *motor);
void Motor_Brake(Motor_t *motor);
float Motor_GetCurrentSpeed(Motor_t *motor);

#endif
