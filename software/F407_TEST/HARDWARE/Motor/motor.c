#include "motor.h"
#include "fast_math.h"

static float Motor_Clamp(float value, float min_value, float max_value)
{
    return fast_math_clampf(value, min_value, max_value);
}

static float Motor_RpsToDeltaTarget(const Motor_t *motor, float speed_rps)
{
    if ((motor == NULL) || (motor->encoder.pulses_per_rev <= 0) || (motor->control_period_ms == 0U)) {
        return 0.0f;
    }

    return speed_rps * (float)motor->encoder.pulses_per_rev * ((float)motor->control_period_ms / 1000.0f);
}

static float Motor_PICompute(Motor_t *motor, float target_delta_count, float current_delta_count)
{
    float error;
    float delta_output;

    error = target_delta_count - current_delta_count;
    delta_output = (motor->kp * (error - motor->prev_error)) + (motor->ki * error);
    motor->control_output = Motor_Clamp(motor->control_output + delta_output, motor->output_min, motor->output_max);
    motor->prev_error = error;
    return motor->control_output;
}

void Motor_Init(Motor_t *motor, TB6612_Handle_t *tb_cfg, Encoder_Handle_t *enc_cfg, float kp, float ki, float kd)
{
    if ((motor == NULL) || (tb_cfg == NULL) || (enc_cfg == NULL)) {
        return;
    }

    motor->tb6612 = *tb_cfg;
    motor->encoder = *enc_cfg;
    TB6612_Init(&motor->tb6612);
    Encoder_Init(&motor->encoder);

    motor->kp = kp;
    motor->ki = ki;
    motor->kd = kd;
    motor->control_output = 0.0f;
    motor->prev_error = 0.0f;
    motor->output_min = -(float)motor->tb6612.logical_max_output;
    motor->output_max = (float)motor->tb6612.logical_max_output;
    motor->target_speed = 0.0f;
    motor->current_speed = 0.0f;
    motor->control_period_ms = 10U;
    motor->last_control_tick = rt_tick_get();
}

void Motor_SetSpeed(Motor_t *motor, float speed_rps)
{
    if (motor == NULL) {
        return;
    }

    motor->target_speed = speed_rps;
}

void Motor_Update(Motor_t *motor)
{
    float pwm_out;
    float target_delta_count;
    float current_delta_count;
    rt_tick_t now;

    if (motor == NULL) {
        return;
    }

    now = rt_tick_get();
    if ((now - motor->last_control_tick) < (rt_tick_t)motor->control_period_ms) {
        return;
    }

    motor->current_speed = Encoder_GetSpeed(&motor->encoder);
    target_delta_count = Motor_RpsToDeltaTarget(motor, motor->target_speed);
    current_delta_count = (float)Encoder_GetDeltaCount(&motor->encoder);
    pwm_out = Motor_PICompute(motor, target_delta_count, current_delta_count);
    TB6612_SetSpeed(&motor->tb6612, (int16_t)pwm_out);
    motor->last_control_tick = now;
}

void Motor_Brake(Motor_t *motor)
{
    if (motor == NULL) {
        return;
    }

    TB6612_Brake(&motor->tb6612);
    motor->target_speed = 0.0f;
    motor->current_speed = 0.0f;
    motor->control_output = 0.0f;
    motor->prev_error = 0.0f;
}

float Motor_GetCurrentSpeed(Motor_t *motor)
{
    if (motor == NULL) {
        return 0.0f;
    }

    return motor->current_speed;
}
