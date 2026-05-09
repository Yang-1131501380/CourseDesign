#include "tb6612.h"
#include "fast_math.h"

#define TB6612_DEFAULT_LOGICAL_MAX 1000

static int16_t TB6612_Clamp(int16_t value, int16_t limit)
{
    return (int16_t)fast_math_clampf((float)value, (float)-limit,
                                     (float)limit);
}

uint16_t TB6612_GetPwmPeriod(const TB6612_Handle_t *tb6612)
{
    if ((tb6612 == NULL) || (tb6612->htim == NULL)) {
        return 0U;
    }

    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(tb6612->htim);
}

void TB6612_Enable(TB6612_Handle_t *tb6612)
{
    if ((tb6612 == NULL) || (tb6612->stby_port == NULL)) {
        return;
    }

    HAL_GPIO_WritePin(tb6612->stby_port, tb6612->stby_pin, GPIO_PIN_SET);
}

void TB6612_Disable(TB6612_Handle_t *tb6612)
{
    if ((tb6612 == NULL) || (tb6612->stby_port == NULL)) {
        return;
    }

    HAL_GPIO_WritePin(tb6612->stby_port, tb6612->stby_pin, GPIO_PIN_RESET);
}

void TB6612_Brake(TB6612_Handle_t *tb6612)
{
    if (tb6612 == NULL) {
        return;
    }

    __HAL_TIM_SET_COMPARE(tb6612->htim, tb6612->channel, 0U);
    HAL_GPIO_WritePin(tb6612->in1_port, tb6612->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(tb6612->in2_port, tb6612->in2_pin, GPIO_PIN_SET);
}

void TB6612_Init(TB6612_Handle_t *tb6612)
{
    if ((tb6612 == NULL) || (tb6612->htim == NULL)) {
        return;
    }

    if (tb6612->logical_max_output <= 0) {
        tb6612->logical_max_output = TB6612_DEFAULT_LOGICAL_MAX;
    }

    HAL_TIM_PWM_Start(tb6612->htim, tb6612->channel);
    TB6612_Enable(tb6612);
    __HAL_TIM_SET_COMPARE(tb6612->htim, tb6612->channel, 0U);
    HAL_GPIO_WritePin(tb6612->in1_port, tb6612->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tb6612->in2_port, tb6612->in2_pin, GPIO_PIN_RESET);
}

void TB6612_SetSpeed(TB6612_Handle_t *tb6612, int16_t speed)
{
    uint16_t pwm_period;
    int16_t clamped;
    uint32_t compare;

    if ((tb6612 == NULL) || (tb6612->htim == NULL)) {
        return;
    }

    TB6612_Enable(tb6612);
    pwm_period = TB6612_GetPwmPeriod(tb6612);
    if (pwm_period == 0U) {
        return;
    }

    clamped = TB6612_Clamp(speed, tb6612->logical_max_output);
    compare = ((uint32_t)((clamped >= 0) ? clamped : -clamped) * pwm_period) /
              (uint32_t)tb6612->logical_max_output;

    if (clamped > 0) {
        HAL_GPIO_WritePin(tb6612->in1_port, tb6612->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(tb6612->in2_port, tb6612->in2_pin, GPIO_PIN_RESET);
    } else if (clamped < 0) {
        HAL_GPIO_WritePin(tb6612->in1_port, tb6612->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(tb6612->in2_port, tb6612->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(tb6612->in1_port, tb6612->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(tb6612->in2_port, tb6612->in2_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(tb6612->htim, tb6612->channel, compare);
}
