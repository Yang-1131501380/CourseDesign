#include "encoder.h"

static int32_t Encoder_CalcDelta(const Encoder_Handle_t *enc, uint32_t count)
{
    uint32_t arr;

    if ((enc == NULL) || (enc->htim == NULL)) {
        return 0;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(enc->htim);
    if (arr <= 0xFFFFU) {
        return (int32_t)(int16_t)((uint16_t)count - (uint16_t)enc->last_count);
    }

    return (int32_t)(count - enc->last_count);
}

void Encoder_Start(Encoder_Handle_t *enc)
{
    if ((enc == NULL) || (enc->htim == NULL)) {
        return;
    }

    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
}

void Encoder_Init(Encoder_Handle_t *enc)
{
    if ((enc == NULL) || (enc->htim == NULL)) {
        return;
    }

    if (enc->direction == 0) {
        enc->direction = 1;
    }

    Encoder_Start(enc);
    enc->last_count = __HAL_TIM_GET_COUNTER(enc->htim);
    enc->last_tick = rt_tick_get();
    enc->speed = 0.0f;
    enc->delta_count = 0;
    enc->total_count = 0;
}

float Encoder_GetSpeed(Encoder_Handle_t *enc)
{
    uint32_t count;
    int32_t delta_count;
    rt_tick_t now;
    rt_tick_t delta_tick;

    if ((enc == NULL) || (enc->htim == NULL) || (enc->pulses_per_rev == 0)) {
        return 0.0f;
    }

    now = rt_tick_get();
    count = __HAL_TIM_GET_COUNTER(enc->htim);
    delta_count = Encoder_CalcDelta(enc, count);
    delta_count *= (int32_t)enc->direction;
    delta_tick = now - enc->last_tick;

    if (delta_tick > 0U) {
        enc->speed = ((float)delta_count * 1000.0f) /
                     ((float)delta_tick * (float)enc->pulses_per_rev);
    }

    enc->delta_count = delta_count;
    enc->total_count += delta_count;
    enc->last_count = count;
    enc->last_tick = now;
    return enc->speed;
}

int32_t Encoder_GetDeltaCount(Encoder_Handle_t *enc)
{
    if (enc == NULL) {
        return 0;
    }

    return enc->delta_count;
}

int32_t Encoder_GetCount(Encoder_Handle_t *enc)
{
    if (enc == NULL) {
        return 0;
    }

    return enc->total_count;
}
