#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include <rtthread.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t pulses_per_rev;
    int8_t direction;
    float speed;
    int32_t delta_count;
    uint32_t last_count;
    rt_tick_t last_tick;
    int32_t total_count;
} Encoder_Handle_t;

void Encoder_Init(Encoder_Handle_t *enc);
void Encoder_Start(Encoder_Handle_t *enc);
float Encoder_GetSpeed(Encoder_Handle_t *enc);
int32_t Encoder_GetDeltaCount(Encoder_Handle_t *enc);
int32_t Encoder_GetCount(Encoder_Handle_t *enc);

#endif
