#ifndef __TB6612_H
#define __TB6612_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    GPIO_TypeDef *stby_port;
    uint16_t stby_pin;
    int16_t logical_max_output;
} TB6612_Handle_t;

void TB6612_Init(TB6612_Handle_t *tb6612);
void TB6612_Enable(TB6612_Handle_t *tb6612);
void TB6612_Disable(TB6612_Handle_t *tb6612);
void TB6612_SetSpeed(TB6612_Handle_t *tb6612, int16_t speed);
void TB6612_Brake(TB6612_Handle_t *tb6612);
uint16_t TB6612_GetPwmPeriod(const TB6612_Handle_t *tb6612);

#endif
