#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "tb6612.h"

void MotorDriver_Init(void);
TB6612_Handle_t *Motor_GetLeftChannel(void);
TB6612_Handle_t *Motor_GetRightChannel(void);

#endif
