#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

void MotorCtrl_Init(void);
void MotorCtrl_Update(void);
void MotorCtrl_Stop(void);
void MotorCtrl_SetDifferentialSpeed(float forward_rps, float turn_rps);
void MotorCtrl_GetSpeed(float *pLeftRps, float *pRightRps);

#endif
