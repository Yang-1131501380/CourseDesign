#include "motor_ctrl.h"
#include "motor_driver.h"
#include "motor.h"

#define MOTOR_CTRL_ENCODER_LINES             500U
#define MOTOR_CTRL_ENCODER_QUADRATURE_MULT   4U
#define MOTOR_CTRL_GEAR_RATIO_NUMERATOR      34014U
#define MOTOR_CTRL_GEAR_RATIO_DENOMINATOR    1000U
#define MOTOR_CTRL_DEFAULT_PULSES_PER_REV \
    ((int32_t)((MOTOR_CTRL_ENCODER_LINES * MOTOR_CTRL_ENCODER_QUADRATURE_MULT * \
                MOTOR_CTRL_GEAR_RATIO_NUMERATOR + (MOTOR_CTRL_GEAR_RATIO_DENOMINATOR / 2U)) / \
               MOTOR_CTRL_GEAR_RATIO_DENOMINATOR))
#define MOTOR_CTRL_DEFAULT_KP 0.2f
#define MOTOR_CTRL_DEFAULT_KI 0.08f
#define MOTOR_CTRL_DEFAULT_KD 0.0f

typedef struct _chassis {
    Encoder_Handle_t leftEncoder;
    Encoder_Handle_t rightEncoder;
    Motor_t leftMotor;
    Motor_t rightMotor;
    uint8_t ready;
} CHASSIS_S;

static CHASSIS_S g_chassis = {
    .leftEncoder = {
        .htim = &htim5,
        .direction = 1,
        .pulses_per_rev = MOTOR_CTRL_DEFAULT_PULSES_PER_REV
    },
    .rightEncoder = {
        .htim = &htim3,
        .direction = -1,
        .pulses_per_rev = MOTOR_CTRL_DEFAULT_PULSES_PER_REV
    }
};

void MotorCtrl_Init(void)
{
    if (g_chassis.ready != 0U) {
        return;
    }

    Motor_Init(&g_chassis.leftMotor, Motor_GetLeftChannel(),
               &g_chassis.leftEncoder, MOTOR_CTRL_DEFAULT_KP,
               MOTOR_CTRL_DEFAULT_KI, MOTOR_CTRL_DEFAULT_KD);
    Motor_Init(&g_chassis.rightMotor, Motor_GetRightChannel(),
               &g_chassis.rightEncoder, MOTOR_CTRL_DEFAULT_KP,
               MOTOR_CTRL_DEFAULT_KI, MOTOR_CTRL_DEFAULT_KD);
    g_chassis.ready = 1U;
}

void MotorCtrl_Update(void)
{
    if (g_chassis.ready == 0U) {
        return;
    }

    Motor_Update(&g_chassis.leftMotor);
    Motor_Update(&g_chassis.rightMotor);
}

void MotorCtrl_Stop(void)
{
    if (g_chassis.ready == 0U) {
        return;
    }

    Motor_Brake(&g_chassis.leftMotor);
    Motor_Brake(&g_chassis.rightMotor);
}

void MotorCtrl_SetDifferentialSpeed(float forward_rps, float turn_rps)
{
    if (g_chassis.ready == 0U) {
        return;
    }

    Motor_SetSpeed(&g_chassis.leftMotor, forward_rps - turn_rps);
    Motor_SetSpeed(&g_chassis.rightMotor, forward_rps + turn_rps);
}

void MotorCtrl_GetSpeed(float *pLeftRps, float *pRightRps)
{
    if ((pLeftRps == 0) || (pRightRps == 0)) {
        return;
    }

    *pLeftRps = Motor_GetCurrentSpeed(&g_chassis.leftMotor);
    *pRightRps = Motor_GetCurrentSpeed(&g_chassis.rightMotor);
}
