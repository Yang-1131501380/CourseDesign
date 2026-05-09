#include "motor_driver.h"

static TB6612_Handle_t g_left_motor = {
    .htim = &htim4,
    .channel = TIM_CHANNEL_3,
    .in1_port = AIN1_GPIO_Port,
    .in1_pin = AIN1_Pin,
    .in2_port = AIN2_GPIO_Port,
    .in2_pin = AIN2_Pin,
    .stby_port = STBY_GPIO_Port,
    .stby_pin = STBY_Pin,
    .logical_max_output = 1000
};

static TB6612_Handle_t g_right_motor = {
    .htim = &htim4,
    .channel = TIM_CHANNEL_4,
    .in1_port = BIN2_GPIO_Port,
    .in1_pin = BIN2_Pin,
    .in2_port = BIN1_GPIO_Port,
    .in2_pin = BIN1_Pin,
    .stby_port = STBY_GPIO_Port,
    .stby_pin = STBY_Pin,
    .logical_max_output = 1000
};

void MotorDriver_Init(void)
{
    TB6612_Init(&g_left_motor);
    TB6612_Init(&g_right_motor);
}

TB6612_Handle_t *Motor_GetLeftChannel(void)
{
    return &g_left_motor;
}

TB6612_Handle_t *Motor_GetRightChannel(void)
{
    return &g_right_motor;
}
