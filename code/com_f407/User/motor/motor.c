#include "motor/motor.h"
#include "tim.h"

void motor_start(void)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void motor_stop(void)
{
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}
