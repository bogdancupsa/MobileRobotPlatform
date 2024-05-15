#include <stdlib.h>
#include <stdio.h>

#include "../Include/motor_control_interface.h"
#include "../Include/gpio_interface.h"

void pwm_start (TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_PWM_Start(htim, Channel);
}

void pwm_stop  (TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_PWM_Stop(htim, Channel);
}

void set_direction (GPIO_TypeDef* GPIO_1,
		            uint16_t GPIO_Pin_1,
					GPIO_TypeDef* GPIO_2,
					uint16_t GPIO_Pin_2,
					uint8_t direction)
{
	if (FORWARD == direction)
	{
		write_gpio_pin(GPIO_1, GPIO_Pin_1, 1);
		write_gpio_pin(GPIO_2, GPIO_Pin_2, 0);
	}

	else if (BACKWARDS == direction)
	{
		write_gpio_pin(GPIO_1, GPIO_Pin_1, 0);
		write_gpio_pin(GPIO_2, GPIO_Pin_2, 1);
	}

	else
	{
		/* NOTHING */
	}
}
