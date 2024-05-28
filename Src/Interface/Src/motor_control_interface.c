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
	if (BACKWARDS == direction)
	{
		write_gpio_pin(GPIO_1, GPIO_Pin_1, 1);
		write_gpio_pin(GPIO_2, GPIO_Pin_2, 0);
	}

	else if (FORWARD == direction)
	{
		write_gpio_pin(GPIO_1, GPIO_Pin_1, 0);
		write_gpio_pin(GPIO_2, GPIO_Pin_2, 1);
	}

	else
	{
		/* NOTHING */
	}
}

void set_turn_values (uint8_t turn_direction, uint16_t* right_motor_value, uint16_t* left_motor_value, uint8_t* direction_left_front, uint8_t* direction_right_front, uint8_t* direction_left_rear, uint8_t* direction_right_rear)
{
	switch (turn_direction)
	{
		case TURN_LEFT:

			*direction_left_front = BACKWARDS;
			*direction_right_front = FORWARD;
			*direction_left_rear = BACKWARDS;
			*direction_right_rear = FORWARD;

			/* 60% fill factor */
			*right_motor_value = 1200;
			*left_motor_value = 1200;
			break;

		case TURN_RIGHT:

			*direction_left_front = FORWARD;
			*direction_right_front = BACKWARDS;
			*direction_left_rear = FORWARD;
			*direction_right_rear = BACKWARDS;

			/* 60% fill factor */
			*right_motor_value = 1200;
			*left_motor_value = 1200;
			break;

		case GO:

			*direction_left_front = FORWARD;
			*direction_right_front = FORWARD;
			*direction_left_rear = FORWARD;
			*direction_right_rear = FORWARD;

			/* 50% fill factor */
			*right_motor_value = 1000;
			*left_motor_value = 1000;
			break;

		case STOP:
			*direction_left_front = FORWARD;
			*direction_right_front = FORWARD;
			*direction_left_rear = FORWARD;
			*direction_right_rear = FORWARD;

			/* 0% fill factor */
			*right_motor_value = 0;
			*left_motor_value = 0;
			break;

		default:
			*direction_left_front = FORWARD;
			*direction_right_front = FORWARD;
			*direction_left_rear = FORWARD;
			*direction_right_rear = FORWARD;

			/* 0% fill factor */
			*right_motor_value = 0;
			*left_motor_value = 0;
			break;
	}
}
