#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

#define FORWARD    0
#define BACKWARDS  1

#define TURN_LEFT  0
#define TURN_RIGHT 1
#define GO         2
#define STOP       3

#define STOP_LIMIT 10

void pwm_start (TIM_HandleTypeDef *htim, uint32_t Channel);

void pwm_stop  (TIM_HandleTypeDef *htim, uint32_t Channel);

void set_direction (GPIO_TypeDef* GPIO_1,
		            uint16_t GPIO_Pin_1,
					GPIO_TypeDef* GPIO_2,
					uint16_t GPIO_Pin_2,
					uint8_t direction);

void set_turn_values (uint8_t turn_direction,
		              uint16_t* right_motor_value,
					  uint16_t* left_motor_value,
					  uint8_t* direction_1_2,
					  uint8_t* direction_3_4);

#endif // MOTOR_CONTROL_H
