#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

#define FORWARD   0
#define BACKWARDS 1

void pwm_start (TIM_HandleTypeDef *htim, uint32_t Channel);
void pwm_stop  (TIM_HandleTypeDef *htim, uint32_t Channel);
void set_direction (GPIO_TypeDef* GPIO_1,
		            uint16_t GPIO_Pin_1,
					GPIO_TypeDef* GPIO_2,
					uint16_t GPIO_Pin_2,
					uint8_t direction);

#endif // MOTOR_CONTROL_H
