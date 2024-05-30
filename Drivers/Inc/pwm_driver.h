/*
 * pwm_driver.h
 *
 *  Created on: Apr 29, 2024
 *      Author: paul.contis
 */

#ifndef INC_PWM_DRIVER_H_
#define INC_PWM_DRIVER_H_

#include "stm32f446re.h"





void TIM1Config(void);
void TIM_PWM_Start(void); //uint8_t timer, uint8_t channel

#endif /* INC_PWM_DRIVER_H_ */
