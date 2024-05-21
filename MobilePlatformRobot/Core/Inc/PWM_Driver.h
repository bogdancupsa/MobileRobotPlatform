/*
 * PWM_Driver.h
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */

#ifndef INC_PWM_DRIVER_H_
#define INC_PWM_DRIVER_H_

void Set_PWM_Pulse(TIM_HandleTypeDef *htim,uint32_t channel, uint32_t pulse);

#endif /* INC_PWM_DRIVER_H_ */
