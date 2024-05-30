/*
 * ServoSG90.h
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */

#ifndef INC_SERVOSG90_H_
#define INC_SERVOSG90_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

void ServoSG90_SteeringWheelCtrl(uint16_t Direction);

#define	TURN_LEFT				2000 // 2 ms
#define TURN_RIGHT				1000 // 1 ms
#define TURN_BACK				1500 // 1.5 ms -initial position
#define TURN_LEFT_REVERSE		3
#define TURN_RIGHT_REVERSE		4

extern TIM_HandleTypeDef htim2;




#endif /* INC_SERVOSG90_H_ */
