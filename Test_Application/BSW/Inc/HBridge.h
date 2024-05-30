/*
 * HBridge.h
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */

#ifndef INC_HBRIDGE_H_
#define INC_HBRIDGE_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*TODO:move to GPIO driver*/
#define LeftWheel_IN1_Pin GPIO_PIN_4
#define LeftWheel_IN1_GPIO_Port GPIOA
#define LeftWheel_IN2_Pin GPIO_PIN_5
#define LeftWheel_IN2_GPIO_Port GPIOA
#define RightWheel_IN3_Pin GPIO_PIN_6
#define RightWheel_IN3_GPIO_Port GPIOA
#define RightWheel_IN4_Pin GPIO_PIN_7
#define RightWheel_IN4_GPIO_Port GPIOA

#define LEFTMOTOR 		0
#define RIGHTMOTOR		1


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;


typedef struct
{
	uint16_t BackRightWheel;
	uint16_t BackLeftWheel;
}Wheels_t;

typedef struct
{
	Wheels_t RotateForward;
	Wheels_t RotateBackward;

}RotateDir_t;

typedef struct
{
	Wheels_t Speeds;

}Speeds_t;


void HBridge_MotorControl(Wheels_t *Wheels, RotateDir_t *Directions, Speeds_t *Speeds);
void HBridge_MotorStop(Wheels_t *Wheels);
void HBridge_PinConfig(void);



#endif /* INC_HBRIDGE_H_ */
