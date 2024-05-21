/*
 * MotorsControlService.c
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "MotorsControlService.h"

#define MAX_PWM_VALUE 624u
#define TURN_PWM_VALUE MAX_PWM_VALUE/2


MCS_Direction_TypeDef Direction;
MCS_State_TypeDef MCSState = MCS_LOCKED;


HAL_StatusTypeDef MCS_Init(void)
{
	Direction = MCS_NOTURN;
	MCSState = MCS_LOCKED;

	return HAL_OK;
}

static void MCS_SetDirection(MCS_Direction_TypeDef Direction)
{
	if(Direction ==  MCS_LEFT)
	{
		Set_PWM_Pulse(&htim2, TIM_CHANNEL_1,MAX_PWM_VALUE);
		Set_PWM_Pulse(&htim2, TIM_CHANNEL_2,MAX_PWM_VALUE);
	}
	else if (Direction ==  MCS_RiGHT )
	{
		Set_PWM_Pulse(&htim2, TIM_CHANNEL_1,MAX_PWM_VALUE);
		Set_PWM_Pulse(&htim2, TIM_CHANNEL_2,MAX_PWM_VALUE);
	}
	else
	{
		Set_PWM_Pulse(&htim2, TIM_CHANNEL_1,MAX_PWM_VALUE);
		Set_PWM_Pulse(&htim2, TIM_CHANNEL_2,MAX_PWM_VALUE);
	}

}

void MCS_MainFunction(void)
{
	switch(MCSState)
	{
		case MCS_FORWARD:
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR1_Pin , GPIO_PIN_RESET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR1_Pin , GPIO_PIN_SET);//IN2
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR2_Pin , GPIO_PIN_RESET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR2_Pin , GPIO_PIN_SET);//IN2

			break;
		case MCS_BACKWARD:
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR1_Pin , GPIO_PIN_SET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR1_Pin , GPIO_PIN_RESET);//IN2
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR2_Pin , GPIO_PIN_SET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR2_Pin , GPIO_PIN_RESET);//IN2

			break;
		case MCS_BREAK:
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR1_Pin , GPIO_PIN_RESET);//IN
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR1_Pin , GPIO_PIN_RESET);//IN2
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR2_Pin , GPIO_PIN_RESET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR2_Pin , GPIO_PIN_RESET);//IN2
			break;
		case MCS_ERROR:

			break;
		default:
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR1_Pin , GPIO_PIN_RESET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR1_Pin , GPIO_PIN_RESET);//IN2
			HAL_GPIO_WritePin(GPIOA , IN1_MOTOR2_Pin , GPIO_PIN_RESET);//IN1
			HAL_GPIO_WritePin(GPIOA, IN2_MOTOR2_Pin , GPIO_PIN_RESET);//IN2
			break;
	}



}
