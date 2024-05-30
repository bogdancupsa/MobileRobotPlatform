/*
 * HBridge.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "HBridge.h"
#include "main.h"

void AdjustDutyCycle(uint16_t Speed, uint8_t Motor);



void HBridge_MotorControl(Wheels_t *Wheels, RotateDir_t *Directions, Speeds_t *Speeds)
{
	if ((Wheels->BackLeftWheel) && (!Wheels->BackRightWheel))
	{
		/** Rotate only left wheel*/
		if (Directions->RotateForward.BackLeftWheel)
		{
			AdjustDutyCycle(Speeds->Speeds.BackLeftWheel, LEFTMOTOR);
			HAL_GPIO_WritePin(LeftWheel_IN1_GPIO_Port, LeftWheel_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LeftWheel_IN2_GPIO_Port, LeftWheel_IN2_Pin, GPIO_PIN_RESET);
		}
		else if(Directions->RotateBackward.BackLeftWheel)
		{
			AdjustDutyCycle(Speeds->Speeds.BackLeftWheel, LEFTMOTOR);
			HAL_GPIO_WritePin(LeftWheel_IN1_GPIO_Port, LeftWheel_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LeftWheel_IN2_GPIO_Port, LeftWheel_IN2_Pin, GPIO_PIN_SET);
		}
	}
	else if((!Wheels->BackLeftWheel) && (Wheels->BackRightWheel))
	{
		/** Rotate only right wheel*/
		if (Directions->RotateForward.BackRightWheel)
		{
			AdjustDutyCycle(Speeds->Speeds.BackRightWheel, RIGHTMOTOR);
			HAL_GPIO_WritePin(RightWheel_IN3_GPIO_Port, RightWheel_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RightWheel_IN4_GPIO_Port, RightWheel_IN4_Pin, GPIO_PIN_RESET);
		}
		else if(Directions->RotateBackward.BackRightWheel)
		{
			AdjustDutyCycle(Speeds->Speeds.BackRightWheel, RIGHTMOTOR);
			HAL_GPIO_WritePin(RightWheel_IN3_GPIO_Port, RightWheel_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightWheel_IN4_GPIO_Port, RightWheel_IN4_Pin, GPIO_PIN_SET);
		}
	}
	else if((Wheels->BackLeftWheel) && (Wheels->BackRightWheel))
	{
		/** Rotate both wheels */
		if ((Directions->RotateForward.BackLeftWheel)&&(Directions->RotateForward.BackRightWheel))
		{
			AdjustDutyCycle(Speeds->Speeds.BackLeftWheel, LEFTMOTOR);
			AdjustDutyCycle(Speeds->Speeds.BackRightWheel, RIGHTMOTOR);
			HAL_GPIO_WritePin(LeftWheel_IN1_GPIO_Port, LeftWheel_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LeftWheel_IN2_GPIO_Port, LeftWheel_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightWheel_IN3_GPIO_Port, RightWheel_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RightWheel_IN4_GPIO_Port, RightWheel_IN4_Pin, GPIO_PIN_RESET);
		}
		else if((Directions->RotateBackward.BackLeftWheel)&&(Directions->RotateBackward.BackRightWheel))
		{
			AdjustDutyCycle(Speeds->Speeds.BackLeftWheel, LEFTMOTOR);
			AdjustDutyCycle(Speeds->Speeds.BackRightWheel, RIGHTMOTOR);
			HAL_GPIO_WritePin(LeftWheel_IN1_GPIO_Port, LeftWheel_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LeftWheel_IN2_GPIO_Port, LeftWheel_IN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RightWheel_IN3_GPIO_Port, RightWheel_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightWheel_IN4_GPIO_Port, RightWheel_IN4_Pin, GPIO_PIN_SET);
		}
	}
}


void HBridge_MotorStop(Wheels_t *Wheels)
{
	if ((!Wheels->BackLeftWheel) && (Wheels->BackRightWheel))
	{
		//AdjustDutyCycle(PWM1, DUTYCYCLE_0); //TODO: Set duty cycle for PWM1 to 0
		HAL_GPIO_WritePin(LeftWheel_IN1_GPIO_Port, LeftWheel_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LeftWheel_IN2_GPIO_Port, LeftWheel_IN2_Pin, GPIO_PIN_RESET);
	}
	else if((!Wheels->BackRightWheel) && (Wheels->BackLeftWheel))
	{
		//AdjustDutyCycle(PWM2, DUTYCYCLE_0); //TODO: Set duty cycle for PWM2 to 0
		HAL_GPIO_WritePin(RightWheel_IN3_GPIO_Port, RightWheel_IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RightWheel_IN4_GPIO_Port, RightWheel_IN4_Pin, GPIO_PIN_RESET);
	}
	else
	{
		//AdjustDutyCycle(PWM, DUTYCYCLE_0); //TODO: Set duty cycle for both PWM to 0
		HAL_GPIO_WritePin(LeftWheel_IN1_GPIO_Port, LeftWheel_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LeftWheel_IN2_GPIO_Port, LeftWheel_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RightWheel_IN3_GPIO_Port, RightWheel_IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RightWheel_IN4_GPIO_Port, RightWheel_IN4_Pin, GPIO_PIN_RESET);
	}
}


void HBridge_PinConfig(void)
{




}


void AdjustDutyCycle(uint16_t Speed, uint8_t Motor)
{
	if(Motor == LEFTMOTOR)
	{
		htim1.Instance->CCR3 = Speed;
	}
	else if(Motor == RIGHTMOTOR)
	{
		htim8.Instance->CCR2 = Speed;
	}
}




