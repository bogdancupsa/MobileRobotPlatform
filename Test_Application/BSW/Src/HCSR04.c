/*
 * HCSR04.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "HCSR04.h"

extern TIM_HandleTypeDef htim3;


void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(__HAL_TIM_GET_COUNTER(&htim3) < time);
}



uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
uint8_t Distance = 0;

#define TRIG_PIN GPIO_PIN_9
#define TRIG_Port GPIOA

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		if(Is_First_Captured==0)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			Is_First_Captured =1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(Is_First_Captured == 1)
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if( IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}
			else if(IC_Val2 > IC_Val1)
			{
				Difference = (0xffff - IC_Val1)+ IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC4);

		}
	}
}

void HCSR04_Read(void)
{
	HAL_GPIO_WritePin(TRIG_Port, TRIG_PIN, GPIO_PIN_SET);
	delay(10);
	HAL_GPIO_WritePin(TRIG_Port, TRIG_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC4);
}
