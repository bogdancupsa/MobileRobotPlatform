/*
 * Sensordatacapture.c
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Sensordatacapture.h"

/* Private defines ------------------------------------------------------------*/
#define SENSOR_DEFAULT_DISTANCE 20
#define SENSOR_SPEED_MASK 0.034

/* Private variables ---------------------------------------------------------*/
const SenData_TypeDef SensorMapping[3] =
{
		{POS_FR, SENSOR_DEFAULT_DISTANCE},
		{POS_FL, SENSOR_DEFAULT_DISTANCE},
		{POS_BR, SENSOR_DEFAULT_DISTANCE},
		{POS_BL, SENSOR_DEFAULT_DISTANCE},
};


/* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture = 0;

/* Capture index */
uint16_t               uhCaptureIndex = 0;

/* Frequency Value */
uint32_t               uwFrequency = 0;

uint8_t               Distance = 0;


void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(GPIOA, TRIG_SEN1_Pin , GPIO_PIN_SET);// pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOA, TRIG_SEN1_Pin , GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  htim : hadc handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint16_t data_len = 0;



	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(uhCaptureIndex == 0)
		{
			/* Get the 1st Input Capture value */
			uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			uhCaptureIndex = 1;
		}
		else if(uhCaptureIndex == 1)
		{
			/* Get the 2nd Input Capture value */
			uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			/* Capture computation */
			if (uwIC2Value2 > uwIC2Value1)
			{
				uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
			}
			else if (uwIC2Value2 < uwIC2Value1)
			{
				/* 0xFFFF is max TIM1_CCRx value */
				uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2) + 1;
			}
			else
			{
				/* If capture values are equal, we have reached the limit of frequency measures */
				Error_Handler();
			}


			Distance = (uwDiffCapture * SENSOR_SPEED_MASK)/2;

			SensorMapping[1].Value = Distance;
			SensorMapping[1].orientation = POS_FR;


		}
	}
}


HAL_StatusTypeDef Get_SensorData_Mapping(SenData_TypeDef *mapping)
{
	if(SensorMappingDone == TRUE)
	{
		&mapping = SensorMapping;
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}
