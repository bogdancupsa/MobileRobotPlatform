/*
 * obstacleDetection.c
 *
 *  Created on: May 20, 2024
 *      Author: alexandru.coca
 */


#include "main.h"
#include "obstacleDetection.h"

extern TIM_HandleTypeDef htim3;

volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile uint8_t echo_flag = 0;

float get_distance()
{
	uint32_t duration;
	float distance, local_time=0, sensor_time =0;
	trig_GPIO_Port->BSRR = trig_Pin;
	HAL_Delay(10);
	trig_GPIO_Port->BSRR = (uint32_t)trig_Pin << 16U;
//if()
//	if(HAL_GPIO_ReadPin(GPIOA, echo_Pin) == GPIO_PIN_SET)
//	{
//		echo_start = __HAL_TIM_GET_COUNTER(&htim3);
//	}
//	else
//	{
//		echo_end = __HAL_TIM_GET_COUNTER(&htim3);
//		echo_flag = 1;
//	}
//
//	while(echo_flag == 0);
//
//	duration = echo_end - echo_start;
//	distance = duration * 0.034 / 2;
//
//	echo_flag = 0;
//	return distance;
	  while (!(HAL_GPIO_ReadPin(GPIOA, echo_Pin)));  // Wait for Echo pin to go HIGH
	  while (HAL_GPIO_ReadPin(GPIOA, echo_Pin))     // Wait for Echo pin to go LOW
	  {
	    local_time++;
	    HAL_Delay(1);
	  }

	  sensor_time = local_time * 2;
	  return sensor_time * 0.034 / 2; // convert to cm
}

//void get_timer(){
//	while (!(HAL_GPIO_ReadPin(GPIOA, echo_Pin)));  // Wait for Echo pin to go HIGH
//    while (HAL_GPIO_ReadPin(GPIOA, echo_Pin))      // Wait for Echo pin to go LOW
//    {
//    	duration++;
//        HAL_Delay(1);
//    }
//
//    sensor_time = local_time * 2;
//    return sensor_time * 0.034 / 2; // convert to cm
//}
