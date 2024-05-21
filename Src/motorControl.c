/*
 * motorControl.c
 *
 *  Created on: May 14, 2024
 *      Author: alexandru.coca
 */
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "motorControl.h"

extern TIM_HandleTypeDef htim1;

void pwm_start (TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_PWM_Start(htim, Channel);
}


void setDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle)
{
	switch(Channel)
	{
	case TIM_CHANNEL_1:
		htim->Instance->CCR1 = dutyCycle;
		break;
	case TIM_CHANNEL_2:
		htim->Instance->CCR2 = dutyCycle;
		break;
	}
}
 void set_direction(uint8_t direction) {


    if (direction == 1) {
  	  In1_GPIO_Port->BSRR = In1_Pin;//set the pin
  	  In2_GPIO_Port->BSRR = (uint32_t)In2_Pin << 16U;// reset the pin
  	  In4_GPIO_Port->BSRR = In4_Pin;
  	  In3_GPIO_Port->BSRR = (uint32_t)In3_Pin << 16U;
	  setDutyCycle(&htim1, TIM_CHANNEL_1, 1000);
	  setDutyCycle(&htim1, TIM_CHANNEL_2, 1000);
    } else if(direction == 0){
    	  In2_GPIO_Port->BSRR = In2_Pin;//set the pin
    	  In1_GPIO_Port->BSRR = (uint32_t)In1_Pin << 16U;// reset the pin
    	  In3_GPIO_Port->BSRR = In3_Pin;
    	  In4_GPIO_Port->BSRR = (uint32_t)In4_Pin << 16U;
    	  setDutyCycle(&htim1, TIM_CHANNEL_1, 1000);
    	  setDutyCycle(&htim1, TIM_CHANNEL_2, 1000);
    }


}

void stop(){
	  setDutyCycle(&htim1, TIM_CHANNEL_1, 0);
	  setDutyCycle(&htim1, TIM_CHANNEL_2, 0);
}

void steer_left(){

	  setDutyCycle(&htim1, TIM_CHANNEL_1, 200);
	  setDutyCycle(&htim1, TIM_CHANNEL_2, 900);
}

void steer_right(void){

	  setDutyCycle(&htim1, TIM_CHANNEL_1, 900);
	  setDutyCycle(&htim1, TIM_CHANNEL_2, 200);
}


