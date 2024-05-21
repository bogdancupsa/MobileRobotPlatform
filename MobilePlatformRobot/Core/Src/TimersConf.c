/*
 * TimersConf.c
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */



HAL_StatusTypeDef Timers_Configuration(void)
{
	HAL_StatusTypeDef status = HAL_OK;

	/*Start PWM for motor control*/
	if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)!= HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
		status = HAL_ERROR;
	}

	if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2)!= HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
		status = HAL_ERROR;
	}

	/*Start the Input Capture in interrupt mode for sensor*/
	if(HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
		status = HAL_ERROR;
	}

	/*Timer configured at 1MHz */
	if(HAL_TIM_Base_Start(&htim1)  != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
		status = HAL_ERROR;
	}

	reutrn status;
}

void delay_us (uint16_t us)
{
	//__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	//while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter

	htim3.Instance->ARR = us - 1; /* auto-reload register */
	htim3.Instance->EGR = 1;      /* re-initialize timer  */
	htim3.Instance->SR &= ~1;     /* reset flag           */
	htim3.Instance->CR1 |= 1;     /* enable counter       */
	while((htim3.Instance->SR & 0x0001) != 1);
	htim3.Instance->SR &= ~(0x0001);
}
