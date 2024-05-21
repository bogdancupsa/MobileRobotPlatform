/*
 * PWM_Driver.c
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */


void Set_PWM_Pulse(TIM_HandleTypeDef *htim,uint32_t channel, uint32_t pulse)
{
	switch (channel)
	{
		case TIM_CHANNEL_1:
			htim->Instance->CCR1 = pulse;
			break;
		case TIM_CHANNEL_2:
			htim->Instance->CCR2 = pulse;
			break;
		case TIM_CHANNEL_3:
			htim->Instance->CCR3 = pulse;
			break;
		case TIM_CHANNEL_4:
			htim->Instance->CCR4 = pulse;
			break;
		case TIM_CHANNEL_5:
			htim->Instance->CCR5 = pulse;
			break;
		case TIM_CHANNEL_6:
			htim->Instance->CCR6 = pulse;
			break;
		default:
		    break;
	}
}
