#include "../../Interface/Include/capture_timer_interface.h"

#include "stdio.h"
#include "stdlib.h"

void start_capture_timer (TIM_HandleTypeDef *htim, uint32_t Channel)
{
	(void)HAL_TIM_IC_Start_IT(htim, Channel);
}

void stop_capture_timer (TIM_HandleTypeDef *htim, uint32_t Channel)
{
	(void)HAL_TIM_IC_Stop_IT(htim, Channel);
}
