#ifndef CAPTURE_TIMER_DRIVER_H
#define CAPTURE_TIMER_DRIVER_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

/*-------------------------------------------------------------------------------------------------
Function: start_capture_timer



Description:
    This function is used to start capture timer.



Parameters In:
    htim: TIM_HandleTypeDef*    --   holds the value of the used timer
    Channel:  uint32_t          --   Channel value

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/

void start_capture_timer     (TIM_HandleTypeDef *htim, uint32_t Channel);

/*-------------------------------------------------------------------------------------------------
Function: stop_capture_timer



Description:
    This function is used to stop capture timer.



Parameters In:
    htim: TIM_HandleTypeDef*    --   holds the value of the used timer
    Channel:  uint32_t          --   Channel value

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/

void stop_capture_timer      (TIM_HandleTypeDef *htim, uint32_t Channel);

#endif // CAPTURE_TIMER_DRIVER_H
