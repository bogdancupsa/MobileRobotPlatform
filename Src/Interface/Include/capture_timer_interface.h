#ifndef CAPTURE_TIMER_DRIVER_H
#define CAPTURE_TIMER_DRIVER_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

void start_capture_timer     (TIM_HandleTypeDef *htim, uint32_t Channel);
void stop_capture_timer      (TIM_HandleTypeDef *htim, uint32_t Channel);

#endif // CAPTURE_TIMER_DRIVER_H
