#ifndef OS_TASKS_H
#define OS_TASKS_H

#include "stm32l4xx_hal.h"

/*-------------------------------------------------------------------------------------------------
Function: HandleUartTask



Description:
    This function masks the code from the uart tasks for the purpose of modularity.



Parameters In:
    huart: UART_HandleTypeDef*  -- handle uart
    pData: uint8_t*             -- buffer to be printed
    Size:  uint16_t             -- size of buffer
    Timeout: uint32_t           -- wait time for uart transmission

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/

void HandleUartTask (UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif //OS_TASKS_H
