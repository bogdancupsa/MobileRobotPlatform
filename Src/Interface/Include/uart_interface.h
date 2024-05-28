#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "stm32l4xx_hal.h"

/*-------------------------------------------------------------------------------------------------
Function: uart_transmit



Description:
    This function is a wrapper for HAL_UART_Transmit function.



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

void uart_transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif // GPIO_DRIVER_H
