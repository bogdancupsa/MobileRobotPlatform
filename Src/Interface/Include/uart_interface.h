#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "stm32l4xx_hal.h"

void uart_transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif // GPIO_DRIVER_H
