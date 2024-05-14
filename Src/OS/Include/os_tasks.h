#ifndef OS_TASKS_H
#define OS_TASKS_H

#include "stm32l4xx_hal.h"

void HandleUartTask (UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif //OS_TASKS_H
