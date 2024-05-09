#include "../../Interface/Include/uart_interface.h"

#include "stdio.h"
#include "stdlib.h"


void uart_transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	(void)HAL_UART_Transmit(huart, pData, Size, Timeout);
}
