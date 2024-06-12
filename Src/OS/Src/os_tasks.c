#include <stdlib.h>
#include "../Include/os_tasks.h"
#include "../../Interface/Include/uart_interface.h"
#include "../../Interface/Include/gpio_interface.h"

void HandleUartTask (UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	(void)uart_transmit(huart, pData, Size, Timeout);
}
