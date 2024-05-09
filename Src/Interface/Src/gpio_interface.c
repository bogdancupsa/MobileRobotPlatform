#include "../../Interface/Include/gpio_interface.h"

#include "stdio.h"
#include "stdlib.h"


void write_gpio_pin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

uint8_t read_gpio_pin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	return (uint8_t)HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void toogle_gpio_pin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}
