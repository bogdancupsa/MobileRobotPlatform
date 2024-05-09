#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32l4xx_hal.h"

void    write_gpio_pin  (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
uint8_t read_gpio_pin   (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)                        ;
void    toogle_gpio_pin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)                        ;

#endif // GPIO_DRIVER_H
