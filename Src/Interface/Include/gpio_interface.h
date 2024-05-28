#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32l4xx_hal.h"

/*-------------------------------------------------------------------------------------------------
Function: write_gpio_pin



Description:
    This function is used to write a GPIO pin.



Parameters In:
    GPIOx:  GPIO_TypeDef*    -- GPIO port
    GPIO_Pin: uint16_t       -- GPIO pin
    PinState: GPIO_PinState  -- desired pin state

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/

void    write_gpio_pin  (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);




/*-------------------------------------------------------------------------------------------------
Function: read_gpio_pin



Description:
    This function is used to read a GPIO pin.



Parameters In:
    GPIOx:  GPIO_TypeDef*    -- GPIO port
    GPIO_Pin: uint16_t       -- GPIO pin

Parameters Out:
    None

Return Value:
    state of a gpio pin:   uint8_t
-------------------------------------------------------------------------------------------------*/

uint8_t read_gpio_pin   (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)                        ;




/*-------------------------------------------------------------------------------------------------
Function: toogle_gpio_pin



Description:
    This function is used to toogle a GPIO pin.



Parameters In:
    GPIOx:  GPIO_TypeDef*    -- GPIO port
    GPIO_Pin: uint16_t       -- GPIO pin

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/

void    toogle_gpio_pin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)                        ;




#endif // GPIO_DRIVER_H
