/*
 * test_app.c
 *
 *  Created on: Apr 25, 2024
 *      Author: paul.contis
 */

#include <gpio_drivers.h>
#include "test_app.h"
/*************************************************************************************************************************************************/
#define LOW				0
#define BTN_PRESSED		LOW
/*************************************************************************************************************************************************/
void sw_delay(void);
/*************************************************************************************************************************************************/

void test_GPIO_config(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBtn;

		/* led gpio configuration */
		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioLed);

		/* button gpio configuration */
		GpioBtn.pGPIOx = GPIOC;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioBtn);
}

void test_GPIO_blink(void)
{
	if((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) == BTN_PRESSED)
	{
		sw_delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
	}
}

void sw_delay(void)
{
	for(uint32_t count = 0 ; count < 500000; count++);
}

