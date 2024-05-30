/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Paul Contis
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f446re.h"
#include "rcc_config.h"
#include "pwm_driver.h"
/*************************************************************************************************************************************************/
/*************************************************************************************************************************************************/
#define LOW				0
#define BTN_PRESSED		LOW
/*************************************************************************************************************************************************/

void GPIO_config(void);

//void TIM6Config(void);
/*void delay_us(uint16_t us );*/
void delay_ms(uint16_t ms );

/*************************************************************************************************************************************************/



int main(void)
{
	SysClockConfig();
	/*TIM6Config();*/

	GPIO_config();
	TIM1Config();
	TIM_PWM_Start(); /*not working properly*/

    /* Loop forever */
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay_ms(1000);
	}

	return 0;
}
/*************************************************************************************************************************************************/

/*************************************************************************************************************************************************/

void GPIO_config(void)
{
	GPIO_Handle_t GpioLed;

		/**led gpio configuration*/
		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioLed);

}




/*test SYS CLock
void TIM6Config(void)
{
	!< Enable Timer clock
	RCC->APB1ENR |= (1<<4);
	!< Set the prescalar and the ARR
	TIM6->PSC = 90-1; // 90MHz/90 = 1MHz ~~ 1 uS delay
	TIM6->ARR = 0xffff;	// Max ARR value
	!< Enable the Timer, and wait for the update Flag to set
	TIM6->CR1 |= (1<<0);
	while(!(TIM6->SR & (1<<0))); //UIF: Update interrupt flag. This bit is set by hardware when the registers are updated
}*/



/*
void delay_us(uint16_t us )
{
	RESET the Counter
	TIM6->CNT = 0;
	Wait for the Counter to reach the entered value
	while (TIM6->CNT < us);

}

void delay_ms(uint16_t ms )
{
	uint16_t i;

	for(i=0; i<ms; i++)
	{
		RESET the Counter
		TIM6->CNT = 0;
		Wait for the Counter to reach the entered value
		while (TIM6->CNT < 1000);
	}
}
*/


void delay_ms(uint16_t ms )
{
	uint16_t i;

	for(i=0; i<ms; i++)
	{
		/*RESET the Counter*/
		TIM1->CNT = 0;
		/*Wait for the Counter to reach the entered value*/
		while (TIM1->CNT < 1000);
	}
}

