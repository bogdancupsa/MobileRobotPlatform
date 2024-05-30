/*
 * pwm_driver.c
 *
 *  Created on: Apr 29, 2024
 *      Author: paul.contis
 */

#include "pwm_driver.h"

/*
 * Period = Cycle Time ( high and low )
 * Frequency = 1/Period
 * Duty Cycle = high time / period
 * Vout = Vmax * Duty Cycle
 */


void TIM1Config(void)
{
	/*!< Enable Timer clock*/
	RCC->APB2ENR |= (1<<0);
	/*!< Set the prescalar and the ARR*/
	TIM1->PSC = 90-1; // 90MHz/90 = 1MHz ~~ 1 uS delay
	TIM1->ARR = 0xffff;	// Max ARR value
	/*!< Enable the Timer, and wait for the update Flag to set*/
	TIM1->CR1 |= (1<<0);
	while(!(TIM1->SR & (1<<0))); //UIF: Update interrupt flag. This bit is set by hardware when the registers are updated*/

}


void TIM_PWM_Start(void) //uint8_t timer, uint8_t channel
{


	TIM1->CCMR1 = 0x68;
	TIM1->CCER = 1;
	TIM1->CCR1 = 65534;
	TIM1->DMAR = 0x81;
	TIM1->BDTR = 0x8000;


}



