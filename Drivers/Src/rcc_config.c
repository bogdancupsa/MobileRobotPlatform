/*
 * rcc_config.c
 *
 *  Created on: Apr 29, 2024
 *      Author: paul.contis
 */

#include "rcc_config.h"





Status_t RCC_OscConfig(RCC_OscInit_t  *RCC_OscInitStruct)
{
	return HAL_OK;
}

Status_t PWREx_EnableOverDrive(void)
{
	return HAL_OK;
}

Status_t RCC_ClockConfig(RCC_ClkInit_t  *RCC_ClkInitStruct, uint32_t FLatency)
{
	return HAL_OK;
}


void Error_Handler(void)
{
	;
}

void SystemClockConfig(void)
{
	RCC_OscInit_t RCC_OscInitStruct = {0};
	RCC_ClkInit_t RCC_ClkInitStruct = {0};

	RCC_PWR_CLK_ENABLE();
	PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;

	if (RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	  /* Activate the Over-Drive mode */

	if (PWREx_EnableOverDrive() != HAL_OK)
	{
	    Error_Handler();
	}

	  /* Initializes the CPU, AHB and APB buses clocks */

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}






	/*
#define PLL_M	4
#define PLL_N	180
#define PLL_P	0 /* PLLP = 2
	 */

	/*!< 1. ENABLE HSE and wait for the HSE to become ready
	RCC->CR |= RCC_CR_HSEON;
	while (!RCC->CR & RCC_CR_HSERDY);

	!< 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	!< 3. Configure the FLASH PREFETCH and the LATENCY related setiings
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

	!< 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	//AHB PR
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	//APB1 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	//APB2 PR
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	!< 5. Configure the MAIN PLL
	RCC->PLLCFGR = ( PLL_M << 0 ) | ( PLL_N << 6 ) | ( PLL_P << 16) | (RCC_PLLCFGR_PLLSRC_HSE);

	!< 6. Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;
	while(!( RCC->CR & RCC_CR_PLLRDY ));

	!< 7. Select the Clock Source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL );*/


}

