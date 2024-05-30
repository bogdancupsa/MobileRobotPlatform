/*
 * stm32f446re.h
 *
 *  Created on: Mar 27, 2024
 *      Author: paul.contis
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_

/*************************************************************************************************************************************************/
#include <stdint.h>
#include <stddef.h>
/*************************************************************************************************************************************************/
#define __vo								volatile
#define __weak 								__attribute__((weak))

/**************************************************************************************************************************************************
 *															 Processor Specific Details
 *************************************************************************************************************************************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx register addresses -set
 */
#define NVIC_ISER0							((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE000E10C)
/*
 * ARM Cortex M4 Processor NVIC ICERx register addresses -clear
 */
#define NVIC_ICER0							((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0XE000E18C)
/*
 * ARM Cortex M4 Processor Interrupt Priority Registers Address Calculation
 */
#define NVIC_PR_BASE_ADDR					((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED				4
/**************************************************************************************************************************************************
 *																	Base addresses
 *************************************************************************************************************************************************/
/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR						0x08000000U	/*  */
#define SRAM1_BASEADDR						0x20000000U /* 112 KB */
#define SRAM2_BASEADDR						0x2001C000U /* 16  KB */
#define ROM_BASEADDR						0x1FFF0000U /*  */
#define SRAM_BASEADDR						SRAM1_BASEADDR
/*************************************************************************************************************************************************/
/*
 * AHBx and APBx Bus Peripherals  base addresses
 */
#define PERIPH_BASEADDR						0x40000000U /*  */
#define APB1PERIPH_BASE						PERIPH_BASEADDR
#define APB2PERIPH_BASE						0x40010000U
#define AHB1PERIPH_BASE						0x40020000U
#define AHB2PERIPH_BASE						0x50000000U
#define AHB3PERIPH_BASE						0x60000000U
/*************************************************************************************************************************************************/
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define TIM2_BASEADDR	        			PERIPH_BASEADDR
#define TIM3_BASEADDR						0x40000400U
#define	TIM4_BASEADDR						0x40000800U
#define	TIM5_BASEADDR						0x40000C00U
#define	TIM6_BASEADDR						0x40001000U
#define TIM7_BASEADDR						0x40001400U
#define TIM12_BASEADDR						0x40001800U
#define TIM13_BASEADDR						0x40001C00U
#define TIM14_BASEADDR						0x40002000U
#define RTC_BASEADDR						0x40002800U

#define PWR_BASE              				(APB1PERIPH_BASE + 0x7000UL)


/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_PWREN_Pos              (28U)
#define RCC_APB1ENR_PWREN_Msk              (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk


/*************************************************************************************************************************************************/
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define TIM1_BASEADDR 						0x40010000U
#define TIM8_BASEADDR						0x40010400U
#define USART1_BASEADDR						0x40011000U
#define USART6_BASEADDR						0x40011400U
#define ADC1_BASEADDR						0x40012000U
#define ADC2_BASEADDR						0x40012100U
#define ADC3_BASEADDR						0x40012200U
#define SPI1_BASEADDR						0x40013000U
#define SPI4_BASEADDR						0x40013400U
#define SYSCFG_BASEADDR						0x40013800U
#define EXTI_BASEADDR                   	0x40013C00U
#define TIM9_BASEADDR0                  	0x40014000U
#define TIM10_BASEADDR                 	 	0x40014400U
#define TIM11_BASEADDR                  	0x40014800U
/*************************************************************************************************************************************************/
/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR                  	0x40020000U
#define GPIOB_BASEADDR                  	0x40020400U
#define GPIOC_BASEADDR                  	0x40020800U
#define GPIOD_BASEADDR                  	0x40020C00U
#define GPIOE_BASEADDR                  	0x40021000U
#define GPIOF_BASEADDR						0x40021400U
#define GPIOG_BASEADDR						0x40021800U
#define GPIOH_BASEADDR						0x40021C00U

#define RCC_BASEADDR						0x40023800U


#define FLASH_R_BASE          				(AHB1PERIPH_BASE + 0x3C00UL)


/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/*!< PVD level configuration */
#define PWR_CR_VOS_Pos         (14U)
#define PWR_CR_VOS_Msk         (0x3UL << PWR_CR_VOS_Pos)                        /*!< 0x0000C000 */
#define PWR_CR_VOS             PWR_CR_VOS_Msk                                  /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */



/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_ICEN_Pos             (9U)
#define FLASH_ACR_ICEN_Msk             (0x1UL << FLASH_ACR_ICEN_Pos)            /*!< 0x00000200 */
#define FLASH_ACR_ICEN                 FLASH_ACR_ICEN_Msk
#define FLASH_ACR_DCEN_Pos             (10U)
#define FLASH_ACR_DCEN_Msk             (0x1UL << FLASH_ACR_DCEN_Pos)            /*!< 0x00000400 */
#define FLASH_ACR_DCEN                 FLASH_ACR_DCEN_Msk
#define FLASH_ACR_PRFTEN_Pos           (8U)
#define FLASH_ACR_PRFTEN_Msk           (0x1UL << FLASH_ACR_PRFTEN_Pos)          /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN               FLASH_ACR_PRFTEN_Msk
#define FLASH_ACR_LATENCY_5WS          0x00000005U

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSEON_Pos                   (16U)
#define RCC_CR_HSEON_Msk                   (0x1UL << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSEON                       RCC_CR_HSEON_Msk

#define RCC_CR_HSERDY_Pos                  (17U)
#define RCC_CR_HSERDY_Msk                  (0x1UL << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */
#define RCC_CR_HSERDY                      RCC_CR_HSERDY_Msk

#define RCC_CR_PLLON_Pos                   (24U)
#define RCC_CR_PLLON_Msk                   (0x1UL << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */
#define RCC_CR_PLLON                       RCC_CR_PLLON_Msk

#define RCC_CR_PLLRDY_Pos                  (25U)
#define RCC_CR_PLLRDY_Msk                  (0x1UL << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */
#define RCC_CR_PLLRDY                      RCC_CR_PLLRDY_Msk

/********************  Bit definition for RCC_CFGR register  ******************/
#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)
#define RCC_CFGR_SWS_Msk                   (0x3UL << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C */
#define RCC_CFGR_SWS                       RCC_CFGR_SWS_Msk                    /*!< SWS[1:0] bits (System Clock Switch Status) */

#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock                   */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk

#define TIM1_BASE             			   (APB2PERIPH_BASE + 0x0000UL)
#define TIM1                			   ((TIM_RegDef_t*) TIM1_BASE)
#define TIM6_BASE             			   (APB1PERIPH_BASE + 0x1000UL)
#define TIM6                			   ((TIM_RegDef_t*) TIM6_BASE)


/**************************************************************************************************************************************************
 *  	   	   	   	   	   	   	   	   	   	   	   	   	   Peripheral register definition structures
 *************************************************************************************************************************************************/
/*
 * This section gives a detailed description of the GPIO registers.
 * The GPIO registers can be accessed by byte (8 bits), half-words (16 bits) or words (32 bits).
 */
typedef struct
{
  __vo uint32_t MODER; 						/*!<GPIO port mode register,							    					Address offset: 0x00 */
  __vo uint32_t OTYPER; 					/*!<GPIO port output type register,						    					Address offset: 0x04 */
  __vo uint32_t OSPEEDER; 					/*!<GPIO port output speed register,						    				Address offset: 0x08 */
  __vo uint32_t PUPDR; 						/*!<GPIO port pull-up/pull-down register,				 	    				Address offset: 0x0C */
  __vo uint32_t IDR; 						/*!<GPIO port input data register, 						    					Address offset: 0x10 */
  __vo uint32_t ODR; 						/*!<GPIO port output data register,                      			    		Address offset: 0x14 */
  __vo uint32_t BSRR; 						/*!<GPIO port bit set/reset register,                    		            	Address offset: 0x18 */
  __vo uint32_t LCKR; 						/*!<GPIO port configuration lock register,               			   			Address offset: 0x1C */
  __vo uint32_t AFR[2]; 					/*!<GPIO alternate function low register (AFRL): AFR[0], 		            	Address offset: 0x20 */
											/*!<GPIO alternate function high register (AFRH): AFR[1],		 	    		Address offset: 0x24 */
} GPIO_RegDef_t;


/**
  * @brief Power Control
  */

typedef struct
{
  __vo uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __vo uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_RegDef_t;


/**
  * @brief FLASH Registers
  */
typedef struct
{
  __vo uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  __vo uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  __vo uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  __vo uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  __vo uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  __vo uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  __vo uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_RegDef_t;



/**
  * @brief TIM
  */

typedef struct
{
  __vo uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __vo uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __vo uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __vo uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __vo uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __vo uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __vo uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __vo uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __vo uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __vo uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __vo uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __vo uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __vo uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __vo uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __vo uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __vo uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __vo uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __vo uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __vo uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __vo uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __vo uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_RegDef_t;





/*
 * This section gives a detailed description of the RCC registers.
 */
typedef struct
{
  __vo uint32_t CR; 						/*!<RCC clock control register,							    					Address offset: 0x00 */
  __vo uint32_t PLLCFGR; 					/*!<RCC PLL configuration register,						    					Address offset: 0x04 */
  __vo uint32_t CFGR; 						/*!<RCC clock configuration register,					 	    				Address offset: 0x08 */
  __vo uint32_t CIR; 						/*!<RCC clock interrupt register,	        			 	    				Address offset: 0x0C */
  __vo uint32_t AHB1RSTR; 					/*!<RCC AHB1 peripheral reset register,						    				Address offset: 0x10 */
  __vo uint32_t AHB2RSTR; 					/*!<RCC AHB2 peripheral reset register,                  			   			Address offset: 0x14 */
  __vo uint32_t AHB3RSTR; 					/*!<RCC AHB3 peripheral reset register,                  			    		Address offset: 0x18 */
  uint32_t RESERVED0; 						/*!<Reserved 0x1C, 											  										 */
  __vo uint32_t APB1RSTR; 					/*!<RCC APB1 peripheral reset register,           		 		    			Address offset: 0x20 */
  __vo uint32_t APB2RSTR; 					/*!<RCC APB2 peripheral reset register,					 	    				Address offset: 0x24 */
  uint32_t RESERVED1[2]; 					/*!<Reserved : RESERVED1[0] 0x28 									 								 */
											/*!<Reserved : RESERVED1[1] 0x2C 									 								 */
  __vo uint32_t AHB1ENR; 					/*!<RCC AHB1 peripheral clock enable register       	 			    		Address offset: 0x30 */
  __vo uint32_t AHB2ENR; 					/*!<RCC AHB2 peripheral clock enable register       	 			    		Address offset: 0x34 */
  __vo uint32_t AHB3ENR; 					/*!<RCC AHB3 peripheral clock enable register			 		    			Address offset: 0x38 */
  uint32_t RESERVED2; 						/*!<Reserved 0x3C 									                 								 */
  __vo uint32_t APB1ENR; 					/*!<RCC APB1 peripheral clock enable register 			 		    			Address offset: 0x40 */
  __vo uint32_t APB2ENR; 					/*!<RCC APB2 peripheral clock enable register 			 		    			Address offset: 0x44 */
  uint32_t RESERVED3[2]; 					/*!<Reserved : RESERVED3[0] 0x48 									 								 */
											/*!<Reserved : RESERVED3[1] 0x4C 									 								 */
  __vo uint32_t AHB1LPENR; 					/*!<RCC AHB1 peripheral clock enable in low power mode register,		    	Address offset: 0x50 */
  __vo uint32_t AHB2LPENR; 					/*!<RCC AHB2 peripheral clock enable in low power mode register,		    	Address offset: 0x54 */
  __vo uint32_t AHB3LPENR; 					/*!<RCC AHB3 peripheral clock enable in low power mode register,		    	Address offset: 0x58 */
  uint32_t RESERVED4; 						/*!<Reserved 0x5C 											  										 */
  __vo uint32_t APB1LPENR; 					/*!<RCC APB1 peripheral clock enable in low power mode register			    	Address offset: 0x60 */
  __vo uint32_t APB2LPENR; 					/*!<RCC APB2 peripheral clock enabled in low power mode register		    	Address offset: 0x60 */
  uint32_t RESERVED5[2]; 					/*!<Reserved : RESERVED5[0] 0x68 				    				         						 */
											/*!<Reserved : RESERVED5[1] 0x6C 					 			         							 */
  __vo uint32_t BDCR; 						/*!<RCC Backup domain control register					 	    				Address offset: 0x70 */
  __vo uint32_t CSR; 						/*!<RCC clock control & status register						    				Address offset: 0x74 */
  uint32_t RESERVED6[2]; 					/*!<Reserved : RESERVED6[0] 0x78 									 								 */
											/*!<Reserved : RESERVED6[1] 0x7C 																	 */
  __vo uint32_t SSCGR; 						/*!<RCC spread spectrum clock generation register				    			Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR; 				/*!<RCC PLLI2S configuration register						    				Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR; 				/*!<RCC PLL configuration register						    					Address offset: 0x88 */
  __vo uint32_t DCKCFGR; 					/*!<RCC dedicated clock configuration register					   	 			Address offset: 0x8C */
  __vo uint32_t CKGATENR; 					/*!<RCC clocks gated enable register						    				Address offset: 0x90 */
  __vo uint32_t DCKCFGR2; 					/*!<RCC dedicated clocks configuration register 2				    			Address offset: 0x94 */
} RCC_RegDef_t;
/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
  __vo uint32_t IMR; 						/*!<Interrupt mask register,							    					Address offset: 0x00 */
  __vo uint32_t EMR; 						/*!<Event mask register,							    						Address offset: 0x04 */
  __vo uint32_t RTSR; 						/*!<Rising trigger selection register,						    				Address offset: 0x08 */
  __vo uint32_t FTSR; 						/*!<Falling trigger selection register,				 		    				Address offset: 0x0C */
  __vo uint32_t SWIER; 						/*!<Software interrupt event register, 						    				Address offset: 0x10 */
  __vo uint32_t PR; 						/*!<Pending register,                      					    				Address offset: 0x14 */
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
  __vo uint32_t MEMRMP;						/*!<SYSCFG memory remap register,					            				Address offset: 0x00 */
  __vo uint32_t PMC;						/*!<SYSCFG peripheral mode configuration register,				  				Address offset: 0x04 */
  __vo uint32_t EXTICR[4]; 					/*!<SYSCFG external interrupt configuration register EXTICR1: EXTICR[0] 	    Address offset: 0x08 */
											/*!<SYSCFG external interrupt configuration register EXTICR2: EXTICR[1] 	    Address offset: 0x0C */
											/*!<SYSCFG external interrupt configuration register EXTICR3: EXTICR[2]         Address offset: 0x10 */
											/*!<SYSCFG external interrupt configuration register EXTICR4: EXTICR[3] 	    Address offset: 0x14 */
  __vo uint32_t CMPCR; 						/*!<Compensation cell control register,				 							Address offset: 0x0C */
  __vo uint32_t CFGR; 						/*!<SYSCFG configuration register, 				         		       			Address offset: 0x10 */
} SYSCFG_RegDef_t;
/*************************************************************************************************************************************************/
/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 */
#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define PWR                					((PWR_RegDef_t*) PWR_BASE)
#define FLASH               				((FLASH_RegDef_t*) FLASH_R_BASE)
/**************************************************************************************************************************************************
 *															    Clock Enable Macros
 *************************************************************************************************************************************************/
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN() 					( RCC->AHB1ENR |= ( 1 << 7 ) )
/*
 * Clock Enable MAcros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 21 ) )

/*
 * Clock Enable MAcros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 14 ) )

/**************************************************************************************************************************************************
 *																 Clock Disable Macros
 *************************************************************************************************************************************************/
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI() 					( RCC->AHB1ENR &= ~( 1 << 7 ) )

/*
 * Clock Disable MAcros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 14 ) )


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 0 ) );	( RCC->AHB1RSTR &= ~( 1 << 0 ) ); }while(0)
#define GPIOB_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 1 ) );	( RCC->AHB1RSTR &= ~( 1 << 1 ) ); }while(0)
#define GPIOC_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 2 ) );	( RCC->AHB1RSTR &= ~( 1 << 2 ) ); }while(0)
#define GPIOD_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 3 ) );	( RCC->AHB1RSTR &= ~( 1 << 3 ) ); }while(0)
#define GPIOE_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 4 ) );	( RCC->AHB1RSTR &= ~( 1 << 4 ) ); }while(0)
#define GPIOF_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 5 ) );	( RCC->AHB1RSTR &= ~( 1 << 5 ) ); }while(0)
#define GPIOG_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 6 ) );	( RCC->AHB1RSTR &= ~( 1 << 6 ) ); }while(0)
#define GPIOH_REG_RESET()					do{ ( RCC->AHB1RSTR |= ( 1 << 7 ) );	( RCC->AHB1RSTR &= ~( 1 << 7 ) ); }while(0)

/*
 * This macro return a code(between 0 and 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)			(( x == GPIOA )? 0:\
											( x == GPIOB )? 1:\
											( x == GPIOC )? 2:\
											( x == GPIOD )? 3:\
											( x == GPIOE )? 4:\
											( x == GPIOF )? 5:\
											( x == GPIOG )? 6:\
									        ( x == GPIOH )? 7:0)
/*
 * IRQ(Interrupt request) number
 */
#define IRQ_NO_EXTI0						6
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRIO						0
#define NVIC_IRQ_PRI1						1
#define NVIC_IRQ_PRI2						2
#define NVIC_IRQ_PRI3						3
#define NVIC_IRQ_PRI4						4
#define NVIC_IRQ_PRI5						5
#define NVIC_IRQ_PRI6						6
#define NVIC_IRQ_PRI7						7
#define NVIC_IRQ_PRI8						8
#define NVIC_IRQ_PRI9						9
#define NVIC_IRQ_PRI10						10
#define NVIC_IRQ_PRI11						11
#define NVIC_IRQ_PRI12						12
#define NVIC_IRQ_PRI13						13
#define NVIC_IRQ_PRI14						14
#define NVIC_IRQ_PRI15						15

/**************************************************************************************************************************************************
 *																	Generic Macros
 *************************************************************************************************************************************************/
#define ENABLE 								1
#define DISABLE 							0
#define SET 								ENABLE
#define RESET			 					DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET 						RESET
#define FLAG_RESET							RESET
#define FLAG_SET							SET


/*************************************************************************************************************************************************/
#include <gpio_drivers.h>
/*************************************************************************************************************************************************/

#endif /* INC_STM32F446RE_H_ */
