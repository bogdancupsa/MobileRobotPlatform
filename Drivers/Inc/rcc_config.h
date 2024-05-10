/*
 * rcc_config.h
 *
 *  Created on: Apr 29, 2024
 *      Author: paul.contis
 */

#ifndef INC_RCC_CONFIG_H_
#define INC_RCC_CONFIG_H_

#include "stm32f446re.h"


/**  RCC_Oscillator_Type Oscillator Type
  */
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U


/**  RCC_HSI_Config HSI Config
  */
#define RCC_HSI_ON                       ((uint8_t)0x01)

#define RCC_HSICALIBRATION_DEFAULT       0x10U         /* Default HSI calibration trimming value */

/**  RCC_PLL_Config PLL Config
  */
#define RCC_PLL_ON                        ((uint8_t)0x02)


/** @defgroup RCC_PLL_Clock_Source PLL Clock Source
  * @{
  */
#define RCC_PLLSOURCE_HSI                RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE                RCC_PLLCFGR_PLLSRC_HSE

/** @defgroup RCC_PLLP_Clock_Divider PLLP Clock Divider
  * @{
  */
#define RCC_PLLP_DIV2                  0x00000002U
#define RCC_PLLP_DIV4                  0x00000004U
#define RCC_PLLP_DIV6                  0x00000006U
#define RCC_PLLP_DIV8                  0x00000008U


/** @defgroup RCC_System_Clock_Type System Clock Type
  * @{
  */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U
#define RCC_CLOCKTYPE_HCLK               0x00000002U
#define RCC_CLOCKTYPE_PCLK1              0x00000004U
#define RCC_CLOCKTYPE_PCLK2              0x00000008U


#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL





#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1

#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4


#define FLASH_LATENCY_5                FLASH_ACR_LATENCY_5WS   /*!< FLASH Five Latency cycles     */


typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} Status_t;




/**
  * @brief  RCC PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432
                            except for STM32F411xE devices where the Min_Data = 192 */

  uint32_t PLLP;       /*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */

  uint32_t PLLQ;       /*!< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */

  uint32_t PLLR;       /*!< PLLR: PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 7     */
}RCC_PLLInit_t;


typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */

  RCC_PLLInit_t PLL;       		 /*!< PLL structure parameters                                                    */
}RCC_OscInit_t;

/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

}RCC_ClkInit_t;




#define RCC_PWR_CLK_ENABLE()      do { __vo uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                       /* Delay after an RCC peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                       (void)tmpreg; } while(0U)

/** @brief  macros configure the main internal regulator output voltage.
  * @param  __REGULATOR__ specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption when the device does
  *         not operate at the maximum frequency (refer to the datasheets for more details).
  *          This parameter can be one of the following values:
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output Scale 3 mode
  * @retval None
  */
#define PWR_VOLTAGESCALING_CONFIG(__REGULATOR__) do {                                                     \
															__vo uint32_t tmpreg = 0x00U;                        \
                                                           /* MODIFY_REG(PWR->CR, PWR_CR_VOS, (__REGULATOR__)); */  \
                                                            /* Delay after an RCC peripheral clock enabling */  \
                                                          /*  tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);     */        \
                                                            (void)tmpreg;                                     \
                                                          } while(0U)


#define PWR_REGULATOR_VOLTAGE_SCALE1         PWR_CR_VOS             /* Scale 1 mode(default value at reset): the maximum value of fHCLK is 168 MHz. It can be extended to
                                                                       180 MHz by activating the over-drive mode. */






#endif /* INC_RCC_CONFIG_H_ */
