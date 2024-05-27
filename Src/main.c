/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "Interface/Include/capture_timer_interface.h"
#include "Interface/Include/gpio_interface.h"
#include "Interface/Include/uart_interface.h"
#include "Interface/Include/motor_control_interface.h"
#include "OS/Include/os_tasks.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define usTIM TIM4

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define UART_BUFFER_SIZE 200

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId UartHandle;
osThreadId MotorControlHandle;
osThreadId FrontUltrasonicHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void const * argument);
void UartTask(void const * argument);
void MotorControlTask(void const * argument);
void FrontUltrasonicSensorTask(void const * argument);

/* USER CODE BEGIN PFP */

void usDelay(uint32_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

SemaphoreHandle_t mutex_frontSensor;

const float speed_of_sound = 0.0343 / 2;
int distanceFrontSensor;

uint8_t  icFlag                 = 0;
uint8_t  captureIdx             = 0;
uint32_t firstEdgeTime          = 0;
uint32_t secondEdgeTime         = 0;
uint16_t right_motor_value      = 0;
uint16_t left_motor_value       = 0;
uint8_t  turn_value             = 0;
uint8_t  direction_left_front   = FORWARD;
uint8_t  direction_right_front  = FORWARD;
uint8_t  direction_left_rear    = FORWARD;
uint8_t  direction_right_rear   = FORWARD;

char     uartBuf[UART_BUFFER_SIZE];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

//  distanceRightSensor = (int*)malloc(sizeof(int));
//  if (distanceRightSensor == NULL)
//  {
//	  Error_Handler();
//  }
  distanceFrontSensor = 0;

  mutex_frontSensor = xSemaphoreCreateMutex();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Uart */
  osThreadDef(Uart, UartTask, osPriorityNormal, 0, 128);
  UartHandle = osThreadCreate(osThread(Uart), NULL);

  /* definition and creation of MotorControl */
  osThreadDef(MotorControl, MotorControlTask, osPriorityNormal, 0, 128);
  MotorControlHandle = osThreadCreate(osThread(MotorControl), NULL);

  /* definition and creation of FrontUltrasonic */
  osThreadDef(FrontUltrasonic, FrontUltrasonicSensorTask, osPriorityNormal, 0, 128);
  FrontUltrasonicHandle = osThreadCreate(osThread(FrontUltrasonic), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//    HAL_Delay(5000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 80-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|Trigger_Pin|IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4REAR_Pin|IN1REAR_Pin|IN2REAR_Pin|IN3REAR_Pin
                          |IN4_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin Trigger_Pin IN1_Pin */
  GPIO_InitStruct.Pin = LED_Pin|Trigger_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4REAR_Pin IN1REAR_Pin IN2REAR_Pin IN3REAR_Pin
                           IN4_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN4REAR_Pin|IN1REAR_Pin|IN2REAR_Pin|IN3REAR_Pin
                          |IN4_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_Pin */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void usDelay(uint32_t us)
{
	if (us < 2)
	{
		us = 2;
	}

	usTIM->ARR = us - 1; /* auto-reload register */
	usTIM->EGR = 1;      /* re-initialize timer  */
	usTIM->SR &= ~1;     /* reset flag           */
	usTIM->CR1 |= 1;     /* enable counter       */
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	toogle_gpio_pin(LED_GPIO_Port, LED_Pin); /* we entered the callback */

	if (0 == captureIdx) /* rising edge */
	{
		firstEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		captureIdx = 1;
	}

	else if (1 == captureIdx) /* falling edge */
	{
		secondEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		captureIdx = 0;
		icFlag = 1; /* we finished capturing the time */
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UartTask */
/**
* @brief Function implementing the Uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartTask */
void UartTask(void const * argument)
{
  /* USER CODE BEGIN UartTask */
  /* Infinite loop */
  for(;;)
  {
	HandleUartTask(&huart2, (uint8_t*)uartBuf, sizeof(uartBuf), 100);
	HAL_Delay(50);
    osDelay(10);
  }
  /* USER CODE END UartTask */
}

/* USER CODE BEGIN Header_MotorControlTask */
/**
* @brief Function implementing the MotorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTask */
void MotorControlTask(void const * argument)
{
  /* USER CODE BEGIN MotorControlTask */
  int value_for_front_distance_sensor = 0;
  /* Infinite loop */
  for(;;)
  {
	  /* DC motor */
	  xSemaphoreTake(mutex_frontSensor, portMAX_DELAY);

	  value_for_front_distance_sensor = distanceFrontSensor;

	  xSemaphoreGive(mutex_frontSensor);

	  if ( value_for_front_distance_sensor > STOP_LIMIT )
	  {
		  turn_value = GO;

		  set_turn_values(turn_value, &right_motor_value, &left_motor_value, &direction_left_front, &direction_right_front, &direction_left_rear, &direction_right_rear);

		  set_direction(IN1_GPIO_Port, IN1_Pin, IN2_GPIO_Port, IN2_Pin, direction_right_front);
		  set_direction(IN3_GPIO_Port, IN3_Pin, IN4_GPIO_Port, IN4_Pin, direction_left_front);

		  set_direction(IN1REAR_GPIO_Port, IN1REAR_Pin, IN2REAR_GPIO_Port, IN2REAR_Pin, direction_left_rear);
		  set_direction(IN3REAR_GPIO_Port, IN3REAR_Pin, IN4REAR_GPIO_Port, IN4REAR_Pin, direction_right_rear);

		  TIM2->CCR3 = right_motor_value;  /* ARR = 1999 AND DUTY CYCLE = CCR / (ARR + 1)  AND CCR*/
		  TIM8->CCR3 = left_motor_value;

		  TIM2->CCR1 = left_motor_value;
		  TIM2->CCR2 = right_motor_value;
	  }
	  else if ( (value_for_front_distance_sensor > 0) && (value_for_front_distance_sensor < STOP_LIMIT) )
	  {
		  turn_value = TURN_RIGHT;

		  set_turn_values(turn_value, &right_motor_value, &left_motor_value, &direction_left_front, &direction_right_front, &direction_left_rear, &direction_right_rear);

		  set_direction(IN1_GPIO_Port, IN1_Pin, IN2_GPIO_Port, IN2_Pin, direction_right_front);
		  set_direction(IN3_GPIO_Port, IN3_Pin, IN4_GPIO_Port, IN4_Pin, direction_left_front);

		  set_direction(IN1REAR_GPIO_Port, IN1REAR_Pin, IN2REAR_GPIO_Port, IN2REAR_Pin, direction_left_rear);
		  set_direction(IN3REAR_GPIO_Port, IN3REAR_Pin, IN4REAR_GPIO_Port, IN4REAR_Pin, direction_right_rear);

		  TIM2->CCR3 = right_motor_value;
		  TIM8->CCR3 = left_motor_value;

		  TIM2->CCR1 = left_motor_value;
		  TIM2->CCR2 = right_motor_value;

		  HAL_Delay(1000);
	  }
	  else
	  {
		  set_turn_values(turn_value, &right_motor_value, &left_motor_value, &direction_left_front, &direction_right_front, &direction_left_rear, &direction_right_rear);

		  set_direction(IN1_GPIO_Port, IN1_Pin, IN2_GPIO_Port, IN2_Pin, direction_right_front);
		  set_direction(IN3_GPIO_Port, IN3_Pin, IN4_GPIO_Port, IN4_Pin, direction_left_front);

		  set_direction(IN1REAR_GPIO_Port, IN1REAR_Pin, IN2REAR_GPIO_Port, IN2REAR_Pin, direction_left_rear);
		  set_direction(IN3REAR_GPIO_Port, IN3REAR_Pin, IN4REAR_GPIO_Port, IN4REAR_Pin, direction_right_rear);

		  TIM2->CCR3 = right_motor_value;
		  TIM8->CCR3 = left_motor_value;

		  TIM2->CCR1 = left_motor_value;
		  TIM2->CCR2 = right_motor_value;
	  }

	  /* front */
	  pwm_start(&htim2, TIM_CHANNEL_3);
	  pwm_start(&htim8, TIM_CHANNEL_3);

	  /* rear */
	  pwm_start(&htim2, TIM_CHANNEL_1);
	  pwm_start(&htim2, TIM_CHANNEL_2);

	  HAL_Delay(50);

	  osDelay(10);
  }
  /* USER CODE END MotorControlTask */
}

/* USER CODE BEGIN Header_FrontUltrasonicSensorTask */
/**
* @brief Function implementing the FrontUltrasonic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FrontUltrasonicSensorTask */
void FrontUltrasonicSensorTask(void const * argument)
{
  /* USER CODE BEGIN FrontUltrasonicSensorTask */
  int calculated_distance_for_front_sensor;
  /* Infinite loop */
  for(;;)
  {
	  /* TRIGGER TO LOW */
	  write_gpio_pin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	  usDelay(5);

	  /* Start measurement by outputing a 10 us pulse on the trigger pin */
	  write_gpio_pin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	  usDelay(10);
	  write_gpio_pin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

	  /* Pulse width on echo signal */
	  /* we jump into capture timer callback whenever there is a rising edge or a falling edge as we set in the .ioc file*/
	  start_capture_timer(&htim3, TIM_CHANNEL_1);

	  uint32_t startTick = HAL_GetTick();

	  do
	  {
		if (0 != icFlag)
		{
			break;
		}
	  } while( (HAL_GetTick() - startTick) < 500 );  // 500ms timeout value if no capture

	  icFlag = 0; /* reset flag */
	  stop_capture_timer(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(50);

	  /* distance = time * sound_speed / 2 */

	  if (secondEdgeTime > firstEdgeTime)
	  {
		calculated_distance_for_front_sensor = (int)( (secondEdgeTime - firstEdgeTime + 0.0f) * speed_of_sound );
	  }

	  else
	  {
		calculated_distance_for_front_sensor = 0;
	  }

	  xSemaphoreTake(mutex_frontSensor, portMAX_DELAY);

	  distanceFrontSensor = calculated_distance_for_front_sensor;

	  xSemaphoreGive(mutex_frontSensor);

	  HAL_Delay(50);
	  sprintf(uartBuf, "Distance in cm is %d\n\r", + calculated_distance_for_front_sensor);

	  osDelay(10);
  }
  /* USER CODE END FrontUltrasonicSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
