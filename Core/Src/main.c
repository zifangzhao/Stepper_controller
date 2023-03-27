/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CE32_stepper_motor_driver.h"
#include "FTL_Sampling.h"
#include "CE32_COMMAND.h"
#include "CE32_UART_INTERCOM.h"
#include "string.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CE32_stepMotor motor[4];
FTL_sampling sampler;
CE32_INTERCOM_Handle hCOMM;
int16_t motor_location[6];
int measure_state=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

void Motor_init(void);
int Vector_Control(int16_t *locs,int len);
int Measure_SendRst(void);
int MatrixScan(int16_t step,int16_t Xbase,int16_t Ybase,int16_t Zbase, int16_t Xspan,int16_t Yspan,int16_t Zspan);
int DepthScan(int16_t step,int16_t Xbase,int16_t Ybase,int16_t Zbase, int16_t Xspan,int16_t Yspan,int16_t Zspan);
void cmd_svr(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port,USB_PowerSwitchOn_Pin,1);
	Motor_init();
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	CE32_INTERCOM_Init(&hCOMM, &huart3);
	INTERCOM_UART_ENABLE(&hCOMM);
	INTERCOM_UART_RXIT_ENABLE(&hCOMM);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//Command Processing services
		cmd_svr();
	}
	//	Stepping_down_Distance(&motor[0], 50);
	//	Stepping_down_Distance(&motor[1], 50);
	//	Stepping_down_Distance(&motor[2], 50);
	//
	//	FTL_Sampling_Init(&sampler,1);
	//	HAL_TIM_Base_Start_IT(&htim16);
	//	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	//	while((sampler.state&FTL_SAMPLE_STATE_FINISH)==0);
	//	Stepping_up_Distance(&motor[0], 50);
	//	Stepping_up_Distance(&motor[1], 50);
	//	Stepping_up_Distance(&motor[2], 50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 3;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 71;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7199;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 10;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT|UART_ADVFEATURE_RXOVERRUNDISABLE_INIT
                              |UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, C3_Pin|BLE_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, A0_Pin|A1_Pin|A2_Pin|A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_PWR_GPIO_Port, BLE_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, B0_Pin|C0_Pin|USB_PowerSwitchOn_Pin|D2_Pin
                          |D0_Pin|D3_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, B1_Pin|B2_Pin|B3_Pin|C1_Pin
                          |C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : C3_Pin */
  GPIO_InitStruct.Pin = C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(C3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_GND_Pin BLE_PWR_Pin */
  GPIO_InitStruct.Pin = BLE_GND_Pin|BLE_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B0_Pin C0_Pin D2_Pin D0_Pin
                           D3_Pin D1_Pin */
  GPIO_InitStruct.Pin = B0_Pin|C0_Pin|D2_Pin|D0_Pin
                          |D3_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin B3_Pin C1_Pin
                           C2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin|B3_Pin|C1_Pin
                          |C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Motor_init(void)
{
	int OnPeriod=10;
	int PostPeriod=1;
	float step_angle=5.625/64;
	float distPerDeg=500.0/360;
	CE32_stepMotor_InitStruct init;
	init.Port_0=A0_GPIO_Port;
	init.Pin_0=A0_Pin;
	init.Port_1=A1_GPIO_Port;
	init.Pin_1=A1_Pin;
	init.Port_2=A2_GPIO_Port;
	init.Pin_2=A2_Pin;
	init.Port_3=A3_GPIO_Port;
	init.Pin_3=A3_Pin;
	init.Init_Pos=0;
	init.Int_Period=PostPeriod;
	init.On_Period=OnPeriod;
	init.step_angle=step_angle;
	init.distPerDegree=distPerDeg;
	CE32_stepMotor_Init(&motor[2],&init);

	init.Port_0=B0_GPIO_Port;
	init.Pin_0=B0_Pin;
	init.Port_1=B1_GPIO_Port;
	init.Pin_1=B1_Pin;
	init.Port_2=B2_GPIO_Port;
	init.Pin_2=B2_Pin;
	init.Port_3=B3_GPIO_Port;
	init.Pin_3=B3_Pin;
	init.Init_Pos=0;
	CE32_stepMotor_Init(&motor[1],&init);

	init.Port_0=C0_GPIO_Port;
	init.Pin_0=C0_Pin;
	init.Port_1=C1_GPIO_Port;
	init.Pin_1=C1_Pin;
	init.Port_2=C2_GPIO_Port;
	init.Pin_2=C2_Pin;
	init.Port_3=C3_GPIO_Port;
	init.Pin_3=C3_Pin;
	CE32_stepMotor_Init(&motor[0],&init);

	init.Port_0=D0_GPIO_Port;
	init.Pin_0=D0_Pin;
	init.Port_1=D1_GPIO_Port;
	init.Pin_1=D1_Pin;
	init.Port_2=D2_GPIO_Port;
	init.Pin_2=D2_Pin;
	init.Port_3=D3_GPIO_Port;
	init.Pin_3=D3_Pin;
	CE32_stepMotor_Init(&motor[3],&init);
}

int Vector_Control(int16_t *locs,int len)
{
	cmd_svr();
	if(len>4)
		len=4;
	for(int i=0;i<len;i++)
	{
		Stepping_To_Distance(&motor[i], locs[i]);
	}
	HAL_TIM_Base_Start_IT(&htim17); //Start motor ticking timer
	while(Stepping_CheckState(&motor[0])|Stepping_CheckState(&motor[1])|Stepping_CheckState(&motor[2])|Stepping_CheckState(&motor[3])!=0);//Wait all motor move to target location
	HAL_TIM_Base_Stop_IT(&htim17);
	memcpy(motor_location,locs,12);
	return 0;
}

int Measure_SendRst(void)
{
	FTL_Sampling_Init(&sampler,3);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	while(sampler.state!=0x02);
	uint8_t TX_cmd[2*SAMPLE_POINT+12+2];
	TX_cmd [0]=0x3C;
	memcpy(&TX_cmd[1],motor_location,12);
	memcpy(&TX_cmd[1+12],sampler.buf,2*SAMPLE_POINT);
	TX_cmd[2*SAMPLE_POINT+12+1]=0x3E;
	CE32_INTERCOM_TX_EnqueueCmd(&hCOMM, TX_cmd, 2*SAMPLE_POINT+12+2);
	return 0;
}


int MatrixScan(int16_t step,int16_t Xbase,int16_t Ybase,int16_t Zbase, int16_t Xspan,int16_t Yspan,int16_t Zspan)
{
	int16_t Initloc[4];
	int16_t Originloc[4];
	
	memcpy(Originloc,motor_location,4*2);
	Initloc[0]=Originloc[0]+Xbase;
	Initloc[1]=Originloc[1]+Ybase;
	Initloc[2]=Originloc[2]+Zbase;
	
	int ord[3]={2,0,1};
	int Nstep[3];
	Nstep[0]=Xspan/step;
	Nstep[1]=Yspan/step;
	Nstep[2]=Zspan/step;

	for(int x=0;x<=Nstep[ord[0]];x++)
	{
		motor_location[ord[0]]=Initloc[ord[0]]+x*step;;
		for(int y=0;y<=Nstep[ord[1]];y++)
		{
			motor_location[ord[1]]=Initloc[ord[1]]+y*step;
			for(int z=0;z<=Nstep[ord[2]];z++)
			{
				motor_location[ord[2]]=Initloc[ord[2]]+z*step;
				Vector_Control(motor_location,3);
				Measure_SendRst();
				if(measure_state==0)
					return -1;
			}
			
			if(++y<=Nstep[ord[1]])
			{
				motor_location[ord[1]]=Initloc[ord[1]]+y*step;
				for(int z=Nstep[ord[2]];z>=0;z--)
				{
					motor_location[ord[2]]=Initloc[ord[2]]+z*step;
					Vector_Control(motor_location,3);
					Measure_SendRst();
					if(measure_state==0)
						return -1;
				}
			}
		}
		if(++x<=Nstep[ord[0]])
		{
			motor_location[ord[0]]=Initloc[ord[0]]+x*step;;
			for(int y=Nstep[ord[1]];y>=0;y--)
			{
				motor_location[ord[1]]=Initloc[ord[1]]+y*step;
				for(int z=0;z<=Nstep[ord[2]];z++)
				{
					motor_location[ord[2]]=Initloc[ord[2]]+z*step;
					Vector_Control(motor_location,3);
					Measure_SendRst();
					if(measure_state==0)
							return -1;
				}
				
				if(--y>=0)
				{
					motor_location[ord[1]]=Initloc[ord[1]]+y*step;
					for(int z=Nstep[ord[2]];z>=0;z--)
					{
						motor_location[ord[2]]=Initloc[ord[2]]+z*step;
						Vector_Control(motor_location,3);
						Measure_SendRst();
						if(measure_state==0)
							return -1;
					}
				}
			}
		}
	}
	Vector_Control(Originloc,3); //Return to initial location
	return 0;
}

int DepthScan(int16_t step,int16_t Xbase,int16_t Ybase,int16_t Zbase, int16_t Xspan,int16_t Yspan,int16_t Zspan)
{
	int16_t Initloc[4];
	int16_t Originloc[4];
	
	memcpy(Originloc,motor_location,4*2);
	Initloc[0]=Originloc[0]+Xbase;
	Initloc[1]=Originloc[1]+Ybase;
	Initloc[2]=Originloc[2]+Zbase;
	
	int ord[3]={0,1,2};
	int Nstep[3];
	Nstep[0]=Xspan/step;
	Nstep[1]=Yspan/step;
	Nstep[2]=Zspan/step;

	
	for(int y=0;y<=Nstep[ord[1]];y++)
	{
		motor_location[ord[0]]=Initloc[ord[0]]-y*step;;
		motor_location[ord[1]]=Initloc[ord[1]]-y*step;
		for(int z=0;z<=Nstep[ord[2]];z++)
		{
			motor_location[ord[2]]=Initloc[ord[2]]+z*step;
			Vector_Control(motor_location,3);
			Measure_SendRst();
			if(measure_state==0)
				return -1;
		}
		
		if(++y<=Nstep[ord[1]])
		{
			motor_location[ord[0]]=Initloc[ord[0]]-y*step;;
			motor_location[ord[1]]=Initloc[ord[1]]-y*step;
			for(int z=Nstep[ord[2]];z>=0;z--)
			{
				motor_location[ord[2]]=Initloc[ord[2]]+z*step;
				Vector_Control(motor_location,3);
				Measure_SendRst();
				if(measure_state==0)
					return -1;
			}
		}
	}
	Vector_Control(Originloc,3); //Return to initial location
	return 0;
}

void cmd_svr(void)
{
	uint8_t *data_ptr;
	uint32_t cmd_len;
	if(CE32_INTERCOM_RX_DequeueCmd(&hCOMM,&data_ptr,&cmd_len)==0)
	{
		switch(data_ptr[0])
		{
			case 0xA0://set speed
			{
				htim17.Instance->ARR = *((uint16_t*)&data_ptr[1]); 
				uint8_t buf[1+1*2];
				buf[0]=0xA0;
				memcpy(&buf[1],&data_ptr[1],1*2);
				CDC_Transmit_FS(buf,sizeof(buf));
				break;
			}
			case 0xD0:
			{
				Vector_Control((int16_t*)&data_ptr[1],4);
				uint8_t buf[1+4*2];
				buf[0]=0xD0;
				memcpy(&buf[1],&data_ptr[1],4*2);
				CDC_Transmit_FS(buf,sizeof(buf));
				break;
			}
			case 0xD1:
			{
				Vector_Control((int16_t*)&data_ptr[1],4);
				Measure_SendRst();
				break;
			}
			case 0xD2:
			{
				int16_t *data=(int16_t*)&data_ptr[1];
				measure_state=1; //Setup measurement flag
				MatrixScan(data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
				break;
			}
			case 0xD3:
			{
				measure_state=0;//Send stop measurement flag
				break;
			}
			case 0xD4:
			{
				int16_t *data=(int16_t*)&data_ptr[1];
				measure_state=1; //Setup measurement flag
				DepthScan(data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
				break;
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
