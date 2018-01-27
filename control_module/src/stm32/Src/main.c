/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "lcd_init.h"
#include "xpt2046.h"
#include "GUI.h"
#include "windows.h"
#include "wake.h"
#include "OWIHighLevelFunctions.h"
#include "settings.h"
#include "beeper.h"
#include "valve.h"
#include "eeprom.h"
#include "cpu_utils.h"
#include "wifi.h"
#include "math.h"
#include "uart_log.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

SRAM_HandleTypeDef hsram1;

osThreadId mainTaskHandle;
osThreadId touchTaskHandle;
osThreadId sensPollTaskHandle;
osThreadId runProcessHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint8_t isCallibrated = 0;
uint8_t OW_Status;
OWI_device devices[MAX_DS18B20];
uint8_t sens_count = 0;
uint16_t wakeMaster_timeout;
uint16_t pressure_capture;
uint16_t water_capture;
uint8_t stateMode, subMode; // Режим и подрежим работы
uint8_t touchIsCallibrated = 0;

SPid pid; // ПИД для регулятора давления в кубе

xSemaphoreHandle InitTempSen;
xSemaphoreHandle shMode;
xSemaphoreHandle shPauseWifiSend;


// debug
#if DEBUG == 1
char d_buf[6];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
void StartMainTask(void const * argument);
void StartTouchTask(void const * argument);
void StartSensPollTask(void const * argument);
void StartRunProcess(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3) {
			WakeMaster_UART_Rx();	
	}
	if (huart == &huart4) {
			WifiOnRx();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3) 
	{
			WakeMaster_UART_Tx();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2) {
		// DEBUG
		#if DEBUG == 1
			sprintf(d_buf, "Err:%i", huart->ErrorCode); 
			LogPrintE("1W", d_buf);
		#endif
		//----------------------------------------------------
		
		// Необходимость данного "костыля" не доказана :)
		if (huart->ErrorCode == HAL_UART_ERROR_ORE){
        // remove the error condition
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        // set the correct state, so that the UART_RX_IT works correctly
        huart->RxState = HAL_UART_STATE_BUSY_RX;
    }
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
	LogInit(&huart1);
	WakeMaster_Init(&huart3);
	OW_Init(&huart2);
	EepromInit(&hi2c1);
	WifiInit(&huart4);
	GUI_Init();
	//GUI_X_InitOS();
//	vTraceEnable(TRC_INIT);
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

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, StartMainTask, osPriorityNormal, 0, 1024); // 1024
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of touchTask */
  osThreadDef(touchTask, StartTouchTask, osPriorityNormal, 0, 160);
  touchTaskHandle = osThreadCreate(osThread(touchTask), NULL);

  /* definition and creation of sensPollTask */
  osThreadDef(sensPollTask, StartSensPollTask, osPriorityAboveNormal, 0, 160); // 128
  sensPollTaskHandle = osThreadCreate(osThread(sensPollTask), NULL);

  /* definition and creation of runProcess */
  osThreadDef(runProcess, StartRunProcess, osPriorityNormal, 0, 160);
  runProcessHandle = osThreadCreate(osThread(runProcess), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	#if DEBUG == 1
		//LogPrintI("Main:", "OS started...");
	#endif
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 500;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 2;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 400;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 200;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 41999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Power_brd_en_Pin|GPO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Valve1_gate_Pin|Valve2_gate_Pin|Valve3_gate_Pin|Beeper_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Valve4_gate_Pin|Debug_LED_Pin|SPI2_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Power_brd_en_Pin GPO1_Pin */
  GPIO_InitStruct.Pin = Power_brd_en_Pin|GPO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Power_brd_rdy_Pin GPI1_Pin GPI2_Pin */
  GPIO_InitStruct.Pin = Power_brd_rdy_Pin|GPI1_Pin|GPI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Valve1_gate_Pin Valve2_gate_Pin Valve3_gate_Pin Beeper_Pin */
  GPIO_InitStruct.Pin = Valve1_gate_Pin|Valve2_gate_Pin|Valve3_gate_Pin|Beeper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP8622_Boot_GPIO0_Pin ESP8622_GPIO2_Pin */
  GPIO_InitStruct.Pin = ESP8622_Boot_GPIO0_Pin|ESP8622_GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;// GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Valve4_gate_Pin Debug_LED_Pin SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = Valve4_gate_Pin|Debug_LED_Pin|SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Spirt_concentr_IRQ_Pin */
  GPIO_InitStruct.Pin = Spirt_concentr_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Spirt_concentr_IRQ_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 8;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 4;
  Timing.BusTurnAroundDuration = 8;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartMainTask function */
void StartMainTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	uint8_t retCode = 0;
	GUI_HWIN hWnd;
	WM_SetDesktopColor(GUI_BLACK);
	GUI_SetBkColor(GUI_BLACK);
	GUI_SetDrawMode(GUI_DM_NORMAL);
	GUI_SetFont(&GUI_Font20_1);
	GUI_Clear();
	InitTempSen = xSemaphoreCreateBinary();
	shMode = xSemaphoreCreateMutex();
	shPauseWifiSend = xSemaphoreCreateMutex();
	// Init GPIO
	osDelay(100);
	GUI_SetColor(GUI_WHITE);
	GUI_DispStringAt("-> Init GPIO", 10, 20);
	GUI_SetColor(GUI_GREEN);
	GUI_DispStringAt("[OK]", 400, 20);
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start_IT(&htim6);
	osDelay(100);
	// Init USARTs
	osDelay(100);
	GUI_SetColor(GUI_WHITE);
	GUI_DispStringAt("-> Init USARTs", 10, 40);
	GUI_SetColor(GUI_GREEN);
	GUI_DispStringAt("[OK]", 400, 40);
	//InitRegs();
	osDelay(100);
	// Init Wake exchange protocol
	GUI_SetColor(GUI_WHITE);
	GUI_DispStringAt("-> Init Wake exchange protocol", 10, 60);
	GUI_SetColor(GUI_GREEN);
	GUI_DispStringAt("[OK]", 400, 60);

	osDelay(100);
	// Init 1-Wire
	osDelay(100);
	GUI_SetColor(GUI_WHITE);
	GUI_DispStringAt("-> Init 1-Wire", 10, 80);
	xSemaphoreGive(InitTempSen);
	GUI_SetColor(GUI_GREEN);
	GUI_DispStringAt("[OK]", 400, 80);
		
	osDelay(100);
	GUI_SetColor(GUI_WHITE);
	GUI_DispStringAt("-> Loading settings...", 10, 100);
#ifndef  RESET_EEPROM
	if (CheckSettings() != EEPROM_VER_EQ){
		LoadDefSettings();
		TouchCalibrate3Points(&hspi1);
		TouchGetCoeff(&Settings.lcd_conf.cali_A, &Settings.lcd_conf.cali_B, &Settings.lcd_conf.cali_C, &Settings.lcd_conf.cali_D,
				&Settings.lcd_conf.cali_E, &Settings.lcd_conf.cali_F);
		SaveSettings();
		GUI_SetBkColor(GUI_BLACK);
		GUI_SetDrawMode(GUI_DM_NORMAL);
		GUI_SetFont(&GUI_Font20_1);
		GUI_Clear();
	} else {
		LoadSettings();
		TouchSetCoeff(Settings.lcd_conf.cali_A, Settings.lcd_conf.cali_B, Settings.lcd_conf.cali_C, Settings.lcd_conf.cali_D,
				Settings.lcd_conf.cali_E, Settings.lcd_conf.cali_F);
	}
#else
	LoadDefSettings();
	TouchCalibrate3Points(&hspi1);
	TouchGetCoeff(&Settings.lcd_conf.cali_A, &Settings.lcd_conf.cali_B, &Settings.lcd_conf.cali_C, &Settings.lcd_conf.cali_D,
				&Settings.lcd_conf.cali_E, &Settings.lcd_conf.cali_F);
	SaveSettings();
	GUI_SetBkColor(GUI_BLACK);
	GUI_SetDrawMode(GUI_DM_NORMAL);
	GUI_SetFont(&GUI_Font20_1);
	GUI_Clear();
	
#endif
	touchIsCallibrated = 1;
	
	GUI_SetColor(GUI_GREEN);
	GUI_DispStringAt("[OK]", 400, 100);
	// Init registers
	osDelay(100);
	GUI_SetColor(GUI_WHITE);
	GUI_DispStringAt("-> Init registers", 10, 120);
	InitRegs();
	GUI_SetColor(GUI_GREEN);
	GUI_DispStringAt("[OK]", 400, 120);
	osDelay(1000);
	
	
	stateMode = MODE_NOTHING;
	
	//GUI_CURSOR_Show();
	BUTTON_SetDefaultSkinClassic();
  /* Infinite loop */
  for(;;)
  {
    switch (retCode){ 
			case SCR_MAIN:
					hWnd = CreateMainWindow(); // Главное меню
					retCode = GUI_ExecCreatedDialog(hWnd);
				break;
			case SCR_TEST_SEL:
					hWnd = CreateSelTests(); // Выбор тестов
					retCode = GUI_ExecCreatedDialog(hWnd);
				break;
			case SCR_TEST_EM:
					hWnd = CreateEmTests(); // Тест ИМ
					retCode = GUI_ExecCreatedDialog(hWnd);
				break;
			case SCR_TEST_SEN:
					SetStateMode(MODE_TEST_SEN);
					hWnd = CreateSenTests(); // Тест сенсоров
					retCode = GUI_ExecCreatedDialog(hWnd);
					SetStateMode(MODE_NOTHING);
				break;
			case SCR_SETTINGS_SEL:
					hWnd = CreateSelSettings(); // Выбор настроек
					if (shPauseWifiSend != NULL) {
						if(xSemaphoreTake(shPauseWifiSend, portMAX_DELAY) == pdTRUE){
							HAL_GPIO_WritePin(Debug_LED_GPIO_Port, Debug_LED_Pin, GPIO_PIN_SET);
							retCode = GUI_ExecCreatedDialog(hWnd);
							xSemaphoreGive(shPauseWifiSend);
							HAL_GPIO_WritePin(Debug_LED_GPIO_Port, Debug_LED_Pin, GPIO_PIN_RESET);
						}
					} 
				break;
			case SCR_SETTINGS_PM:
					hWnd = CreateSettingsPmWnd(); // Настройки силового блока
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_SETTINGS_TS:
					SetStateMode(MODE_INIT_SEN);
					hWnd = CreateSettingsTempWnd(); // Настройки датчиков температуры
					retCode = GUI_ExecCreatedDialog(hWnd);
					SetStateMode(MODE_NOTHING);
				break;
			case SCR_SETTINGS_SEL_CAL:
					hWnd = CreateSelCalibrations(); // Выбор калибровок
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_SETTINGS_PSC:
					hWnd = CreatePSenWnd(); // Калибровка датчика давления 
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_SETTINGS_VC:
					hWnd = CreateCAlibrationValveWnd(); // Калибровка клапана
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_SETTINGS_DIS:
					hWnd = CreateDistSetWnd(); // Настройки дистилляции
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_SETTINGS_RECT:
					hWnd = CreateRectSetWnd(); // Настройки ректификации
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_RUN_CLEAR:
					hWnd = CreateRunClrWnd(); // Процесс чистки колоны 
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_RUN_DIS:
					hWnd = CreateRunDistWnd(); // Процесс дистилляции 
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			case SCR_RUN_RECT:
					hWnd = CreateRunRectWnd(); // Процесс ректификации 
					retCode = GUI_ExecCreatedDialog(hWnd); 
				break;
			default:
					hWnd = CreateMainWindow();
					retCode = GUI_ExecCreatedDialog(hWnd);
				break;
		}
		if (hWnd != NULL) {
			WM_DeleteWindow(hWnd);
		}
		//GUI_Exec();
		GUI_Delay(150);

  }
  /* USER CODE END 5 */ 
}

/* StartTouchTask function */
void StartTouchTask(void const * argument)
{
  /* USER CODE BEGIN StartTouchTask */
	uint32_t x, y;
	uint8_t rw_flag = 0;
  GUI_PID_STATE pid;
	wakeMaster_timeout = 0;
  /* Infinite loop */
  for(;;)
  {		
		if (!WakeMaster_Busy()) {
			if (WakeMaster_Timeout()) {
				wakeMaster_timeout ++;
			}
			if (!rw_flag) {
					WakeMaster_GetParams(0, 16); // read RO registers
			} else {
					WakeMaster_SetParams(16, 16); // write RW registers
			}
			rw_flag = ~rw_flag;
		} 
		WakeMaster_Commands_Exe();
		
		if (!touchIsCallibrated)
			{
				osDelay(25);	
				continue;
			}	
			
		if (getTouchState()) {
				pid.Pressed = 1;
				//HAL_GPIO_WritePin(Debug_LED_GPIO_Port, Debug_LED_Pin, GPIO_PIN_SET);
			} else {
				pid.Pressed = 0;
				//HAL_GPIO_WritePin(Debug_LED_GPIO_Port, Debug_LED_Pin, GPIO_PIN_RESET);
			}
			// read coords
			touchGetSense(&hspi1, &x, &y);
			pid.x = x;
			pid.y = y;
		  pid.Layer = 0;
			GUI_TOUCH_StoreStateEx (&pid);		
			osDelay(25);
  }
  /* USER CODE END StartTouchTask */
}

/* StartSensPollTask function */
void StartSensPollTask(void const * argument)
{
  /* USER CODE BEGIN StartSensPollTask */
	uint8_t num = 0;
	//uint8_t flag = 0;
	float pressure = 0.0;

  /* Infinite loop */
  for(;;)
  {
    if (InitTempSen != NULL) {
			if (xSemaphoreTake(InitTempSen, portTICK_PERIOD_MS * 2 ) == pdTRUE) {
				//OW_Init(&huart2);
				if (OWI_SearchDevices(devices, MAX_DS18B20, &num) == SEARCH_SUCCESSFUL){
					sens_count = num;
					//osDelay(100);
					switch (num){
						/*case 4:
							InitSensor(devices[3].id, 0, 100, DS18B20_12BIT_RES);
							osDelay(10);*/
						case 3:
							InitSensor(devices[2].id, 0, 100, DS18B20_12BIT_RES);

						case 2:
							InitSensor(devices[1].id, 0, 100, DS18B20_12BIT_RES);
							
						case 1:
							InitSensor(devices[0].id, 0, 100, DS18B20_12BIT_RES);	
					}
				}
			}
		}
		if (OWI_DetectPresence() == OW_OK ) {
			switch (GetStateMode()) {
				case MODE_INIT_SEN:
						//if (num > 0 && OWI_DetectPresence() == OW_OK) {
						if (num != 0) {
							Sensors.t_kub = GetTemperatureSkipRom();
						}
					break;
				case MODE_TEST_SEN:
						Sensors.t_kub = GetTemperatureMatchRom(Settings.id_t_kub.id);
						Sensors.t_kolona_n = GetTemperatureMatchRom(Settings.id_t_kolona_n.id);
						Sensors.t_kolona_v = GetTemperatureMatchRom(Settings.id_t_kolona_v.id);
						//Sensors.t_after_def = GetTemperatureMatchRom(Settings.id_t_after_def.id);
					break;
				case MODE_RUN_CLS:
						Sensors.t_kub = ExpFilter(GetTemperatureMatchRom(Settings.id_t_kub.id), Sensors.t_kub, K_EXP);
					break;
				case MODE_RUN_DIST:
						Sensors.t_kub = ExpFilter(GetTemperatureMatchRom(Settings.id_t_kub.id), Sensors.t_kub, K_EXP);
					break;
				case MODE_RUN_RECT:
						Sensors.t_kub = ExpFilter(GetTemperatureMatchRom(Settings.id_t_kub.id), Sensors.t_kub, K_EXP);
						Sensors.t_kolona_n = ExpFilter(GetTemperatureMatchRom(Settings.id_t_kolona_n.id), Sensors.t_kolona_n, K_EXP);
						Sensors.t_kolona_v = ExpFilter(GetTemperatureMatchRom(Settings.id_t_kolona_v.id), Sensors.t_kolona_v, K_EXP);
						//Sensors.t_after_def = GetTemperatureMatchRom(Settings.id_t_after_def.id, &huart2);
					break;
			}
			if (GetStateMode() != MODE_NOTHING) {
				StartAllConvert_T();
			}
		} 
		// Calc pressure
		if (Settings.adc_min.value > 0 && Settings.adc_max.value > 0) {
		 pressure =((float)Settings.p_max.value - (float)Settings.p_min.value) / ((float)Settings.adc_max.value - (float)Settings.adc_min.value) * \
			((float)pressure_capture - (float)Settings.adc_min.value) + (float)Settings.p_min.value;
			Sensors.p_kub = ExpFilter(pressure, Sensors.p_kub, K_EXP_PRESSURE);
		} else {
			Sensors.p_kub = 0.0F;
		}
		//===================================================================================================================
		// Calc water flow
		Sensors.f_coolant = (float)water_capture / 7.5F; // l/min
		if (Sensors.f_coolant < 0.5F) {
			Sensors.f_coolant_al = 1;
		} else {
			Sensors.f_coolant_al = 0;
		}
		// Read spirt concetration sensor state
		Sensors.spirt_al = !HAL_GPIO_ReadPin(Spirt_concentr_IRQ_GPIO_Port, Spirt_concentr_IRQ_Pin);
		//===================================================================================================================
		
		// Read regs
		Sensors.t_coolant = R[RO_TMP1];
		Sensors.heater1_pow = GetPowerHeater1();
		Sensors.heater2_pow = GetPowerHeater2();
		Sensors.heater1_cur = GetCurentHeater1();
		Sensors.heater2_cur = GetCurentHeater2();
		Sensors.l_coolant_al = GetLevelSensorState();
		//===================================================================================================================
		// Автоматика безопасности
		SafeControl(Sensors.p_kub, Sensors.spirt_al, Sensors.f_coolant, GetCoolingPumpState());
		
		// get CPU load
		Sensors.cpu_load = osGetCPUUsage();
		
		// Отправка данных на модуль Wi-Fi, если зашли в меню выбора настроек, то данные не шлем для возможности обновления 
		// модуля Wi-Fi, вот такой костыль :(
		
		if (shPauseWifiSend != NULL) {
			if(xSemaphoreTake(shPauseWifiSend, 0) == pdTRUE){ // no wait 
				WifiSendStatus();
				xSemaphoreGive(shPauseWifiSend);
			}
	  }
					
		osDelay(1000);
  }
  /* USER CODE END StartSensPollTask */
}

/* StartRunProcess function */
void StartRunProcess(void const * argument)
{
  /* USER CODE BEGIN StartRunProcess */
	uint16_t tm1 = 0;
	uint8_t firstRun = 0;
	uint16_t period;
	float delta, delta_old;
	float t_ss;
	uint8_t ssTrig, timerOn;
	
  /* Infinite loop */
  for(;;)
  {
		switch(GetStateMode()){
			case MODE_RUN_RECT: // Режим  ректификации
				switch(GetSubMode()){
					case SMODE_RECT_PREPARE:		 // Подготовка
							Heater1On();
							Heater2On(CMD_MODE_MAN);
							SetPowerHeaterMan(1000); // 100 %
							ResetTime(&globalState.rect_state.time);
							globalState.rect_state.Q_fact = 0;
							globalState.rect_state.skip_head = 0;
							globalState.rect_state.ss_count = 0;
							globalState.rect_state.T_elapsed = 10; // 10
							globalState.rect_state.V_get_now = 0;
							SetKpPID(&pid, Settings.pwr_block_set.Kp2.value / 100.0);
							SetKiPID(&pid, Settings.pwr_block_set.Ki2.value / 100.0);
							SetKdPID(&pid, Settings.pwr_block_set.Kd2.value / 100.0);
							SetOutLimPID(&pid, 0, 1000);  // limit 0 - 100 %
							SetSubMode(SMODE_RECT_PREHEAT);
						break;
					case SMODE_RECT_PREHEAT:					// Преднагрев
								TickTime(&globalState.rect_state.time);
								if (Sensors.t_kub >= 70.0F) {
								  Heater1Off();
									#if RECT_POWER_CTRL == POWER_CONST
										Heater2On(CMD_MODE_AUTO);
										SetPowerHeaterAuto(Settings.rect_set.pow_heater.value); // Режим поддержания постоянной мощности
									#else
										Heater2On(CMD_MODE_MAN);
										SetPowerHeaterMan(Settings.rect_set.pow_heater.value); // Режим ручного управления
									#endif
									tm1 = 0;
									SetSubMode(SMODE_RECT_HEATING);
								}
						break;
					case SMODE_RECT_HEATING:					// Разогрев			
							TickTime(&globalState.rect_state.time);
							firstRun = 1;
							//if (Sensors.t_kub >= 75.0F) {  // Изменить проверку на проверку с привязкой ко времени W!
							if (TimeTempCheckGt(Sensors.t_kub, 75.0F, TEMP_TIME_CHK)) {
								CoolingFanOn(CMD_MODE_AUTO);
								CoolingPumpOn();
								if (Sensors.t_kolona_n > 60.0F && Sensors.t_kolona_v > 60.0F) {
									delta = Sensors.t_kolona_n - Sensors.t_kolona_v;
									if (fabs(delta - delta_old) <= DELTA_STABILIZE) {
										tm1++;
										if (tm1 >= 60) {
											globalState.rect_state.T_elapsed --;
											tm1 = 0;
										}
										if (globalState.rect_state.T_elapsed == 0) {
											tm1 = 0;
											globalState.rect_state.T_elapsed = 90; // 90 min work youself 1
											SetSubMode(SMODE_RECT_WORK_YOUSELF1);
										}
									} else {
										tm1 = 0;
										globalState.rect_state.T_elapsed = 10; // 10 min
										delta_old = delta;
									}
								}
							}
							
						break;
					case SMODE_RECT_WORK_YOUSELF1:	// Работа на себя
							TickTime(&globalState.rect_state.time);
							// Перевод колоны по поддержанию давления
							// или мощности по Свитеку
							RectificationSetPower(Sensors.t_kub, Sensors.p_kub, Settings.rect_set.p_kub.value, Settings.rect_set.pow_svitek.value, 
									Settings.rect_set.pow_heater.value, Settings.rect_set.mode.value, &pid);
							tm1++;
							if (tm1 >= 60 && globalState.rect_state.T_elapsed > 0) {
								if (globalState.rect_state.T_elapsed == 61) { 
									Settings.rect_set.pow_heater.value += RECT_ADD_PWR; // увеличиваем мощность после прогрева колоны
								}
								tm1 = 0;
								globalState.rect_state.T_elapsed --;
							}
							if (globalState.rect_state.T_elapsed == 0) {
								BeeperOnDelay(500);
								firstRun = 1;
							}
						break;
					case SMODE_RECT_GET_HEAD:					// Отбор голов
							TickTime(&globalState.rect_state.time);
							// Перевод колоны по поддержанию давления
							// или мощности по Свитеку
							RectificationSetPower(Sensors.t_kub, Sensors.p_kub, Settings.rect_set.p_kub.value, Settings.rect_set.pow_svitek.value, 
									Settings.rect_set.pow_heater.value, Settings.rect_set.mode.value, &pid);
					
							globalState.rect_state.V_get_now = 120; // мл/ч
							period = ValveCalcPeriod(globalState.rect_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
							if (firstRun) {
								ValveOnPwm(VALVE_1, period, VALVE_PULSE);
								firstRun = 0;
							} else {
								ValveResume();
								ValveChangePwm(period, VALVE_PULSE);
							}
							
							globalState.rect_state.Q_fact = Settings.flow_koeff.value / 2.0F * ValveGetOpenCount();
							
							if (globalState.rect_state.Q_fact >= Settings.rect_set.Q_head.value) {
								SetSubMode(SMODE_RECT_GET_SUBHEAD);
								firstRun = 1;
							}
						break;
					case SMODE_RECT_GET_SUBHEAD:   // отбор подголовников
							TickTime(&globalState.rect_state.time);
							// Перевод колоны по поддержанию давления
							// или мощности по Свитеку
							RectificationSetPower(Sensors.t_kub, Sensors.p_kub, Settings.rect_set.p_kub.value, Settings.rect_set.pow_svitek.value, 
									Settings.rect_set.pow_heater.value, Settings.rect_set.mode.value, &pid);
					
							globalState.rect_state.V_get_now = 300; // мл
							period = ValveCalcPeriod(globalState.rect_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
							if (firstRun) {
								ValveOnPwm(VALVE_3, period, VALVE_PULSE);
								firstRun = 0;
							} else {
								ValveResume();
								ValveChangePwm(period, VALVE_PULSE);
							}
							
							globalState.rect_state.Q_fact = Settings.flow_koeff.value / 2.0F * ValveGetOpenCount();
							
							if (globalState.rect_state.Q_fact >= Settings.rect_set.Q_sub_head.value) {
								SetSubMode(SMODE_RECT_WORK_YOUSELF2);
								globalState.rect_state.T_elapsed = 25; // 25 min work youself 2
								ValveOffAll();
								BeeperOnDelay(15000);
								//firstRun = 1;
							}
						break;
					case SMODE_RECT_WORK_YOUSELF2:			// Работа на себя 2
							TickTime(&globalState.rect_state.time);
							// Перевод колоны по поддержанию давления
							// или мощности по Свитеку
							RectificationSetPower(Sensors.t_kub, Sensors.p_kub, Settings.rect_set.p_kub.value, Settings.rect_set.pow_svitek.value, 
									Settings.rect_set.pow_heater.value, Settings.rect_set.mode.value, &pid);
					
							tm1++;
							if (tm1 >= 60 && globalState.rect_state.T_elapsed > 0) {
								tm1 = 0;
								globalState.rect_state.T_elapsed --;
							}
							if (globalState.rect_state.T_elapsed == 0) {
								SetSubMode(SMODE_RECT_GET_BODY);
								t_ss = Sensors.t_kolona_n;
								globalState.rect_state.V_get_now = CalcVBody(Sensors.t_kub, Settings.rect_set.V_get.value, globalState.rect_state.ss_count); // "Шпора"
								period = ValveCalcPeriod(globalState.rect_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
								ValveOnPwm(VALVE_2, period, VALVE_PULSE);
								BeeperOnDelay(1500);
								//firstRun = 1;
							}
						break;
					case SMODE_RECT_GET_BODY:					// Отбор тела
						
						TickTime(&globalState.rect_state.time);
						// Перевод колоны по поддержанию давления
						// или мощности по Свитеку
						RectificationSetPower(Sensors.t_kub, Sensors.p_kub, Settings.rect_set.p_kub.value, Settings.rect_set.pow_svitek.value, 
							Settings.rect_set.pow_heater.value, Settings.rect_set.mode.value, &pid);
					
						globalState.rect_state.V_get_now = CalcVBody(Sensors.t_kub, Settings.rect_set.V_get.value, globalState.rect_state.ss_count); // "Шпора"
						period = ValveCalcPeriod(globalState.rect_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
							
						
						if (Sensors.t_kolona_n >= Settings.rect_set.dt.value + t_ss ) {
                tm1 = 0;
								ValvePause();
                timerOn = 0;
                if (ssTrig == 0) {
									ssTrig = 1;
                  globalState.rect_state.ss_count ++;
                }
            } else {
               if (timerOn) {
									ValveResume();
									ValveChangePwm(period, VALVE_PULSE);
									ssTrig = 0;
								} else {
									 tm1 ++;
                   if (Sensors.t_kub > Settings.rect_set.t_kub_ss.value) {
                       if (tm1 >= Settings.rect_set.T2.value * 60) {
                           timerOn = 1;
                       }
                    } else {
                       if (tm1 >= Settings.rect_set.T1.value * 60) {
                           timerOn = 1;
                       }
                    }
                 }    
              }
							//if (Sensors.t_kub >= 94.0F) { // Изменить проверку на проверку с привязкой ко времени W!
							if (TimeTempCheckGt(Sensors.t_kub, 94.0F, TEMP_TIME_CHK)){
								BeeperOnDelay(1000);
								firstRun = 1;
								SetSubMode(SMODE_RECT_GET_TAIL);
							} 
  					break;
					case SMODE_RECT_GET_TAIL:					// Отбор хвостов
						
							TickTime(&globalState.dist_state.time);
							// Перевод колоны по поддержанию давления
							// или мощности по Свитеку
							RectificationSetPower(Sensors.t_kub, Sensors.p_kub, Settings.rect_set.p_kub.value, Settings.rect_set.pow_svitek.value, 
									Settings.rect_set.pow_heater.value, Settings.rect_set.mode.value, &pid);
					
							globalState.rect_state.V_get_now = Settings.rect_set.V_get.value / 2 ;
							period = ValveCalcPeriod(globalState.rect_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
											
							if (firstRun) {
								ValveOnPwm(VALVE_3, period, VALVE_PULSE);
								firstRun = 0;
							} else {
								ValveChangePwm(period, VALVE_PULSE);
							}
							
							if (Sensors.t_kub >= Settings.rect_set.t_kub_off.value) { // Изменить проверку на проверку с привязкой ко времени W!
								tm1 = 0;
								globalState.rect_state.T_elapsed = 3; // min
								ValveOffAll();
								Heater1Off();
								Heater2Off();
								SetSubMode(SMODE_RECT_END);
							}
						break;
					case SMODE_RECT_END:							// Окончание процесса
						
							TickTime(&globalState.rect_state.time);
							tm1++;
							if (tm1 >= 60 && globalState.rect_state.T_elapsed > 0) {
								tm1 = 0;
								globalState.rect_state.T_elapsed --;
							}
							if (globalState.rect_state.T_elapsed == 0) {
								CoolingFanOff();
								CoolingPumpOff();
							}
						break;
					case SMODE_RECT_GET_PAUSE:				// Пауза отбора
							ValvePause();
						break;
					case SMODE_RECT_EXIT:
							Heater1Off();
							Heater2Off();
							ValveOffAll();
							CoolingFanOff();
							CoolingPumpOff();
							SetStateMode(MODE_NOTHING);
						break;
					case SMODE_EMSTOP:
							Heater1Off();
							Heater2Off();
							ValveOffAll();
							//CoolingFanOff();
							//CoolingPumpOff();
							//SetStateMode(MODE_NOTHING);
						break;
					}
				break;
//=================================================================================================================================
			case MODE_RUN_DIST: 								// Режим  дистилляции
				switch(GetSubMode()){
					case SMODE_DIST_PREPARE:				// Подготовка
							Heater1On();
							Heater2On(CMD_MODE_MAN);
							SetPowerHeaterMan(1000); 		// 100 %
							ResetTime(&globalState.dist_state.time);
							firstRun = 1;
							SetSubMode(SMODE_DIST_PREHEAT);
						break;
					case SMODE_DIST_PREHEAT:					// Преднагрев
								TickTime(&globalState.dist_state.time);
								if (Sensors.t_kub >= 70.0F) {
								  Heater1Off();
									#if DIST_POWER_CTRL == POWER_CONST
										Heater2On(CMD_MODE_AUTO);
										SetPowerHeaterAuto(Settings.dist_set.pow_heater.value); // Режим поддержания постоянной мощности
									#else
										Heater2On(CMD_MODE_MAN);
										SetPowerHeaterMan(Settings.dist_set.pow_heater.value); // Режим ручного управления
									#endif
									SetSubMode(SMODE_DIST_HEATING);
								}
						break;
					case SMODE_DIST_HEATING:					// Разогрев
							TickTime(&globalState.dist_state.time);
							#if DIST_POWER_CTRL == POWER_CONST
							SetPowerHeaterAuto(Settings.dist_set.pow_heater.value);  
							#else
							SetPowerHeaterMan(Settings.dist_set.pow_heater.value); 
							#endif
							firstRun = 1;
							if (Sensors.t_kub >= 75.0F) {
								CoolingFanOn(CMD_MODE_AUTO);
								CoolingPumpOn();
							}
						break;
					case SMODE_DIST_GET_HEAD:					// Отбор голов
							#if DIST_POWER_CTRL == POWER_CONST
							SetPowerHeaterAuto(Settings.dist_set.pow_heater.value);  
							#else
							SetPowerHeaterMan(Settings.dist_set.pow_heater.value); 
							#endif
							TickTime(&globalState.dist_state.time);
							globalState.dist_state.V_get_now = 0.1F * (float)Settings.dist_set.V_get.value;
							period = ValveCalcPeriod(globalState.dist_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
							if (firstRun) {
								ValveOnPwm(VALVE_1, period, VALVE_PULSE);
								firstRun = 0;
							} else {
								ValveResume();
								ValveChangePwm(period, VALVE_PULSE);
							}
							
							globalState.dist_state.Q_fact = Settings.flow_koeff.value / 2.0F * ValveGetOpenCount();
							
							if (globalState.dist_state.Q_fact >= Settings.dist_set.Q_head.value) {
								tm1 = 0;
								globalState.dist_state.T_elapsed = 15; // minutes 15
								ValveOffAll();
								BeeperPulseDelay(250, 500, 2500);
								SetSubMode(SMODE_DIST_WORK_YOUSELF);
								firstRun = 1;
							}
						break;
					case SMODE_DIST_WORK_YOUSELF:			// Работа на себя
							TickTime(&globalState.dist_state.time);
							tm1++;
							if (tm1 >= 60 && globalState.dist_state.T_elapsed > 0) {
								tm1 = 0;
								globalState.dist_state.T_elapsed --;
							}
							if (globalState.dist_state.T_elapsed == 0) {
								BeeperOnDelay(1500);
							}
							#if DIST_POWER_CTRL == POWER_CONST
							SetPowerHeaterAuto(Settings.dist_set.pow_heater.value);  
							#else
							SetPowerHeaterMan(Settings.dist_set.pow_heater.value); 
							#endif
						break;
					case SMODE_DIST_GET_BODY:					// Отбор тела
						
							TickTime(&globalState.dist_state.time);
							globalState.dist_state.V_get_now = Settings.dist_set.V_get.value;
							period = ValveCalcPeriod(globalState.dist_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
															
							if (firstRun) {
								ValveOnPwm(VALVE_2, period, VALVE_PULSE);
								firstRun = 0;
							} else {
								ValveResume();
								ValveChangePwm(period, VALVE_PULSE);
							}
							
							#if DIST_POWER_CTRL == POWER_CONST
							SetPowerHeaterAuto(Settings.dist_set.pow_heater.value);  
							#else
							SetPowerHeaterMan(Settings.dist_set.pow_heater.value); 
							#endif
							if (Sensors.t_kub >= Settings.dist_set.t_kub_get_tail.value) {
								BeeperOnDelay(500);
								firstRun = 1;
								SetSubMode(SMODE_DIST_GET_TAIL);
							}
						break;
					case SMODE_DIST_GET_TAIL:					// Отбор хвостов
						
							TickTime(&globalState.dist_state.time);
							globalState.dist_state.V_get_now = Settings.dist_set.V_get.value / 2 ;
							period = ValveCalcPeriod(globalState.dist_state.V_get_now, Settings.flow_koeff.value, VALVE_PULSE);
											
							if (firstRun) {
								ValveOnPwm(VALVE_3, period, VALVE_PULSE);
								firstRun = 0;
							} else {
								ValveChangePwm(period, VALVE_PULSE);
							}
							
							#if DIST_POWER_CTRL == POWER_CONST
							SetPowerHeaterAuto(Settings.dist_set.pow_heater.value);  
							#else
							SetPowerHeaterMan(Settings.dist_set.pow_heater.value); 
							#endif
							if (Sensors.t_kub >= Settings.dist_set.t_kub_off.value) {
								tm1 = 0;
								globalState.dist_state.T_elapsed = 3; // min
								ValveOffAll();
								Heater1Off();
								Heater2Off();
								SetSubMode(SMODE_DIST_END);
							}
						break;
					case SMODE_DIST_END:							// Окончание процесса
						
							TickTime(&globalState.dist_state.time);
							tm1++;
							if (tm1 >= 60 && globalState.dist_state.T_elapsed > 0) {
								tm1 = 0;
								globalState.dist_state.T_elapsed --;
							}
							if (globalState.dist_state.T_elapsed == 0) {
								CoolingFanOff();
								CoolingPumpOff();
							}
						break;
					case SMODE_DIST_GET_PAUSE:				// Пауза отбора
							ValvePause();
						break;
					case SMODE_DIST_EXIT:
							Heater1Off();
							Heater2Off();
							ValveOffAll();
							CoolingFanOff();
							CoolingPumpOff();
							SetStateMode(MODE_NOTHING);
						break;
					case SMODE_EMSTOP:
							Heater1Off();
							Heater2Off();
							ValveOffAll();
							//CoolingFanOff();
							//CoolingPumpOff();
							//SetStateMode(MODE_NOTHING);
						break;
					}
				break;
//=================================================================================================================================
			case MODE_RUN_CLS: // Режим  чистки колоны
					switch(GetSubMode()){
						case SMODE_CLR_HEATING:
								Heater1On();
								Heater2On(CMD_MODE_MAN);
								SetPowerHeaterMan(1000); // 100 %
								globalState.clr_state.T_elapsed = Settings.clean_set.T_cleaning.value;
								if (Sensors.t_kub >= 95) {
									tm1 = 0;	
									SetSubMode(SMODE_CLR_CLEANING);
								}
							break;
						case SMODE_CLR_CLEANING:
								tm1++;
								if (tm1 >= 60) {
									tm1 = 0;
									globalState.clr_state.T_elapsed --;
								}
								if (Sensors.t_kub >= Settings.clean_set.t_open_valve.value) {
									globalState.clr_state.valve_num = 0;
									CoolingPumpOn();
									CoolingFanOn(CMD_MODE_AUTO); 
									SetSubMode(SMODE_CLR_OPNVALVE);
									BeeperOnDelay(500);
								}								
							break;
						case SMODE_CLR_OPNVALVE:
								tm1++;
								if (tm1 >= 60) {
									tm1 = 0;
									globalState.clr_state.T_elapsed --;
								}
								switch(globalState.clr_state.valve_num){
									case 0: 
											ValveOn(VALVE_1);
										break;
									case 2: 
											ValveOn(VALVE_2);
										break;
									case 4: 
											ValveOn(VALVE_3);
										break;
									default:
											ValveOffAll();
										break;
								}
								
								globalState.clr_state.valve_num ++;
								if (globalState.clr_state.valve_num  > 5) {
									globalState.clr_state.valve_num = 0;
								}
								if (globalState.clr_state.T_elapsed == 0) {
									Heater1Off();
									Heater2Off();
									ValveOffAll();
									CoolingPumpOff();
									CoolingFanOff();
									BeeperPulseDelay(250, 500, 3000);
									SetStateMode(MODE_NOTHING);
								}
							break;
						case SMODE_CLR_EXIT:
								Heater1Off();
								Heater2Off();
								ValveOffAll();
								CoolingPumpOff();
								CoolingFanOff();
								SetStateMode(MODE_NOTHING);
							break;
						
					}
				break;
//=================================================================================================================================
				case MODE_SET_VC: // Режим калибровки клапанов
						switch(GetSubMode()){
							case SMODE_VC_START_HEAT:
									Heater2On(CMD_MODE_MAN);
									SetPowerHeaterMan(Settings.dist_set.pow_heater.value); // Режим ручного управления
									CoolingFanOn(CMD_MODE_AUTO);
									CoolingPumpOn();
								break;								
							case SMODE_VC_PREPARE:  // Подготавливаем
									globalState.vlvclb_state.pulseCount = 200.0F / Settings.flow_koeff.value * 2; // 50 мл со скоростью 1000 мл час свременем работы клапана 500/500 мс
									globalState.vlvclb_state.Q_fact = 200;
									globalState.vlvclb_state.Q_calc = 0.0F;
									ValveOnPwm(VALVE_1, 1000, VALVE_PULSE);
									SetSubMode(SMODE_VC_CAL);
								break;
							case SMODE_VC_CAL: // Калибруем
									globalState.vlvclb_state.Q_calc = Settings.flow_koeff.value * (float)ValveGetOpenCount() / 2.0F;
									SetPowerHeaterMan(Settings.dist_set.pow_heater.value); // Режим ручного управления
									if (ValveGetOpenCount() > globalState.vlvclb_state.pulseCount){
										Heater1Off();
										Heater2Off();
										ValveOffAll();
										CoolingPumpOff();
										CoolingFanOff();
										SetSubMode(SMODE_VC_ENDSAVE);
										BeeperPulseDelay(500, 1000, 3000);
									}
								break;
							case SMODE_VC_ENDSAVE:  // считам новый коэффициент
									Settings.flow_koeff.value = (float)globalState.vlvclb_state.Q_fact / (float)globalState.vlvclb_state.pulseCount * 2.0F;
								break;
							case SMODE_VC_STOP:  //Ждем, если остановили калибровку 
									Heater1Off();
									Heater2Off();
									ValveOffAll();
									CoolingPumpOff();
									CoolingFanOff();
								break;
							case SMODE_EMSTOP:
								Heater1Off();
								Heater2Off();
								ValveOffAll();
								//CoolingFanOff();
								//CoolingPumpOff();
								//SetStateMode(MODE_NOTHING);
							break;
						}
					break;
							
		}
    osDelay(1000);
  }
  /* USER CODE END StartRunProcess */
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
	if (htim->Instance == TIM1) {
		WakeMaster_Tick();
		BeeperTick();
		ValveTick();
		OWI_Tick();
  }
	
	if (htim->Instance == TIM6){
		 pressure_capture = __HAL_TIM_GetCounter(&htim3);	//read TIM3 counter value
		 water_capture = __HAL_TIM_GetCounter(&htim8);
		__HAL_TIM_SetCounter(&htim3, 0);	//reset counter
		__HAL_TIM_SetCounter(&htim8, 0);
	}
/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
