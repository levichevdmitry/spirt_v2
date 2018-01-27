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
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <wake.h>
#include <registers.h>
#include <settings.h>
#include <OWIHighLevelFunctions.h>
#include <util.h>
#include <pid.h>
#include <cpu_utils.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId mainWorkerHandle;
osThreadId heaterWorkerHandle;
osThreadId tempSenHandle;
osThreadId rmsCalcHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

xSemaphoreHandle  RW_Reg_Mutex, rmsCalcSemaphor;

uint8_t sens_count = 0;
float coolant_temp = 0.0;
float heater2Power = 0;
uint8_t OW_Status;
uint8_t heater2_sp_halpfs;
uint8_t halpf_count; // counter halpf periods for full cicle 
uint16_t adc_buf[3]; // ADC raw value
CircleBuffer * CurentSensor1;
CircleBuffer * CurentSensor2;
CircleBuffer * AvgCS1;
CircleBuffer * AvgCS2;
BitCircleBuffer * SlideHalpfCounter;

uint8_t  adc_ready = 0,
				 cs_calibrated = 0;
uint16_t cs1_offset = 0, 
				 cs2_offset = 0;

int16_t	cpu_temp;
int16_t cs2_avg_mv;

uint8_t halph_wave_counter;
uint8_t curent_en_flag = 0;
#if HEATER_CTRL_METHOD == POWER_CTRL_BRESENHAM
int16_t error;	
#endif

//----------------------------
volatile uint8_t heater_pwr;
volatile uint16_t sp;
SPid pid;
volatile float pv, err, maxPower;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartMainWorker(void const * argument);
void StartHeaterWorker(void const * argument);
void StartTempSen(void const * argument);
void StartRmsCalc(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Init_Registers(void);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2) {
		
		// Необходимость данного "костыля" не доказана :)
		if (huart->ErrorCode == HAL_UART_ERROR_ORE){
        // remove the error condition
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        // set the correct state, so that the UART_RX_IT works correctly
        huart->State = HAL_UART_STATE_BUSY_RX;
    }
	}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	Init_Registers();
	Wake_Init(&huart1);
	OW_Init(&huart2);
	HAL_IWDG_Start(&hiwdg);
	CurentSensor1 = CB_Create(CB_CS_CAPACITY);
	CurentSensor2 = CB_Create(CB_CS_CAPACITY);
	AvgCS1 = CB_Create(CB_AVG_CAPACITY1);
	AvgCS2 = CB_Create(CB_AVG_CAPACITY2);
	//SlideHalpfConter = BitCB_Create(CB_AVG_CAPACITY2 * 2); //100 * 2 = 200 halpf periods / 8 = (25 byte)
	SlideHalpfCounter = BitCB_Create(CB_AVG_CAPACITY2); // 100 halpf periods / 8 + 1 = (13 byte)
	adc_ready = 1; // start adc conversion after all configuration
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	RW_Reg_Mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of mainWorker */
  osThreadDef(mainWorker, StartMainWorker, osPriorityNormal, 0, 128);
  mainWorkerHandle = osThreadCreate(osThread(mainWorker), NULL);

  /* definition and creation of heaterWorker */
  osThreadDef(heaterWorker, StartHeaterWorker, osPriorityNormal, 0, 128);
  heaterWorkerHandle = osThreadCreate(osThread(heaterWorker), NULL);

  /* definition and creation of tempSen */
  osThreadDef(tempSen, StartTempSen, osPriorityBelowNormal, 0, 128);
  tempSenHandle = osThreadCreate(osThread(tempSen), NULL);

  /* definition and creation of rmsCalc */
  osThreadDef(rmsCalc, StartRmsCalc, osPriorityAboveNormal, 0, 128);
  rmsCalcHandle = osThreadCreate(osThread(rmsCalc), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 3;
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

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50; // 5 ms
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : ZQ_IRQ_Pin */
  GPIO_InitStruct.Pin = ZQ_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZQ_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Water_Level_Pin DI1_Pin DI2_Pin OH_Pin 
                           En_Pin */
  GPIO_InitStruct.Pin = Water_Level_Pin|DI1_Pin|DI2_Pin|OH_Pin 
                          |En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OH_Led_Pin Link_Led_Pin FAN_Pin Rdy_Led_Pin 
                           G5_Pin G4_Pin G3_Pin G2_Pin 
                           G1_Pin */
  GPIO_InitStruct.Pin = OH_Led_Pin|Link_Led_Pin|FAN_Pin|Rdy_Led_Pin 
                          |G5_Pin|G4_Pin|G3_Pin|G2_Pin 
                          |G1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Rdy_Pin */
  GPIO_InitStruct.Pin = Rdy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Rdy_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OH_Led_Pin|Link_Led_Pin|FAN_Pin|Rdy_Led_Pin 
                          |G5_Pin|G4_Pin|G3_Pin|G2_Pin 
                          |G1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Rdy_GPIO_Port, Rdy_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

// Начальная инициализация параметров
void Init_Registers(void){
	R[RO_HSV] = HW_SW_VER; 	// See settings.h
	R[RW_SP_PWR_M] = 500;   // 50 %
	R[RW_SP_PWR_A] = 500;		// 50 %
	R[RW_SP_T] = 24;				// 24 C
	R[RW_R1] = 323;					// 32.3 Om
	R[RW_R2] = 161;					// 16.1 Om
	R[RW_KP] = 100;					// Kp = 1;
	R[RW_KI] = 1;						// Ki = 0,01
	R[RW_KD] = 0;						// Kd = 0
	R[RW_CMD] = 0x0000;			// CMD = 0
	R[RO_ETA] = 0x0000;			// ETA = 0
}
//--------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) //huart == &huart1 
	{
			HAL_GPIO_WritePin(Link_Led_GPIO_Port, Link_Led_Pin, GPIO_PIN_SET);
			Wake_UART_Rx();	
	}
}
//--------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) //huart == &huart1
	{	
			Wake_UART_Tx();
			HAL_GPIO_WritePin(Link_Led_GPIO_Port, Link_Led_Pin, GPIO_PIN_RESET);
	}
}
//--------------------------------------------------------------------------------------
// Zero cross detection
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
/*
static uint8_t halph_wave_counter;
#if HEATER_CTRL_METHOD == POWER_CTRL_BRESENHAM
static int16_t error;	
#endif
	*/
//static uint8_t period_flag = 0;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if (GPIO_Pin == ZQ_IRQ_Pin) {
			if (heater2_sp_halpfs == HALPHS_COUNT) { // tolerance 0.5 %
				//HEATER_ON;
				HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_SET);
				halph_wave_counter = 0;
				BitCB_PlaceTail(SlideHalpfCounter, 1); 
				heater2_sp_halpfs = heater_pwr; // write new power value
				curent_en_flag = 1;
				#if HEATER_CTRL_METHOD == POWER_CTRL_BRESENHAM
				error = HALPHS_COUNT / 2;	
				#endif
			} else
			#if HEATER_CTRL_METHOD == POWER_CTRL_PWM  // PWM control
				if (heater2_sp_halpfs > 0) {
					if (++halph_wave_counter >= heater2_sp_halpfs) {
						//HEATER_OFF;
						BitCB_PlaceTail(SlideHalpfConter, 0); 
						HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
						curent_en_flag = 0;
					} else { 
						//HEATER_ON;
						BitCB_PlaceTail(SlideHalpfConter, 1); 
						HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_SET);
						curent_en_flag = 1;
					} 
					if (halph_wave_counter >= HALPHS_COUNT) { //199
						halph_wave_counter = 0;
						heater2_sp_halpfs = heater_pwr; // write new power value
					}
				} else {
					//HEATER_OFF;
					HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
					curent_en_flag = 0;
					halph_wave_counter = 0;
					BitCB_PlaceTail(SlideHalpfConter, 0); 
					heater2_sp_halpfs = heater_pwr; // write new power value
				}
			#else // Bresenham method control
				if (heater2_sp_halpfs > 0) {
					
					error -= heater2_sp_halpfs; 
					if (error < 0) {
						//HEATER_ON;
						error +=  HALPHS_COUNT;
						BitCB_PlaceTail(SlideHalpfCounter, 1); 
						HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_SET);
						curent_en_flag = 1;
						HAL_TIM_Base_Start_IT(&htim6);
					} else { 
						//HEATER_OFF;
						BitCB_PlaceTail(SlideHalpfCounter, 0); 
						HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
						curent_en_flag = 0;
					} 
					if (++halph_wave_counter >= HALPHS_COUNT) { //199
						halph_wave_counter = 0;
						heater2_sp_halpfs = heater_pwr; // write new power value
						error = HALPHS_COUNT / 2;
					}
					/*
					error += heater2_sp_halpfs;
					if (error >= HALPHS_COUNT) {
						//HEATER_ON;
						error -=  HALPHS_COUNT;
						BitCB_PlaceTail(SlideHalpfConter, 1); 
						HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_SET);
						curent_en_flag = 1;
						HAL_TIM_Base_Start_IT(&htim6);
					} else { 
						//HEATER_OFF;
						BitCB_PlaceTail(SlideHalpfConter, 0); 		
						HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
						curent_en_flag = 0;
						heater2_sp_halpfs = heater_pwr; // write new power value
					} 
					*/
				} else {
					//HEATER_OFF;
					HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
					curent_en_flag = 0;
					halph_wave_counter = 0;
					BitCB_PlaceTail(SlideHalpfCounter, 0); 
					heater2_sp_halpfs = heater_pwr; // write new power value
					error = HALPHS_COUNT / 2;
				}
			#endif
		//period_flag = !period_flag;
		//if (period_flag && rmsCalcSemaphor != NULL) { // синхронизация расчета с переходом серез ноль каждый период
		if (rmsCalcSemaphor != NULL) { // синхронизация расчета с переходом серез ноль каждый период		
			xSemaphoreGiveFromISR(rmsCalcSemaphor, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); // Переключаемся на задачу обработчик
		}
	}
}
//--------------------------------------------------------------------------------------
// End ADC conversion
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc == &hadc1) {
		int16_t SenValRaw;
		const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
		const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
				
		if (cs_calibrated == CS_AVG_CNT) {
			SenValRaw = (float)CS_ADC_VREF / (float)CS_ADC_MAX * adc_buf[1] * CS_K_ATT - 2500 + 65;//(float)cs1_offset; // get mv from cs 1 
			CB_PlaceTail(CurentSensor1, SenValRaw);
			if (curent_en_flag) { 
				SenValRaw = (float)CS_ADC_VREF / (float)CS_ADC_MAX * adc_buf[2] * CS_K_ATT - 2500 + 65;//(float)cs2_offset; // get mv from cs 2
			} else {
				SenValRaw = 0;
			}
			CB_PlaceTail(CurentSensor2, SenValRaw);
			
			cpu_temp = (uint16_t)((V25 - adc_buf[0]) / Avg_Slope - 25); // расчет температуры
			
		} else {
			cs1_offset += adc_buf[1]; // измеряем ток без нагрузки, т.е. ноль
			cs2_offset += adc_buf[2];
			cs_calibrated ++;
			if (cs_calibrated == CS_AVG_CNT){ // усредняем и разрешаем работу главного реле
				cs1_offset /= CS_AVG_CNT;
				cs1_offset = (float)CS_ADC_VREF / (float)CS_ADC_MAX * cs1_offset * CS_K_ATT;
				cs2_offset /= CS_AVG_CNT;
				cs2_offset = (float)CS_ADC_VREF / (float)CS_ADC_MAX * cs2_offset * CS_K_ATT;
			}
		}
	}
}

/* USER CODE END 4 */

/* StartMainWorker function */
void StartMainWorker(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET); // on cooler
	
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
		
		if (xSemaphoreTake(RW_Reg_Mutex, portMAX_DELAY) == pdTRUE) {
			Wake_Commands_Exe();
			
			// main relay and ready led control
			if (HAL_GPIO_ReadPin(OH_GPIO_Port, OH_Pin) && //Проверка датчика перегрева радиаторов
					!HAL_GPIO_ReadPin(En_GPIO_Port, En_Pin) && // Проверка разрешающего сигнала от блока управления
					(cs_calibrated == CS_AVG_CNT)){ // проверка окончания процесса калибровки датчиков тока
						
				HAL_GPIO_WritePin(Rdy_Led_GPIO_Port, Rdy_Led_Pin, GPIO_PIN_SET); // on Ready led	
				R[RO_ETA] |= ((uint16_t)1 <<  ETA_BIT_MR); // set main relay bit in ETA
			} else {
				
				HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_RESET); // off heater 1
				HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET); // off heater 2
				HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_RESET); // off mixer
				HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_RESET); // off fan
				HAL_GPIO_WritePin(G5_GPIO_Port, G5_Pin, GPIO_PIN_RESET); // off pump
								
				HAL_GPIO_WritePin(Rdy_Led_GPIO_Port, Rdy_Led_Pin, GPIO_PIN_RESET); // off Ready led
				
				R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_MR); // reset main relay bit in ETA
				R[RW_CMD] = 0x0000; // Выключаем все при отвале реле
			}
			
			// проверка на перегрев радиаторов
			if (!HAL_GPIO_ReadPin(OH_GPIO_Port, OH_Pin)){
				HAL_GPIO_WritePin(OH_Led_GPIO_Port, OH_Led_Pin, GPIO_PIN_SET); // on OH led	
				HAL_GPIO_WritePin(Rdy_GPIO_Port, Rdy_Pin, GPIO_PIN_RESET); // off Ready signal
				R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_OH); // set OH bit in ETA
			} else {
				HAL_GPIO_WritePin(OH_Led_GPIO_Port, OH_Led_Pin, GPIO_PIN_RESET); // off OH led
				HAL_GPIO_WritePin(Rdy_GPIO_Port, Rdy_Pin, GPIO_PIN_SET); // on Ready signal
				R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_OH); // reset OH bit in ETA
			}
			
			// cooling system water tempreture sensor
			if (sens_count != 0){
				R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_TS); // set temp sen bit in ETA
			} else {
				R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_TS); // reset temp sen bit in ETA
			}
			
			// level sensor 
			if (HAL_GPIO_ReadPin(Water_Level_GPIO_Port, Water_Level_Pin)){
				R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_LS); // set level bit in ETA
			} else {
				R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_LS); // reset level bit in ETA
			}
			
			// heater1 on\ off state
			if ( (R[RW_CMD]&((uint16_t)1 << CMD_BIT_H1E)) != 0){
				HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_SET); // on heater 1
			} else {
				HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_RESET); // off heater 1
			}
			
			// mixer on\ off state
			if ( (R[RW_CMD]&((uint16_t)1 << CMD_BIT_ME)) != 0){
				HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_SET); // on mixer
				R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_MS); // set mixer bit in ETA
			} else {
				HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_RESET); // off mixer
				R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_MS); // reset mixer bit in ETA
			}
			
			// pump on\ off state
			if ( (R[RW_CMD]&((uint16_t)1 << CMD_BIT_PE)) != 0){
				HAL_GPIO_WritePin(G5_GPIO_Port, G5_Pin, GPIO_PIN_SET); // on pump
				R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_PS); // set pump bit in ETA
			} else {
				HAL_GPIO_WritePin(G5_GPIO_Port, G5_Pin, GPIO_PIN_RESET); // off pump
				R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_PS); // reset pump bit in ETA
			}
			
			// если датчик температуры обнаружен, то работа в штатном режиме, иначе включаем вентилятор совместно с насосом
			if (sens_count) {
				// cooling system fan control
				if ((R[RW_CMD]&((uint16_t)1 << CMD_BIT_FE)) != 0){
					//
					if ((R[RW_CMD]&((uint16_t)1 << CMD_BIT_FM)) != 0) { // fan auto mode
						if ((uint16_t)coolant_temp >= R[RW_SP_T] ) {
						HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_SET); // on fan
						R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_FS); // set fan bit in ETA
					} else if ((uint16_t)coolant_temp < R[RW_SP_T] - DELTA_T_COOL){
						HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_RESET); // off fan
						R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_FS); // reset fan bit in ETA
						}
					} else { // fan manual mode
						HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_SET); // on fan
						R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_FS); // set fan bit in ETA
					}
				} else { // off fan
					HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_RESET); // off fan
					R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_FS); // reset fan bit in ETA
				}
			} else {
				if ( (R[RW_CMD]&((uint16_t)1 << CMD_BIT_PE)) != 0){
					HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_SET); // on fan
					R[RO_ETA] |= ((uint16_t)1 << ETA_BIT_FS); // set fan bit in ETA
				} else {
					HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_RESET); // off fan
					R[RO_ETA] &= ~((uint16_t)1 << ETA_BIT_FS); // reset fan bit in ETA
				}
			}			
			xSemaphoreGive(RW_Reg_Mutex);
		}
		
		osDelay(10);
  }
  /* USER CODE END 5 */ 
}

/* StartHeaterWorker function */
void StartHeaterWorker(void const * argument)
{
  /* USER CODE BEGIN StartHeaterWorker */
	float tmp_pwr;

	SetOutLimPID(&pid, 0.0, (float)HALPHS_COUNT); // tollerance 0.5 %
  /* Infinite loop */
  for(;;)
  {
		if (xSemaphoreTake(RW_Reg_Mutex, portMAX_DELAY) == pdTRUE) {
			
			// Расчет выходной мощности			
			R[RO_PWR1] = (((float)R[RO_CS1] / 100.0) * ((float)R[RO_CS1] / 100.0) * ((float)R[RW_R1] / 10.0) / 10); // 0.01 kW
			//*********************************	
			halpf_count = BitCB_GetOnes(SlideHalpfCounter);
			// Расчет мощности для второго ТЭНа, пропорционально времени включения (количеству полупериодов)
			tmp_pwr =   ((float)HALPHS_COUNT / (float)halpf_count * ((float)R[RO_CS2] / 100.0)) * ((float)HALPHS_COUNT / (float)halpf_count * ((float)R[RO_CS2] / 100.0));
			#if CS2_EXP_FILTER == 0
				heater2Power = (tmp_pwr * ((float)R[RW_R2] / 10.0) * ((float)halpf_count / (float)HALPHS_COUNT)); // W
			#else
				heater2Power = heater2Power * (1.0F - CS2_EXP_COEFF) + CS2_EXP_COEFF * (tmp_pwr * ((float)R[RW_R2] / 10.0) * ((float)halpf_count / (float)HALPHS_COUNT)); // W
			#endif
			
			R[RO_PWR2] = ( heater2Power / 10); // 0.01 kW
			//*********************************
			R[RO_TMP1] = coolant_temp;
			R[RO_CPU_TMP] = cpu_temp;
			R[RO_CPU_LOD] = osGetCPUUsage();
			
			if ((R[RW_CMD]&((uint16_t)1 << CMD_BIT_H2E)) != 0) { // heater 2 on
				if ((R[RW_CMD]&((uint16_t)1 << CMD_BIT_H2M)) != 0) { // heater 2 on in auto mode
					//  ПИД 			
					//sp = (R[RW_SP_PWR_A] / 10) * 2;  // %
					//sp += (R[RW_SP_PWR_A] % 10) / 5; // 0.5 % from 0 to 200
					sp = (R[RW_SP_PWR_A] / 10);  // 1 % from 0 to 100
					
					maxPower = (float)(HEATER_W_VOLT * HEATER_W_VOLT) / ((float)R[RW_R2] / 10.0); // максимальная теоритическая мощность
					
					pv = (heater2Power * (float)HALPHS_COUNT) / maxPower; // реальная мощность на ТЭНе в 0.5%
				
					err = (float)sp - pv; // Прямой закон регулирования
					if (err < -PID_HYST || err > PID_HYST) { // Hysteresys
						SetKpPID(&pid, R[RW_KP] / 100.0);
						SetKiPID(&pid, R[RW_KI] / 100.0);
						SetKdPID(&pid, R[RW_KD] / 100.0);
						heater_pwr = UpdatePID(&pid, err, pv); 
					}
					
				} else { // heater 2 on in manual mode
					//heater_pwr = (R[RW_SP_PWR_M] / 10) * 2;  // %
					//heater_pwr += (R[RW_SP_PWR_M] % 10) / 5; // 0.5 %
					heater_pwr = (R[RW_SP_PWR_M] / 10);  // 1 % from 0 to 100
					ResetiStatePID(&pid);
				}
			} else { // heater 2 off
				ResetiStatePID(&pid);
				heater_pwr = 0;
			}
			/*
			portENTER_CRITICAL();
				heater2_sp_halpfs = heater_pwr; // write new power value
			portEXIT_CRITICAL();
			*/
			xSemaphoreGive(RW_Reg_Mutex);
		}
    osDelay(1000);
  }
  /* USER CODE END StartHeaterWorker */
}

/* StartTempSen function */
void StartTempSen(void const * argument)
{
  /* USER CODE BEGIN StartTempSen */
	OWI_device devices[MAX_DS18B20];
	uint8_t num = 0;
	if (OWI_SearchDevices(devices, MAX_DS18B20, &huart2, &num) == SEARCH_SUCCESSFUL){
		sens_count = num;
	}
  /* Infinite loop */
  for(;;)
  {
		if (num > 0) { // если есть датчики, то читаем температуру
			if (OWI_DetectPresence(&huart2) == OW_OK) {
				coolant_temp = GetTemperatureSkipRom(&huart2);
				StartAllConvert_T(&huart2);
			} else {
				num = 0;
				sens_count = 0;
				coolant_temp = 0;
			}
		} else { //иначе пробуем искать датчики снова
			if (OWI_SearchDevices(devices, MAX_DS18B20, &huart2, &num) == SEARCH_SUCCESSFUL){
				sens_count = num;
			}
		}
    osDelay(2000);
  }
  /* USER CODE END StartTempSen */
}

/* StartRmsCalc function */
void StartRmsCalc(void const * argument)
{
  /* USER CODE BEGIN StartRmsCalc */
	vSemaphoreCreateBinary(rmsCalcSemaphor);
	while (rmsCalcSemaphor == NULL){
		//HAL_TIM_Base_Start_IT(&htim6);
		HAL_GPIO_TogglePin(Rdy_Led_GPIO_Port, Rdy_Led_Pin);
		osDelay(500);
	}
	
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(rmsCalcSemaphor, portMAX_DELAY) == pdTRUE){
			
			CB_PlaceTail(AvgCS1, CB_GetRMS(CurentSensor1));
			CB_PlaceTail(AvgCS2, CB_GetRMS(CurentSensor2));
			
			CB_Flush(CurentSensor1);
			CB_Flush(CurentSensor2);
						
			R[RO_CS1] = CB_GetAVG(AvgCS1) / (CS_K_SEN / 100.0) * CS_GEN_COEFF; // 0.01 A
			cs2_avg_mv = CB_GetAVG(AvgCS2);
			R[RO_CS2] =  cs2_avg_mv / (CS_K_SEN / 100.0) * CS_GEN_COEFF; // 0.01 A
			
		}
		osDelay(5);
  }
  /* USER CODE END StartRmsCalc */
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
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM1 && adc_ready) { // need test
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, sizeof(adc_buf));
	}
	
	if (htim->Instance == TIM6) { // every 5 ms
			HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop(&htim6);
			//HAL_GPIO_TogglePin(OH_Led_GPIO_Port, OH_Led_Pin);
			//xSemaphoreGiveFromISR(rmsCalcSemaphor, &xHigherPriorityTaskWoken);
			//portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); // Переключаемся на задачу обработчик				
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
		HAL_GPIO_WritePin(Rdy_Led_GPIO_Port, Rdy_Led_Pin, GPIO_PIN_SET); 
		HAL_Delay(500);
		HAL_GPIO_WritePin(Rdy_Led_GPIO_Port, Rdy_Led_Pin, GPIO_PIN_RESET); 
		HAL_Delay(1000);
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
