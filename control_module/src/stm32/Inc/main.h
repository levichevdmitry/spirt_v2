/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Power_brd_en_Pin GPIO_PIN_2
#define Power_brd_en_GPIO_Port GPIOE
#define Power_brd_rdy_Pin GPIO_PIN_3
#define Power_brd_rdy_GPIO_Port GPIOE
#define GPO1_Pin GPIO_PIN_4
#define GPO1_GPIO_Port GPIOE
#define GPI1_Pin GPIO_PIN_5
#define GPI1_GPIO_Port GPIOE
#define GPI2_Pin GPIO_PIN_6
#define GPI2_GPIO_Port GPIOE
#define Valve1_gate_Pin GPIO_PIN_0
#define Valve1_gate_GPIO_Port GPIOC
#define Valve2_gate_Pin GPIO_PIN_1
#define Valve2_gate_GPIO_Port GPIOC
#define Valve3_gate_Pin GPIO_PIN_2
#define Valve3_gate_GPIO_Port GPIOC
#define ESP8622_Boot_GPIO0_Pin GPIO_PIN_3
#define ESP8622_Boot_GPIO0_GPIO_Port GPIOC
#define P_in_Pin GPIO_PIN_6
#define P_in_GPIO_Port GPIOA
#define Beeper_Pin GPIO_PIN_4
#define Beeper_GPIO_Port GPIOC
#define TP_IRQ_Pin GPIO_PIN_5
#define TP_IRQ_GPIO_Port GPIOC
#define Valve4_gate_Pin GPIO_PIN_0
#define Valve4_gate_GPIO_Port GPIOB
#define Debug_LED_Pin GPIO_PIN_1
#define Debug_LED_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define Water_IRQ_Pin GPIO_PIN_6
#define Water_IRQ_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define ESP8622_GPIO2_Pin GPIO_PIN_11
#define ESP8622_GPIO2_GPIO_Port GPIOC
#define PWM_Led_Pin GPIO_PIN_8
#define PWM_Led_GPIO_Port GPIOB
#define Spirt_concentr_IRQ_Pin GPIO_PIN_1
#define Spirt_concentr_IRQ_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
