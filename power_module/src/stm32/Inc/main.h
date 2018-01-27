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

#define ZQ_IRQ_Pin GPIO_PIN_15
#define ZQ_IRQ_GPIO_Port GPIOC
#define CS1_Pin GPIO_PIN_0
#define CS1_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_1
#define CS2_GPIO_Port GPIOA
#define Water_Level_Pin GPIO_PIN_4
#define Water_Level_GPIO_Port GPIOA
#define DI1_Pin GPIO_PIN_5
#define DI1_GPIO_Port GPIOA
#define DI2_Pin GPIO_PIN_6
#define DI2_GPIO_Port GPIOA
#define OH_Led_Pin GPIO_PIN_12
#define OH_Led_GPIO_Port GPIOB
#define Link_Led_Pin GPIO_PIN_13
#define Link_Led_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_14
#define FAN_GPIO_Port GPIOB
#define Rdy_Led_Pin GPIO_PIN_15
#define Rdy_Led_GPIO_Port GPIOB
#define OH_Pin GPIO_PIN_8
#define OH_GPIO_Port GPIOA
#define Rdy_Pin GPIO_PIN_11
#define Rdy_GPIO_Port GPIOA
#define En_Pin GPIO_PIN_12
#define En_GPIO_Port GPIOA
#define G5_Pin GPIO_PIN_3
#define G5_GPIO_Port GPIOB
#define G4_Pin GPIO_PIN_4
#define G4_GPIO_Port GPIOB
#define G3_Pin GPIO_PIN_5
#define G3_GPIO_Port GPIOB
#define G2_Pin GPIO_PIN_6
#define G2_GPIO_Port GPIOB
#define G1_Pin GPIO_PIN_7
#define G1_GPIO_Port GPIOB
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
