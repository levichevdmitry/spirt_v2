#ifndef _XPT2046_H
#define _XPT2046_H

#include "stm32f4xx_hal.h"
#include "GUI.h"
#include "lcd_init.h"


#define XPT_CS_ON				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)
#define XPT_CS_OFF			HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)
#define XPT_IRQ_STATE 	HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin)

void touchGetSense(SPI_HandleTypeDef *hspi, uint32_t * x, uint32_t * y);
uint8_t getTouchState(void);
void TouchCalibrate3Points ( SPI_HandleTypeDef *hspi );
void TouchSetCoeff(int32_t A, int32_t B, int32_t C, int32_t D, int32_t E, int32_t F);
void TouchGetCoeff(int32_t *A, int32_t *B, int32_t *C, int32_t *D, int32_t *E, int32_t *F);

#endif
