
#ifndef			_UART_LOG_H_
#define			_UART_LOG_H_

#include "stm32f4xx_hal.h"

#define 		LOG_BUFFER_SIZE 		64 // Размер буфера для сообщения

void LogInit(UART_HandleTypeDef *huart);
void LogPrintI(const char * cl, const char * msg);
void LogPrintW(const char * cl, const char * msg);
void LogPrintE(const char * cl, const char * msg);

#endif
