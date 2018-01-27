/*
utils for	ESP8622 wi-fi module

*/

#ifndef		_WIFI_H_
#define		_WIFI_H_

#include "stm32f4xx_hal.h"
#include "settings.h"
#include "utils.h"

void WifiInit(UART_HandleTypeDef *huart);
void WifiSendStatus(void);
void WifiOnRx(void);
char * WifiGetIP(void);

#endif
