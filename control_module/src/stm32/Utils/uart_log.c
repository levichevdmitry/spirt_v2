
#include "uart_log.h"
#include "string.h"

UART_HandleTypeDef *huart_log;
char log_buf[LOG_BUFFER_SIZE];

// init 
void LogInit(UART_HandleTypeDef *huart){
	if (huart != NULL) {
		huart_log = huart;
	}
}

// print message
void LogPrint(const char * type, const char * cl, const char * msg){
	if (huart_log == NULL) {
		return;
	}
	sprintf(log_buf, "%s:\t%s:\t%s\n", type, cl, msg);
	HAL_UART_Transmit_IT(huart_log, (uint8_t *)log_buf, strlen(log_buf));
}

// Print information message
void LogPrintI(const char * cl, const char * msg){
	LogPrint("I", cl, msg);
}

// Print warning message
void LogPrintW(const char * cl, const char * msg){
	LogPrint("W", cl, msg);
}

// Print error message
void LogPrintE(const char * cl, const char * msg){
	LogPrint("E", cl, msg);
}
