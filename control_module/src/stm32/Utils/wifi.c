/*
	utils for	ESP8622 wi-fi module
*/

#include "wifi.h"
#include "uart_log.h"

UART_HandleTypeDef * wifi_huart;
char buffer[256];
uint8_t rx_buf;
uint8_t ip[19] = {'I','P',':','0','0','0','.','0',
									'0','0','.','0','0','0','.','0','0','0','\0'};
uint8_t data_buffer[20];
uint8_t buf_pointer;

void WifiInit (UART_HandleTypeDef *huart){
	if (huart != NULL) {
		wifi_huart = huart;
		HAL_UART_Receive_IT(huart, &rx_buf, sizeof(rx_buf));
		buf_pointer = 0;
	}
}

void WifiSendStatus(void) {
	
	uint16_t v_get, q_fact;
  char tm_buf_elapsed[6], tm_buf[9];	

	if (HAL_UART_GetState(wifi_huart) == HAL_UART_STATE_BUSY_TX) {
		//HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin);
		return;
	}
	
	if (GetStateMode() == MODE_RUN_RECT) {
			sprintf(tm_buf_elapsed, "%d", globalState.rect_state.T_elapsed);
			sprintf(tm_buf, "\"%d:%d:%d\"", globalState.rect_state.time.hour, globalState.rect_state.time.min, globalState.rect_state.time.sec);
			v_get = globalState.rect_state.V_get_now;
		  q_fact = globalState.rect_state.Q_fact;
	} else if (GetStateMode() == MODE_RUN_DIST) {
			sprintf(tm_buf_elapsed, "%i", globalState.dist_state.T_elapsed);
			sprintf(tm_buf, "\"%d:%d:%d\"", globalState.dist_state.time.hour, globalState.dist_state.time.min, globalState.dist_state.time.sec);
			v_get = globalState.dist_state.V_get_now;
		  q_fact = globalState.dist_state.Q_fact;
	} else {
			sprintf(tm_buf_elapsed, "%i", 0);
			sprintf(tm_buf, "\"%d:%d:%d\"", 0, 0, 0);
			v_get = 0;
		  q_fact = 0;
	}
	
	sprintf(buffer, "{\"mode\":%d,\n\
\"smode\":%d,\n\
\"time\":%s,\n\
\"t_elapsed\":%s,\n\
\"q_fact\":%d,\n\
\"v_get\":%d,\n\
\"t_kub\":%0.2f,\n\
\"p_kub\":%0.2f,\n\
\"t_koln\":%0.2f,\n\
\"t_kolv\":%0.2f,\n\
\"t_oj\":%d,\n\
\"f_oj\":%0.1f,\n\
\"pwr\":%0.2f,\n\
\"alcopdk\":%d,\n\
\"stop_cnt\":%d,\n\
\"eta\":%d\n}",
										GetStateMode(),
										GetSubMode(),
										tm_buf,
										tm_buf_elapsed,
										q_fact,
										v_get,
										Sensors.t_kub,
										Sensors.p_kub,
										Sensors.t_kolona_n,
										Sensors.t_kolona_v,
										Sensors.t_coolant,
										Sensors.f_coolant,
										Sensors.heater1_pow + Sensors.heater2_pow,
										Sensors.spirt_al,
										globalState.rect_state.ss_count,
										R[RO_ETA]
										);
										
	HAL_UART_Transmit_DMA(wifi_huart, (uint8_t *)buffer, strlen(buffer));
	
}

char * WifiGetIP(void){
	return (char *)ip;	
}

void WifiOnRx(void){
	uint8_t error_flags = wifi_huart->ErrorCode; 		//чтение флагов ошибок
	uint8_t data_byte = rx_buf;               			//чтение данных
	HAL_UART_Receive_IT(wifi_huart, &rx_buf, sizeof(rx_buf));
	
  if(error_flags != 0)                     //если обнаружены ошибки при приеме байта
  {
		buf_pointer = 0;
		return;
	}
	
	switch(buf_pointer){
		case 0: 
				if (data_byte == 'I'){
					data_buffer[buf_pointer] = data_byte;
					buf_pointer++;
				} else {
					buf_pointer = 0;
				}
			break;
		case 1: 
				if (data_byte == 'P'){
					data_buffer[buf_pointer] = data_byte;
					buf_pointer++;
				} else {
					buf_pointer = 0;
				}
			break;
		case 2: 
				if (data_byte == ':'){
					data_buffer[buf_pointer] = data_byte;
					buf_pointer++;
				} else {
					buf_pointer = 0;
				}
			break;
		default:
				if (buf_pointer >= sizeof(data_buffer)){
					buf_pointer = 0;
					break;
				}
				if (data_byte == '\n') {
					data_buffer[buf_pointer] = '\0';
					strcpy((char *)ip, (char *)data_buffer); 
					buf_pointer = 0;
				} else {
					data_buffer[buf_pointer++] = data_byte;
				}
	}
	
}



