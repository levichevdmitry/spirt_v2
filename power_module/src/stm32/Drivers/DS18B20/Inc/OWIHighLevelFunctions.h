#ifndef _OWI_ROM_FUNCTIONS_H_
#define _OWI_ROM_FUNCTIONS_H_

#include <string.h> // Used for memcpy.
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

typedef struct
{
    unsigned char id[8];    //!< The 64 bit identifier.
} OWI_device;

// статус возврата функций
#define OW_NO_DEVICE	0
#define OW_OK					1
#define OW_ERROR			2


// если нужно отдавать тики FreeRTOS, то раскомментировать
#define OW_GIVE_TICK_RTOS


#define SEARCH_SUCCESSFUL     					0x00
#define SEARCH_CRC_ERROR      					0x01
#define SEARCH_ERROR          					0xff
#define AT_FIRST              					0xff
#define SET_SETTINGS_SUCCESSFUL     		0x00
#define SET_SETTINGS_ERROR    					0x10

#define DS18B20_FAMILY_ID                0x28 
#define DS18B20_CONVERT_T                0x44
#define DS18B20_READ_SCRATCHPAD          0xbe
#define DS18B20_WRITE_SCRATCHPAD         0x4e
#define DS18B20_COPY_SCRATCHPAD          0x48
#define DS18B20_RECALL_E                 0xb8
#define DS18B20_READ_POWER_SUPPLY        0xb4

#define DS18B20_9BIT_RES      0x1F
#define DS18B20_10BIT_RES     0x3F
#define DS18B20_11BIT_RES     0x5F
#define DS18B20_12BIT_RES     0x7F

//void OWI_SendByte(unsigned char data, unsigned char pin);
uint8_t OW_Init(UART_HandleTypeDef * huart);
void OWI_SendByte(unsigned char data, UART_HandleTypeDef * huart);
//unsigned char OWI_ReceiveByte(unsigned char pin);
unsigned char OWI_ReceiveByte(UART_HandleTypeDef * huart);
void OWI_SkipRom(UART_HandleTypeDef * huart);
void OWI_ReadRom(unsigned char * romValue, UART_HandleTypeDef * huart);
void OWI_MatchRom(unsigned char * romValue, UART_HandleTypeDef * huart);
unsigned char OWI_SearchRom(unsigned char * bitPattern, unsigned char lastDeviation, UART_HandleTypeDef * huart);
unsigned char OWI_SearchDevices(OWI_device * devices, unsigned char numDevices, UART_HandleTypeDef * huart, unsigned char *num);
unsigned char FindFamily(unsigned char familyID, OWI_device * devices, unsigned char numDevices, unsigned char lastNum);
void StartAllConvert_T(UART_HandleTypeDef * huart);
float GetTemperatureSkipRom(UART_HandleTypeDef * huart);
float GetTemperatureMatchRom(unsigned char * romValue, UART_HandleTypeDef * huart);
unsigned char InitSensor(unsigned char * romValue, UART_HandleTypeDef * huart, signed char lowAlm, signed char hiAlm, unsigned char resolution);
uint8_t OWI_DetectPresence(UART_HandleTypeDef *huart);
#endif
