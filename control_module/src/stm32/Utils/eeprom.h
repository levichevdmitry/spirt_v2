
#ifndef		_EEPROM_H_
#define 	_EEPROM_H_

#include "stm32f4xx_hal.h"

#define EEPROM_ADDRESS            0xA0
#define EEPROM_MAXPKT             32              	//(page size)
#define EEPROM_WRITE              10              	//time to wait in ms
#define EEPROM_TIMEOUT            5 * EEPROM_WRITE  //timeout while writing
#define EEPROM_SECTIONSIZE				64

void EepromInit(I2C_HandleTypeDef* i2cPort);
void EepromReadObject(uint8_t* settings, uint16_t Size, int section);
void EepromSaveObject(uint8_t* settings, uint16_t Size, int section);


#endif
