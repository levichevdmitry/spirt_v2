
#include "eeprom.h"

I2C_HandleTypeDef* i2c_port;

void EepromInit(I2C_HandleTypeDef* i2cPort)
{
	//init the eeprom object
	i2c_port = i2cPort;
}

HAL_StatusTypeDef ReadEEPROM(uint16_t address, uint8_t* MemTarget,
		uint16_t Size)
{
	uint16_t Counter = 0;
	HAL_StatusTypeDef Result = HAL_OK;
	while (Counter < Size && Result == HAL_OK)
	{
		uint16_t Diff = Size - Counter;

		if (Diff >= EEPROM_MAXPKT)
		{
			//Multi-Byte
			Result = HAL_I2C_Mem_Read(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], EEPROM_MAXPKT, EEPROM_TIMEOUT);
			Counter += EEPROM_MAXPKT;
		}
		else
		{
			//and the remaining ones...low packet size
			Result = HAL_I2C_Mem_Read(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], Diff, EEPROM_TIMEOUT);
			Counter += Diff;
		}
		HAL_Delay(EEPROM_WRITE / 2);
	}
	return Result;
}

HAL_StatusTypeDef WriteEEPROM(uint16_t address, uint8_t* MemTarget,
		uint16_t Size)
{
	uint16_t Counter = 0;
	HAL_StatusTypeDef Result = HAL_OK;
	while (Counter < Size && Result == HAL_OK)
	{
		uint16_t Diff = Size - Counter;

		if (Diff >= EEPROM_MAXPKT)
		{
			//Multi-Byte
			Result = HAL_I2C_Mem_Write(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], EEPROM_MAXPKT, EEPROM_TIMEOUT);
			Counter += EEPROM_MAXPKT;
		}
		else
		{
			//and the remaining ones...low packet size
			Result = HAL_I2C_Mem_Write(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], Diff, EEPROM_TIMEOUT);
			Counter += Diff;
		}
		HAL_Delay(EEPROM_WRITE);
	}
	return Result;
}

//This is basically just a nicer wrapper
void EepromReadObject(uint8_t* settings, uint16_t Size, int section)
{
	//Read a screen settings object from eeprom
	ReadEEPROM(section * EEPROM_SECTIONSIZE, (uint8_t*) settings, Size);
}

void EepromSaveObject(uint8_t* settings, uint16_t Size, int section)
{
	WriteEEPROM(section * EEPROM_SECTIONSIZE, (uint8_t*) settings, Size);
}
