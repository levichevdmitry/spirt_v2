
#ifndef	_CONTROL_H_
#define	_CONTROL_H_

#include "registers.h"
#include "stm32f4xx_hal.h"

#define CMD_MODE_MAN			0
#define CMD_MODE_AUTO			1


void Heater1On(void);
void Heater1Off(void);
void Heater2On(uint8_t mode);
void Heater2Off(void);
void MixerOn(void);
void MixerOff(void);
void CoolingPumpOn(void);
void CoolingPumpOff(void);
void CoolingFanOn(uint8_t mode);
void CoolingFanOff(void);
void ResetCMD(void);
//-----------------------------------

void SetPowerHeaterMan(uint16_t power);
void SetPowerHeaterAuto(uint16_t power);
void SetCoolingTemp(uint8_t t);

//-----------------------------------

float GetCurentHeater1(void);
float GetCurentHeater2(void);
float GetPowerHeater1(void);
float GetPowerHeater2(void);
uint8_t GetCoolingPumpState(void);
uint8_t GetLevelSensorState(void);
// ====================================================================================================================================


#endif
