
#include "control.h"

/*
Функции управления силовым блоком
Потоко не безопасные
Необходимо добавить объект синхронизации доступа
*/ 

// Heater 1 on
void Heater1On(void) {
	R[RW_CMD] |= (1<<CMD_BIT_H1E);
}
// Heater 1 off
void Heater1Off(void) {
	R[RW_CMD] &= ~(1<<CMD_BIT_H1E);
}

// Heater 2 on and set mode manual/auto
void Heater2On(uint8_t mode) {
	R[RW_CMD] |= (1<<CMD_BIT_H2E);
	
	if (mode) {
		R[RW_CMD] |= (1<<CMD_BIT_H2M);
	} else {
		R[RW_CMD] &= ~(1<<CMD_BIT_H2M);
	}
}
// Heater 2 off
void Heater2Off(void) {
	R[RW_CMD] &= ~(1<<CMD_BIT_H2E);
}
// Mixer on
void MixerOn(void) {
	R[RW_CMD] |= (1<<CMD_BIT_ME);
}
// Mixer off
void MixerOff(void) {
	R[RW_CMD] &= ~(1<<CMD_BIT_ME);
}
// Pump on
void CoolingPumpOn(void) {
	R[RW_CMD] |= (1<<CMD_BIT_PE);
}
// Pump off
void CoolingPumpOff(void) {
	R[RW_CMD] &= ~(1<<CMD_BIT_PE);
}
// Fan on and set mode manual/auto
void CoolingFanOn(uint8_t mode) {
	
	R[RW_CMD] |= (1<<CMD_BIT_FE);
	if (mode) {
		R[RW_CMD] |= (1<<CMD_BIT_FM);
	} else {
		R[RW_CMD] &= ~(1<<CMD_BIT_FM);
	}
}
// Fan off
void CoolingFanOff(void) { 
	R[RW_CMD] &= ~(1<<CMD_BIT_FE);
}

// Reset CMD
void ResetCMD(void){
	R[RW_CMD] = 0;
}
//====================================================================
// set output power for heater 2 in manual mode
void SetPowerHeaterMan(uint16_t power) {
	R[RW_SP_PWR_M] = power;
}

// set output power for heater 2 in auto mode
void SetPowerHeaterAuto(uint16_t power) {
	R[RW_SP_PWR_A] = power;
}

void SetCoolingTemp(uint8_t t) {
	R[RW_SP_T] = t;
}

//====================================================================
// Get curent for Heater 1 in Amper
float GetCurentHeater1(void) {
	return (float)R[RO_CS1] / 100.0F;
}

// Get curent for Heater 2 in Amper
float GetCurentHeater2(void) {
	return (float)R[RO_CS2] / 100.0F;
}

// Get power for Heater 1 in 0.01 kW
float GetPowerHeater1(void) {
	return (float)R[RO_PWR1] / 100.0F;
}

// Get power for Heater 2 in 0.01 kW
float GetPowerHeater2(void) {
	return (float)R[RO_PWR2] / 100.0F;
}

// Get cooling pump state
uint8_t GetCoolingPumpState(void) {
	return R[RO_ETA] & (uint16_t)(1 << ETA_BIT_PS);
}
// Get level sensor state
uint8_t GetLevelSensorState(void) {
	return !(R[RO_ETA] & (uint16_t)(1 << ETA_BIT_LS));
}




