#ifndef _VALVE_H_
#define _VALVE_H_

#include "stm32f4xx_hal.h"

//===============================================================================
// enum valves
//===============================================================================

#define VALVE_NONE				0
#define VALVE_1						1
#define VALVE_2						2
#define VALVE_3						3

//===============================================================================

void ValveTick(void);
void ValveOnPwm(uint8_t valve, uint16_t period, uint16_t pulse);
void ValveOffAll(void);
void ValveOn(uint8_t valve);
void ValveInit(void);
uint16_t ValveGetOpenCount(void);
void ValveResetOpenCounter(void);
void ValveChangePwm(uint16_t period, uint16_t pulse);
void ValvePause(void);
void ValveResume(void);
uint16_t ValveCalcPeriod(uint16_t V_get, float k, uint16_t pulse);

#endif
