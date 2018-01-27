
#ifndef 	_UTILS_H_
#define		_UTILS_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "OWIHighLevelFunctions.h"
#include "settings.h"
#include "pid.h"
#include "control.h"

uint8_t OneWireIdEq(const OWI_device * dev1, const OWI_device * dev2);
void OneWireIdToStr(char * buf, const OWI_device * dev);
void OneWireIdCopy(OWI_device * dev1, const OWI_device * dev2);
void SetStateMode(uint8_t _mode);
void SetSubMode(uint8_t _subMode);
uint8_t GetStateMode(void);
uint8_t GetSubMode(void);
void ResetTime(timeParam * tm);
void TickTime(timeParam * tm);
uint16_t CalcVBody(float t_kub, uint16_t V_nom, uint16_t ss_count);
float ExpFilter(float x, float old_x, float k);
uint8_t TimeTempCheckGt(float t, float setpoint, uint8_t time_sp);
void RectificationSetPower(float t_kub, float p_kub, float p_kub_sp, uint16_t add_pow, uint16_t base_pow, uint8_t p_stab, SPid * pid);
void SafeControl(float p_kub, uint8_t spirt_al, float coolant_flow, uint8_t pump_run);

#endif
