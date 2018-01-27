
#include "utils.h"
#include "cmsis_os.h"


extern uint8_t stateMode, subMode; // ����� � �������� ������
extern xSemaphoreHandle shMode;

// 1-wire util function
inline void OneWireIdToStr(char * buf, const OWI_device * dev){
	 sprintf(buf, "%X%X%X%X%X%X%X%X", dev->id[0], dev->id[1], dev->id[2], dev->id[3], dev->id[4], dev->id[5], dev->id[6], dev->id[7]);
}

uint8_t OneWireIdEq(const OWI_device * dev1, const OWI_device * dev2){
	uint8_t i;
	for (i = 0; i < 8; i++){
		if (dev1->id[i] != dev2->id[i]) {
			return 0;
		}
	}
	return 1;
}

inline void OneWireIdCopy(OWI_device * dev1, const OWI_device * dev2) {
	memcpy(dev1, dev2, sizeof(OWI_device));
}
// =============================================================================
// mode util function
void SetStateMode(uint8_t _mode){
		if (shMode != NULL) {
			if(xSemaphoreTake(shMode, portMAX_DELAY) == pdTRUE){
				stateMode = _mode;
				xSemaphoreGive(shMode);
				
			}
		}
}

void SetSubMode(uint8_t _subMode){
		if (shMode != NULL) {
			if(xSemaphoreTake(shMode, portMAX_DELAY) == pdTRUE){
				subMode = _subMode;
				xSemaphoreGive(shMode);
				
			}
		}
}

uint8_t GetStateMode(void){
	uint8_t _mode;
		if (shMode != NULL) {
			if(xSemaphoreTake(shMode, portMAX_DELAY) == pdTRUE){
				_mode = stateMode;
				xSemaphoreGive(shMode);
			}
		}
		return _mode;
}

uint8_t GetSubMode(void){
	uint8_t _subMode;
		if (shMode != NULL) {
			if(xSemaphoreTake(shMode, portMAX_DELAY) == pdTRUE){
				_subMode = subMode;
				xSemaphoreGive(shMode);
			}
		}
		return _subMode;
}
// =============================================================================
// Time util function
void ResetTime(timeParam * tm) {
	tm->sec = 0;
	tm->min = 0;
	tm->hour = 0;
}

void TickTime(timeParam * tm) {
	tm->sec ++;
	if (tm->sec >= 60) {
		tm->min ++;
		tm->sec = 0;
	}
	if (tm->min >= 60) {
		tm->hour ++;
		tm->min = 0;
	}
}
// =============================================================================
// Rectification utils function

uint16_t CalcVBody(float t_kub, uint16_t V_nom, uint16_t ss_count) {
	float Vget;
	Vget = (float)V_nom * (1.0F - (float)ss_count * 0.03F);
	if (t_kub >= 84.0F) {
		Vget = (980.0F - 10.0F * t_kub) / 127 * Vget;
	} 
	return (uint16_t)Vget;
}

// exp filter

float ExpFilter(float x, float old_x, float k){
	return  x * k + (1.0F - k) * old_x;
}


// �������� ����������� � ������� ���������� �������
uint8_t TimeTempCheckGt(float t, float setpoint, uint8_t time_sp){ // ������ ���������� 1 ��� � �������
static uint8_t timer = 0;
	if (t >= setpoint) {
		timer ++;
	} else {
		timer = 0;
	}
	return (timer >= time_sp);
}


//=============================================================================
// rectification power control
/*
RectificationSetPower() - ������� ������� �������� ��� ������������
float t_kub  				- ������� ����������� � ����, �
float p_kub 				- ������� �������� � ����, �� ��. ��.
float p_kub_sp	 		- ������� �������� ��� �����������, �� ��. ��.
uint16_t add_pow	 	-	������� �������� �� ������ �� �������, %
uint16_t base_pow	 	-	������� ��������
uint8_t p_stab	 		- ����� ������������ 0- �������� 1- ��������
SPid * pid					- ��������� �� ��������� ��� ��� ���������� ��������
*/

void RectificationSetPower(float t_kub, float p_kub, float p_kub_sp, uint16_t add_pow, uint16_t base_pow, uint8_t p_stab, SPid * pid) {
	uint16_t power;
	
	if (p_stab) { // ������������ �� ��������
		power = UpdatePID(pid, p_kub_sp - p_kub, p_kub);
		Heater2On(CMD_MODE_MAN);
		SetPowerHeaterMan(power);
	} else { // ������������ �� �������� � �������������� �� �������
		if (t_kub > RECT_SVITEC_TEMP) { 
			power = (t_kub - RECT_SVITEC_TEMP) * add_pow * 10.0F + base_pow; // �������� �� 10 ��� ��������� ���������
			if (power > 1000) {
				power = 1000;
			}
		} else {
			power = base_pow;
		}
		#if RECT_POWER_CTRL == POWER_CONST
				Heater2On(CMD_MODE_AUTO);
				SetPowerHeaterAuto(power); // ����� ����������� ���������� ��������
		#else
				Heater2On(CMD_MODE_MAN);
				SetPowerHeaterMan(power); // ����� ������� ����������
		#endif
	}
}


//===============================================================================
// Safe automation control
/*
float p_kub 					- ������� �������� �������� � ����
uint8_t spirt_al 			- ������� ���������� ��� ����� ������ � �������
float coolant_flow 		- ������ ���� ����� ����������� (� ������� ������ �� ������������)
uint8_t pump_run			- ������� ������ ������ (� ������� ������ �� ������������)
*/
void SafeControl(float p_kub, uint8_t spirt_al, float coolant_flow, uint8_t pump_run){
static uint16_t timer;
	if (GetStateMode() == MODE_RUN_RECT || GetStateMode() == MODE_RUN_DIST || GetStateMode() == MODE_SET_VC) {
		if (p_kub >= PRESSURE_TRIG_LVL2 ) {  // ���������� ����
			SetSubMode(SMODE_EMSTOP);
			HAL_GPIO_WritePin(Power_brd_en_GPIO_Port, Power_brd_en_Pin, GPIO_PIN_RESET);	
			CoolingFanOff();
			CoolingPumpOff();	
		} else if (p_kub >= PRESSURE_TRIG_LVL1) { // ���������� ����� ��������, ��������� ��������
				SetSubMode(SMODE_EMSTOP);
		} else if (spirt_al){
			if (++timer >= SPIRT_AL_TIME_HYST) {  // ���������� ����� ��������, ��������� ��������
				SetSubMode(SMODE_EMSTOP);
				timer = 0;
			}
		} else {
			HAL_GPIO_WritePin(Power_brd_en_GPIO_Port, Power_brd_en_Pin, GPIO_PIN_SET);
			timer = 0;
		}
	} else {
		if (p_kub >= PRESSURE_TRIG_LVL2 || spirt_al) { // ��������� ���� �����, ���� ������� ����� �� �������, �� �������� ��� ������������ ���������
			//SetSubMode(SMODE_EMSTOP);
			HAL_GPIO_WritePin(Power_brd_en_GPIO_Port, Power_brd_en_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(Power_brd_en_GPIO_Port, Power_brd_en_Pin, GPIO_PIN_SET);
		}
	}
}
