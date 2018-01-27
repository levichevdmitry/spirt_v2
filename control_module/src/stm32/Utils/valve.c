
#include "valve.h"

uint16_t _periodV = 1000;
uint16_t _pulseV = 500;
uint8_t _valve = VALVE_NONE;
uint8_t _valve_befor_pause = VALVE_NONE;
uint16_t tickV = 0;
uint16_t open_counter = 0;
uint16_t open_counter_befor_pause = 0;

void ValveInit(void){
	_periodV = 500;
	_pulseV = 500;
	tickV = 0;
	open_counter = 0;
	_valve = VALVE_NONE;
}

void ValveOn(uint8_t valve){
	
	_valve = VALVE_NONE;
	// off all valves
	HAL_GPIO_WritePin(Valve1_gate_GPIO_Port, Valve1_gate_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve2_gate_GPIO_Port, Valve2_gate_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve3_gate_GPIO_Port, Valve3_gate_Pin, GPIO_PIN_RESET);
	
	//open_counter = 0;
	
	switch (valve) {
		case VALVE_1:
				HAL_GPIO_WritePin(Valve1_gate_GPIO_Port, Valve1_gate_Pin, GPIO_PIN_SET);
			break;
		case VALVE_2:
				HAL_GPIO_WritePin(Valve2_gate_GPIO_Port, Valve2_gate_Pin, GPIO_PIN_SET);
			break;
		case VALVE_3:
				HAL_GPIO_WritePin(Valve3_gate_GPIO_Port, Valve3_gate_Pin, GPIO_PIN_SET);
			break;
	}	
}

void ValveOffAll(void){
	
	_valve = VALVE_NONE;
	HAL_GPIO_WritePin(Valve1_gate_GPIO_Port, Valve1_gate_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve2_gate_GPIO_Port, Valve2_gate_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve3_gate_GPIO_Port, Valve3_gate_Pin, GPIO_PIN_RESET);
}

void ValveOnPwm(uint8_t valve, uint16_t period, uint16_t pulse){
	
	ValveOffAll();
	_periodV = period;
	_pulseV = pulse;
	_valve = valve;
	tickV = 0;
	open_counter = 0;
}

void ValveChangePwm(uint16_t period, uint16_t pulse){
	_periodV = period;
	_pulseV = pulse;
}

uint16_t ValveGetOpenCount(void){
	return open_counter;
}

void ValveResetOpenCounter(void){
	open_counter = 0;
}

void ValvePause(void) {
	if (_valve != VALVE_NONE) {
		_valve_befor_pause = _valve;
		open_counter_befor_pause = open_counter;
		ValveOffAll();
	}
}

void ValveResume(void){
	if (_valve_befor_pause != VALVE_NONE) {
		_valve = _valve_befor_pause;
		_valve_befor_pause = VALVE_NONE;
		open_counter = open_counter_befor_pause;
	}
}

void ValveTick(void){ // tick base 1 ms
	
GPIO_PinState pinState;
	
	if (_valve != VALVE_NONE) {
		tickV ++;
		if (tickV > _periodV) {
			tickV = 0;
			open_counter ++;
		}
		if (tickV <= _pulseV) {
			pinState = GPIO_PIN_SET;
		} else {
			pinState = GPIO_PIN_RESET;
		}
		switch (_valve) {
		case VALVE_1:
				HAL_GPIO_WritePin(Valve1_gate_GPIO_Port, Valve1_gate_Pin, pinState);
			break;
		case VALVE_2:
				HAL_GPIO_WritePin(Valve2_gate_GPIO_Port, Valve2_gate_Pin, pinState);
			break;
		case VALVE_3:
				HAL_GPIO_WritePin(Valve3_gate_GPIO_Port, Valve3_gate_Pin, pinState);
			break;
		}	
	}
}

//=========================================================================
// Calculate period

uint16_t ValveCalcPeriod(uint16_t V_get, float k, uint16_t pulse) {
	float k_flow;
	uint32_t dose_count, time_full_dose;
	uint32_t time_pause;
	uint16_t period;
	 
	k_flow = (k * (float)pulse) / 1000.0F; // приводим все к мс
	dose_count = ((float)V_get / k_flow); // считем количество доз
	
	time_full_dose = dose_count * pulse; // считаем общее время открытого состояния клапана из 1 одного часа
	if (time_full_dose >= 3600000) { // если пропускная способность клапана меньше чем необходимая, то открываем на полную
		return pulse + 1;
	}
	time_pause = 3600000 - time_full_dose; // считем общее время паузы
	period = time_pause / dose_count + pulse; // считаем период
	if (period > pulse) {	
		return period;
	} else {
		return pulse + 1;
	}
}

