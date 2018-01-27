
#include "beeper.h"

uint8_t mode = BEEPER_MODE_OFF;
uint16_t _delay = 2000;
uint16_t _pulse	= 500;
uint16_t _period = 1000;
uint16_t tick = 0;

void BeeperOn(void) {
	
	mode = BEEPER_MODE_ON;
	HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_SET);
}

void BeeperOnDelay(uint16_t delay) {
	
	mode = BEEPER_MODE_DELAY;
	_delay = delay;
	HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_SET);
	tick = 0;
}

void BeeperOff(void) {
	
	mode = BEEPER_MODE_OFF;
	HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_RESET);
}

void BeeperPulse(uint16_t pulse, uint16_t period) {
	
	mode = BEEPER_MODE_PWM;
	_pulse = pulse;
	_period = period;
	HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_SET);
	tick = 0;
}

void BeeperPulseDelay(uint16_t pulse, uint16_t period, uint16_t delay) {
	
	mode = BEEPER_MODE_PWM_DELAY;
	_pulse = pulse;
	_period = period;
	_delay = delay;
	HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_SET);
	tick = 0;
}

void BeeperTick(void) {  // time base 1 ms
	
	tick ++;
	
	if (mode == BEEPER_MODE_DELAY || mode == BEEPER_MODE_PWM_DELAY) {
		if (tick >= _delay ) {
			BeeperOff();
		}
	} 
	if (mode == BEEPER_MODE_PWM || mode == BEEPER_MODE_PWM_DELAY) {
		if (tick % _period < _pulse) {
			HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(Beeper_GPIO_Port, Beeper_Pin, GPIO_PIN_RESET);
		}
	} 
	
}
