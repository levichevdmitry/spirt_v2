
#ifndef _BEEPER_H_
#define _BEEPER_H_

#include "stm32f4xx_hal.h"

//===================================================================

#define BEEPER_MODE_OFF					0
#define BEEPER_MODE_ON					1
#define BEEPER_MODE_DELAY				2
#define BEEPER_MODE_PWM					3
#define BEEPER_MODE_PWM_DELAY		4
			
//===================================================================

void BeeperOn(void);
void BeeperOnDelay(uint16_t delay);
void BeeperOff(void);
void BeeperPulse(uint16_t pulse, uint16_t period);
void BeeperPulseDelay(uint16_t pulse, uint16_t period, uint16_t delay);
void BeeperTick(void);

#endif
