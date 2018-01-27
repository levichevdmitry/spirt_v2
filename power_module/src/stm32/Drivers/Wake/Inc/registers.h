/*===============================================================

File: registers.h 
Version 1.1
Date 21.02.17
Author Levichev Dmitriy aka Nikopol

===============================================================*/

#ifndef _REGISTERS_H_
#define _REGISTERS_H_

#include <wake.h>

#define REG_COUNT				32
#define REG_WR_START		16

//---------------------------------------------------------------
// error codes
//---------------------------------------------------------------
#define  	REG_NOERR			0   // no errror
#define  	REG_INDERR 		1   // error index > REG_COUNT
#define  	REG_ORERR			2		// error cant write to index < REG_WR_START
//---------------------------------------------------------------
// indexes
//---------------------------------------------------------------
#define		RO_HSV				0  	// HW and SW versions
#define		RO_CS1				1		// current sensor 1 value, 0.1 A
#define		RO_CS2				2		// current sensor 2 value, 0.1 A
#define		RO_PWR1				3		// active power 1, 0.1 Vt
#define		RO_PWR2				4		// active power 2, 0.1 Vt
#define		RO_TMP1				5		// cooling system tempreture, 1 C
#define		RO_CPU_TMP		6		// cpu tempreture, 1 C
#define		RO_CPU_LOD		7		// cpu load, 1 %
#define		RO_ETA				15	// Status word

#define		RW_SP_PWR_M		16	// setpoint power for manual mode, 0.1 %
#define		RW_SP_PWR_A		17	// setpoint power for automatic mode, 0.1 %
#define		RW_SP_T				18	// setpoint temperature for cooling system, 1 C
#define		RW_R1					19	// resistant for heater 1, 0.1 Om
#define		RW_R2					20	// resistant for heater 2, 0.1 Om
#define		RW_KP					21	// Kp for PID,  0.01
#define		RW_KI					22	// Ki for PID,  0.01
#define		RW_KD					23	// Kd for PID,  0.01
#define		RW_CMD				31	// command word

//---------------------------------------------------------------
// CMD bits
//---------------------------------------------------------------
#define CMD_BIT_H1E			0		// Heater 1 enable
#define CMD_BIT_H2E			1		// Heater (PID) 2 eneble 
#define CMD_BIT_H2M			2		// Heater 2 mode
#define CMD_BIT_ME			3		// Mixer enable
#define CMD_BIT_PE			4		// Pump enable
#define CMD_BIT_FE			5		// Fan eneble
#define CMD_BIT_FM			6		// Fan mode
//---------------------------------------------------------------
// ETA bits
//---------------------------------------------------------------
#define ETA_BIT_MR			0		// Main relay state
#define ETA_BIT_OH			1		// Over heat sensor
#define ETA_BIT_TS			2		// Temperature sensor connection state
#define ETA_BIT_LS			3		// Level sensor state
#define ETA_BIT_HM			4		// Heater mode
#define ETA_BIT_MS			5		// Mixer state	
#define ETA_BIT_PS			6		// Pump state
#define ETA_BIT_FS			7		// Fan state

//---------------------------------------------------------------
// register
//---------------------------------------------------------------
extern uint16_t R[REG_COUNT];  // Регистры для чтения/записи параметров

//---------------------------------------------------------------
// function
//---------------------------------------------------------------
uint8_t read_reg(uint8_t index, uint8_t * low_byte, uint8_t * hi_byte);
uint8_t write_reg(uint8_t index, uint8_t low_byte, uint8_t hi_byte);
uint8_t read_n_reg(uint8_t index, uint8_t *count, uint8_t * reg_bytes);
uint8_t write_n_reg(uint8_t index, uint8_t count, uint8_t * reg_bytes);

#endif
