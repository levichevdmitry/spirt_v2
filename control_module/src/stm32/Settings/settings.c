
#include "settings.h"

settingsStruct Settings;
sensorValuesStruct Sensors;
globalStateStruct globalState;


//--------------------------------------------------------------
// Начальная инициализация регистров
//--------------------------------------------------------------
void InitRegs(void) {
	
	R[RW_SP_PWR_M] = 500;   																		// 50 %
	R[RW_SP_PWR_A] = 500;																				// 50 %
	R[RW_SP_T] = Settings.pwr_block_set.t_coolant.value;				// 20 C
	R[RW_R1] = Settings.pwr_block_set.R_heater1.value;					// 32.3 Om
	R[RW_R2] = Settings.pwr_block_set.R_heater2.value;					// 16.1 Om
	R[RW_KP] = Settings.pwr_block_set.Kp.value;									// Kp = 1.0;
	R[RW_KI] = Settings.pwr_block_set.Ki.value;									// Ki = 0.01
	R[RW_KD] = Settings.pwr_block_set.Kd.value;									// Kd = 0.0
	R[RW_CMD] = 0x0000;																					// CMD = 0
	
}

//--------------------------------------------------------------
// Функции работы с настройками
//--------------------------------------------------------------

void SaveSettings(void){
	EepromSaveObject((uint8_t*)&Settings, sizeof(Settings), 0);
}

void LoadSettings(void){
	EepromReadObject((uint8_t*)&Settings, sizeof(Settings), 0);
	// kill after configuration
	//Settings.flow_koeff.max_val = 20.0;// W! need erase
}

uint8_t CheckSettings(void) {
	uint16_t eepromVersion;
	EepromReadObject((uint8_t*)&eepromVersion, sizeof(eepromVersion), 0);
	if (eepromVersion != EEPROM_VER) {
		return EEPROM_VER_NEQ;
	}
	return EEPROM_VER_EQ;
}

void LoadDefSettings(void){
	uint8_t i;
	
	Settings.eeprom_ver = EEPROM_VER;
	
	for (i = 0; i < 8; i++) {
		Settings.id_t_kub.id[i] = 0;
		Settings.id_t_kolona_n.id[i] = 0;
		Settings.id_t_kolona_v.id[i] = 0;
		Settings.id_t_after_def.id[i] = 0;
	}
	
	// Lcd configuration
	
	Settings.lcd_conf.cali_A = -21485; 
	Settings.lcd_conf.cali_B = -100; 
	Settings.lcd_conf.cali_C = 82926165; 
	Settings.lcd_conf.cali_D = -50; 
	Settings.lcd_conf.cali_E = 14225; 
	Settings.lcd_conf.cali_F = -4866561;
	
	// pressure sensor
	
	Settings.adc_min.value = 500;
	Settings.adc_min.min_val = 0;
	Settings.adc_min.max_val = 2500;
	
	Settings.adc_max.value = 1800;
	Settings.adc_max.min_val = 0;
	Settings.adc_max.max_val = 2500;
	
	Settings.p_min.value = 0;
	Settings.p_min.min_val = -100;
	Settings.p_min.max_val = 1000;
	
	Settings.p_max.value = 75; 
	Settings.p_max.min_val = -100;
	Settings.p_max.max_val = 1000;
	
	Settings.flow_koeff.value = 0.55; 
	Settings.flow_koeff.min_val = 0.0;
	Settings.flow_koeff.max_val = 20.0;
	
	// Параметры силового блока
	// ==================================================
	Settings.pwr_block_set.t_coolant.value = 20;
	Settings.pwr_block_set.t_coolant.min_val = 0; 
	Settings.pwr_block_set.t_coolant.max_val = 40;
	
	Settings.pwr_block_set.R_heater1.value = 323;
	Settings.pwr_block_set.R_heater1.min_val = 1;
	Settings.pwr_block_set.R_heater1.max_val = 1000;
	
	Settings.pwr_block_set.R_heater2.value = 161;
	Settings.pwr_block_set.R_heater2.min_val = 1;
	Settings.pwr_block_set.R_heater2.max_val = 1000;
	
	Settings.pwr_block_set.Kp.value = 61; // 0.61
	Settings.pwr_block_set.Kp.min_val = 0;
	Settings.pwr_block_set.Kp.max_val = 10000;
	
	Settings.pwr_block_set.Ki.value = 1;
	Settings.pwr_block_set.Ki.min_val = 20; // 0.20
	Settings.pwr_block_set.Ki.max_val = 10000;
	
	Settings.pwr_block_set.Kd.value = 0;
	Settings.pwr_block_set.Kd.min_val = 0;
	Settings.pwr_block_set.Kd.max_val = 10000;
	
	Settings.pwr_block_set.Kp2.value = 100;
	Settings.pwr_block_set.Kp2.min_val = 0;
	Settings.pwr_block_set.Kp2.max_val = 10000;
	
	Settings.pwr_block_set.Ki2.value = 1;
	Settings.pwr_block_set.Ki2.min_val = 0;
	Settings.pwr_block_set.Ki2.max_val = 10000;
	
	Settings.pwr_block_set.Kd2.value = 0;
	Settings.pwr_block_set.Kd2.min_val = 0;
	Settings.pwr_block_set.Kd2.max_val = 10000;
	
	// Параметры пропарки
	// ==================================================
	Settings.clean_set.T_cleaning.value = 30; 			// minutes
	Settings.clean_set.T_cleaning.min_val = 0;
	Settings.clean_set.T_cleaning.max_val = 100;
	
	Settings.clean_set.t_open_valve.value = 100; 		// C
	Settings.clean_set.t_open_valve.min_val = 90;
	Settings.clean_set.t_open_valve.max_val = 110;
	
	// ==================================================
	// Параметры дистилляции
	Settings.dist_set.pow_heater.value = 500;  				// %
	Settings.dist_set.pow_heater.min_val = 0;  				//
	Settings.dist_set.pow_heater.max_val = 1000;  			//
	
	Settings.dist_set.Q_head.value = 300;
	Settings.dist_set.Q_head.min_val = 0;
	Settings.dist_set.Q_head.max_val = 1000;
	
	Settings.dist_set.t_kub_get_tail.value = 94;
	Settings.dist_set.t_kub_get_tail.min_val = 90;
	Settings.dist_set.t_kub_get_tail.max_val = 98;
	
	Settings.dist_set.t_kub_off.value = 99;
	Settings.dist_set.t_kub_off.min_val = 96;
	Settings.dist_set.t_kub_off.max_val = 100;
	
	Settings.dist_set.t_kub_V_down.value = 92;
	Settings.dist_set.t_kub_V_down.min_val = 90;
	Settings.dist_set.t_kub_V_down.max_val = 96;
	
	Settings.dist_set.V_get.value = 2100;
	Settings.dist_set.V_get.min_val = 1000;
	Settings.dist_set.V_get.max_val = 3000;
	// ==================================================
	// Параметры ректификации
	Settings.rect_set.dt.value = 0.45;
	Settings.rect_set.dt.min_val = 0.1;
	Settings.rect_set.dt.max_val = 2.0;
	
	Settings.rect_set.mode.value = 1;
	Settings.rect_set.mode.min_val = 0;
	Settings.rect_set.mode.max_val = 1;
	
	Settings.rect_set.pow_heater.value = 700;
	Settings.rect_set.pow_heater.min_val = 0;
	Settings.rect_set.pow_heater.max_val = 1000;
	
	Settings.rect_set.pow_svitek.value = 1;
	Settings.rect_set.pow_svitek.min_val = 0;
	Settings.rect_set.pow_svitek.max_val = 5;
	
	Settings.rect_set.p_kub.value = 20;
	Settings.rect_set.p_kub.min_val = 15;
	Settings.rect_set.p_kub.max_val = 30;
	
	Settings.rect_set.Q_head.value = 300;
	Settings.rect_set.Q_head.min_val = 50;
	Settings.rect_set.Q_head.max_val = 1000;
	
	Settings.rect_set.Q_sub_head.value = 300;
	Settings.rect_set.Q_sub_head.min_val = 50;
	Settings.rect_set.Q_sub_head.max_val = 1000;
	
	Settings.rect_set.T1.value = 1;
	Settings.rect_set.T1.min_val = 1;
	Settings.rect_set.T1.max_val = 10;
	
	Settings.rect_set.T2.value = 2;
	Settings.rect_set.T2.min_val = 1;
	Settings.rect_set.T2.max_val = 10;
	
	Settings.rect_set.t_kub_off.value = 98;
	Settings.rect_set.t_kub_off.min_val = 90;
	Settings.rect_set.t_kub_off.max_val = 100;
	
	Settings.rect_set.t_kub_ss.value = 92;
	Settings.rect_set.t_kub_ss.min_val = 88;
	Settings.rect_set.t_kub_ss.max_val = 96;
	
	Settings.rect_set.V_get.value = 1950;
	Settings.rect_set.V_get.min_val = 1000;
	Settings.rect_set.V_get.max_val = 3000;
	
}
