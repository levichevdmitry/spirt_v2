
#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include "stm32f4xx_hal.h"
#include "registers.h"
#include "OWIHighLevelFunctions.h"
#include "eeprom.h"

//=================================================================================
#define 	DEBUG							1					// on/off debug mode

#define   VERSION						"0.2.5"				// software version

#if DEBUG == 1
	#define 	SW_VERSION				VERSION"d" 		// view string software version for debug
#else
	#define 	SW_VERSION				VERSION				// view string software version for releese
#endif
//=================================================================================
#define 	MAX_DS18B20				5 				// max find sensors
//=================================================================================
#define 	VALVE_PULSE				500  			// default valve pulse 500 ms
#define		K_EXP							0.65F  		// k for exp filter
#define 	K_EXP_PRESSURE		0.9F			// k for pressure exp filter
#define 	DELTA_STABILIZE		0.15F			// ������ �������� ���������� ��� ������������ ������ � ������ �����������
#define		RECT_ADD_PWR			30				// ���������� �������� ������ 30 ��� ������ �� ���� � ������ ������������ � 0.1 %
#define 	RECT_SVITEC_TEMP	90.0F			// ����������� ��� ������� �������� �������� �������� ������������� �������� �� �������
#define 	TEMP_TIME_CHK			15				// ����� �������� ����������� �� ���������� ������� (������������ ��� �������� ����� ���������� �������� ������������)
//#define 	RESET_EEPROM  						// uncomment for reset EEPROM to default values every startup
//=================================================================================
#define		POWER_CONST				0
#define		POWER_MAN					1
// ����� ������� ����������� � ������������ ��� ������ ������������ ��������
#define 	RECT_POWER_CTRL		POWER_CONST
// ����� ������� ����������� � ����������� ��� ������ ������������ ��������
#define 	DIST_POWER_CTRL		POWER_CONST
//=================================================================================
// ��������� ������������
#define		PRESSURE_TRIG_LVL1		45.0F  	// off level by software command
#define		PRESSURE_TRIG_LVL2		50.0F		// off level by hardware command
#define		SPIRT_AL_TIME_HYST		10			// spirt concentracion alarm signal hystiresys time, seconds
//=================================================================================
// ��������� ������������ ��

#define 	HEATER2_TEST_MODE		POWER_CONST // ������������ ���� 2 � ������ ������ ��� ������ ����������� ��������

//=================================================================================
// screen enums
#define SCR_MAIN									0		// Main screen
#define SCR_TEST_SEL							1		// Test selection screen
#define SCR_TEST_EM								2		// Test execute mechanism
#define SCR_TEST_SEN							3		// Test sensors
#define SCR_SETTINGS_SEL					4		// Settings selection screen
#define SCR_SETTINGS_TS						5 	// Temperature sensors settings screen
#define SCR_SETTINGS_DIS					6		// Distilation settings screen
#define SCR_SETTINGS_RECT					7		// Rectificatin settings screen
#define SCR_RUN_DIS								8		// Work distilation screen
#define SCR_RUN_RECT							9		// Work rectification screen
#define SCR_RUN_CLEAR							10		// Work rectification screen
#define	SCR_SETTINGS_VC						11 	// Valve calibration screen
#define SCR_SETTINGS_PM						12	// Power module settings
#define SCR_SETTINGS_SEL_CAL			13	// Select calibration screen
#define	SCR_SETTINGS_PSC					14 	// Calibration pressure sensor screen
//=================================================================================
// Modes enums
#define MODE_NOTHING							0		// ����� �����������
#define MODE_INIT_SEN							1		// ����� ������������� �������� �����������
#define MODE_RUN_CLS							2		// ����� ������� �����
#define MODE_RUN_DIST							3		// ����� �����������
#define MODE_RUN_RECT							4		// ����� ������������
#define MODE_TEST_SEN							5   // ����� ������������ ��������
#define MODE_SET_VC								6   // ����� ���������� �������

// sub modes enums

#define SMODE_CLR_HEATING					0  // ����� ��������, ��� ����� - ��������
#define SMODE_CLR_CLEANING				1  // ����� ��������, ��� ����� - ������
#define SMODE_CLR_OPNVALVE				3  // ����� ��������, ��� ����� - ������ ��������
#define SMODE_CLR_EXIT						4  // ����� ��������, ��� ����� - ������ ��������

#define SMODE_VC_PREPARE					0	 // ���������� � ����������
#define SMODE_VC_CAL							1	 // ����������
#define SMODE_VC_ENDSAVE					2  // ��������� � ����������
#define SMODE_VC_STOP							3	 // �������� ����������
#define SMODE_VC_START_HEAT				4	 // ��������� ������

#define SMODE_DIST_PREPARE				0	// ����������
#define SMODE_DIST_PREHEAT				1	// ����������
#define SMODE_DIST_HEATING				2	// ��������	
#define SMODE_DIST_GET_HEAD				3	// ����� �����
#define SMODE_DIST_WORK_YOUSELF		4	// ������ �� ���� [X]
#define SMODE_DIST_GET_BODY				5	// ����� ����
#define SMODE_DIST_GET_TAIL				6	// ����� �������
#define SMODE_DIST_END						7	// ��������� �������� [X]
#define SMODE_DIST_GET_PAUSE			8	// ����� ������
#define SMODE_DIST_EXIT						9	// �����
//#define SMODE_DIST_EMSTOP					10// ��������� ���������

#define SMODE_RECT_PREPARE				0	// ����������
#define SMODE_RECT_PREHEAT				1	// ����������
#define SMODE_RECT_HEATING				2	// ��������	[X]
#define SMODE_RECT_WORK_YOUSELF1	3	// ������ �� ���� 1 [X]
#define SMODE_RECT_GET_HEAD				4	// ����� �����
#define SMODE_RECT_GET_SUBHEAD		5	// ����� �����
#define SMODE_RECT_WORK_YOUSELF2	6	// ������ �� ���� 2 [X]
#define SMODE_RECT_GET_BODY				7	// ����� ����
#define SMODE_RECT_GET_TAIL				8	// ����� �������
#define SMODE_RECT_END						9	// ��������� �������� [X]
#define SMODE_RECT_GET_PAUSE			10// ����� ������
#define SMODE_RECT_EXIT						11// �����
//#define SMODE_RECT_EMSTOP					12 // ��������� �������

#define SMODE_EMSTOP							255	// ��������� ��������� ��������

//=================================================================================
#define EEPROM_VER								0
#define EEPROM_VER_EQ							1
#define EEPROM_VER_NEQ						0
//=================================================================================
typedef struct {
	uint8_t value;
	uint8_t min_val;
	uint8_t max_val;
} byteParam;

typedef struct {
	uint16_t value;
	uint16_t min_val;
	uint16_t max_val;
} uintParam;

typedef struct {
	int16_t value;
	int16_t min_val;
	int16_t max_val;
} intParam;

typedef struct {
	float value;
	float min_val;
	float max_val;
} floatParam;

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
} timeParam;
//=================================================================================
// ��������� ������ ��� �������� �������� � �������� ��������
//=================================================================================

typedef struct {
	uintParam pow_heater;			// �������� ����������� ��� ���������� 70� � ����
	uintParam V_get;					// ���. �������� ������ �� ������ (2100 ��/�)
	uintParam Q_head;					// ����� �����
	byteParam t_kub_V_down;		// ����������� ��� �������� �������� ������ ���� � 2 ����
	byteParam t_kub_get_tail;	// ����������� ������ ������ �������
	byteParam t_kub_off;			// ����������� ���� ��� ��������� ��������
} distSettings;

typedef struct {
	byteParam mode;						// ����� ������������ ������ �� �������� ��� ��������.
	uintParam pow_heater;			// �������� ���� ����� 70� � ����
	byteParam pow_svitek;			// �������� �������� �� ������ �� �������, %
	byteParam p_kub;					// �������� � ������ (20 - 22 �� ��. ��.)
	floatParam dt;						// ������ (0,45 �)
	byteParam T1;							// ����� ������ ������ ����� �� ���� (1 ���) �� ����� �����
	byteParam T2;							// ����� ������ ������ ����� �� ���� (2 ���) �� ����� ����� ��� � � ���� > 92C
	byteParam t_kub_ss;				// ����������� ������ ������ �����/�����.
	byteParam t_kub_off;			// ����������� ���������� ������� ������������ (98�)
	uintParam Q_head;					// ��������� ����� �����, 300 ��
	uintParam Q_sub_head;			// ��������� ����� �������������, ��
	uintParam V_get;					// ����������� �������� ������ �������� �������, 1950 ��/�	
} rectSettings;

typedef struct {
	byteParam T_cleaning;					// ����� �������� ������
	byteParam t_open_valve;			  // ����������� ������ ���������� ��������
} cleanSettings;

typedef struct {
	byteParam t_coolant;					// ����������� ���� � ������� ����������
	uintParam R_heater1;					// ������������� ����������� 0,1 Om
	uintParam R_heater2;					// ������������� ����������� 0,1 Om
	uintParam Kp;									// Kp 0,01
	uintParam Ki;									// Ki 0,01
	uintParam Kd;									// Kd 0,01
	
	uintParam Kp2;								// Kp 0,01
	uintParam Ki2;								// Ki 0,01
	uintParam Kd2;								// Kd 0,01
	
} powerSettings;

typedef struct {
	int32_t cali_A; 
	int32_t cali_B; 
  int32_t cali_C; 
  int32_t cali_D; 
  int32_t cali_E; 
  int32_t cali_F;
} lcdConfig;


typedef struct {
	uint16_t eeprom_ver; 					// ������ eeprom ��� ����������
	OWI_device id_t_kub; 					// id ������� ����������� � ����
	OWI_device id_t_kolona_n;			// id ������� ����������� ������ ���
	OWI_device id_t_kolona_v;			// id ������� ����������� ������ ����
	OWI_device id_t_after_def;		// id ������� ����������� ������ ������������
	
	floatParam flow_koeff;				// ����������� ������� ����� ������ ��/�
	uintParam adc_min;						// ��� ��� ��� ������� ��������
	uintParam adc_max;						// ���� ��� ��� ������� ��������
	intParam	p_min;							// �������� �������
	intParam  p_max;							// �������� ��������
	
	lcdConfig lcd_conf;						// ��������� ���������� �������
	
	distSettings dist_set;				// ��������� �����������
	rectSettings rect_set;				// �������� ������������
	cleanSettings clean_set;			// �������� �������� ������
	powerSettings pwr_block_set;	// ��������� �������� �����
} settingsStruct;

typedef struct {
		float t_kub; 					// ����������� � ����
		float t_kolona_n; 		// ����������� ������ ���
		float t_kolona_v; 		// ����������� ������ ����
		float t_after_def;		// ����������� ���� ����� ������������
		float p_kub;					// �������� � ����, �� ��. ��.
		float heater1_pow;		// �������� �� ��� 1
		float heater2_pow;		// �������� �� ��� 2,3
		float heater1_cur;		// ���� ���� �� ��� 1
		float heater2_cur;		// ���� ���� �� ��� 2, 3
		uint8_t t_coolant;  	// ����������� ���� ����� ���������
		float f_coolant;			// ������ ���� ����� �����������
		uint8_t spirt_al;			// ���������� ������������ ������
		uint8_t f_coolant_al; // ��������� �������
		uint8_t l_coolant_al; // ��������� ������ ������ � �������������� �����
		uint16_t cpu_load;		// �������� ����������
} sensorValuesStruct;


typedef struct {
	uint8_t valve_num; // ����� ����������� �������
	uint8_t T_elapsed; // ����� �� ��������� ��������
} stateCleaning;

typedef struct {
	uint16_t pulseCount; // ���������� ����������� ���
	uint16_t Q_fact;		 // ���������� ���������� �����
	float Q_calc;		 // ����� ���������� �� ������ �����, ���������	
} stateValveCalibration;

typedef struct {
	float Q_fact; // ���������� ���������� ������ 
	uint16_t T_elapsed; // ���������� ����� ������ �� ����
	uint8_t lastSubMode; // ���������� ���������
	float V_get_now; // ������� �������� ������
	timeParam time;    // ����� �� ������ ��������
} stateDistillation;

typedef struct {
	float Q_fact; // ���������� ���������� ������
	uint16_t T_elapsed; // ���������� ����� ������ �� ����
	uint8_t lastSubMode; // ���������� ���������
	float V_get_now; // ������� �������� ������
	timeParam time;    // ����� �� ������ ��������
	uint16_t ss_count; // ���������� ������ ������
	uint8_t skip_head; // ������� ������ �����
} stateRectification;

typedef struct {
	stateCleaning clr_state;
	stateValveCalibration vlvclb_state;
	stateDistillation dist_state;
	stateRectification rect_state;
} globalStateStruct;

//=================================================================================

extern uint8_t sens_count;
extern uint16_t wakeMaster_timeout;
extern uint16_t pressure_capture;
extern settingsStruct Settings;
extern sensorValuesStruct Sensors;
extern globalStateStruct globalState;
extern OWI_device devices[MAX_DS18B20];
extern xSemaphoreHandle InitTempSen;

//=================================================================================

void InitRegs(void);
void SaveSettings(void);
void LoadSettings(void);
uint8_t CheckSettings(void);
void LoadDefSettings(void);

#endif
