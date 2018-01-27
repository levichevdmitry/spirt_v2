
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
#define 	DELTA_STABILIZE		0.15F			// Дельта разности температур при стабилизации колоны в режиме ректификции
#define		RECT_ADD_PWR			30				// Добавочная мощность спустя 30 мин работы на себя в режиме ректификации в 0.1 %
#define 	RECT_SVITEC_TEMP	90.0F			// Температура при которой начинает работать алгоритм корректировки мощности по Свитеку
#define 	TEMP_TIME_CHK			15				// Время проверки температуры на превышение уставки (используется для перехода между итерациями процесса ректификации)
//#define 	RESET_EEPROM  						// uncomment for reset EEPROM to default values every startup
//=================================================================================
#define		POWER_CONST				0
#define		POWER_MAN					1
// Режим запуска нагревателя в ректификации при выборе стабилизации мощности
#define 	RECT_POWER_CTRL		POWER_CONST
// Режим запуска нагревателя в дистилляции при выборе стабилизации мощности
#define 	DIST_POWER_CTRL		POWER_CONST
//=================================================================================
// Настройки безопасности
#define		PRESSURE_TRIG_LVL1		45.0F  	// off level by software command
#define		PRESSURE_TRIG_LVL2		50.0F		// off level by hardware command
#define		SPIRT_AL_TIME_HYST		10			// spirt concentracion alarm signal hystiresys time, seconds
//=================================================================================
// Настройки тестирования ИМ

#define 	HEATER2_TEST_MODE		POWER_CONST // Тестирование ТЭНа 2 в ручном режиме или режиме поддержания мощности

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
#define MODE_NOTHING							0		// Режим бездействия
#define MODE_INIT_SEN							1		// Режим инициализации датчиков температуры
#define MODE_RUN_CLS							2		// Режим очистки паром
#define MODE_RUN_DIST							3		// Режим дистилляции
#define MODE_RUN_RECT							4		// Режим ректификации
#define MODE_TEST_SEN							5   // Режим тестирования датчиков
#define MODE_SET_VC								6   // Режим калибровки клапана

// sub modes enums

#define SMODE_CLR_HEATING					0  // Режим пропарки, под режим - разогрев
#define SMODE_CLR_CLEANING				1  // Режим пропарки, под режим - чистка
#define SMODE_CLR_OPNVALVE				3  // Режим пропарки, под режим - чистка клапанов
#define SMODE_CLR_EXIT						4  // Режим пропарки, под режим - чистка клапанов

#define SMODE_VC_PREPARE					0	 // Подготовка к калибровке
#define SMODE_VC_CAL							1	 // Калибровка
#define SMODE_VC_ENDSAVE					2  // Окончание и сохранение
#define SMODE_VC_STOP							3	 // Прервать калибровку
#define SMODE_VC_START_HEAT				4	 // Запустить нагрев

#define SMODE_DIST_PREPARE				0	// Подготовка
#define SMODE_DIST_PREHEAT				1	// Преднагрев
#define SMODE_DIST_HEATING				2	// Разогрев	
#define SMODE_DIST_GET_HEAD				3	// Отбор голов
#define SMODE_DIST_WORK_YOUSELF		4	// Работа на себя [X]
#define SMODE_DIST_GET_BODY				5	// Отбор тела
#define SMODE_DIST_GET_TAIL				6	// Отбор хвостов
#define SMODE_DIST_END						7	// Окончание процесса [X]
#define SMODE_DIST_GET_PAUSE			8	// Пауза отбора
#define SMODE_DIST_EXIT						9	// Выход
//#define SMODE_DIST_EMSTOP					10// Аварийная остановка

#define SMODE_RECT_PREPARE				0	// Подготовка
#define SMODE_RECT_PREHEAT				1	// Преднагрев
#define SMODE_RECT_HEATING				2	// Разогрев	[X]
#define SMODE_RECT_WORK_YOUSELF1	3	// Работа на себя 1 [X]
#define SMODE_RECT_GET_HEAD				4	// Отбор голов
#define SMODE_RECT_GET_SUBHEAD		5	// Отбор голов
#define SMODE_RECT_WORK_YOUSELF2	6	// Работа на себя 2 [X]
#define SMODE_RECT_GET_BODY				7	// Отбор тела
#define SMODE_RECT_GET_TAIL				8	// Отбор хвостов
#define SMODE_RECT_END						9	// Окончание процесса [X]
#define SMODE_RECT_GET_PAUSE			10// Пауза отбора
#define SMODE_RECT_EXIT						11// Выход
//#define SMODE_RECT_EMSTOP					12 // Аварийный останов

#define SMODE_EMSTOP							255	// Аварийная остановка процесса

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
// Структуры данных для хранения настроек и значений датчиков
//=================================================================================

typedef struct {
	uintParam pow_heater;			// Мощность нагревателя при достижении 70С в кубе
	uintParam V_get;					// Ном. скорость отбора из колоны (2100 мл/ч)
	uintParam Q_head;					// Объем голов
	byteParam t_kub_V_down;		// Температура для снижения скорости отбора тела в 2 раза
	byteParam t_kub_get_tail;	// Температура начала отбора хвостов
	byteParam t_kub_off;			// Температура куба для окончания процесса
} distSettings;

typedef struct {
	byteParam mode;						// Режим стабилизации колоны по мощности или давлению.
	uintParam pow_heater;			// Мощность ТЭНа после 70С в кубе
	byteParam pow_svitek;			// Прибавка мощности на градус по Свитеку, %
	byteParam p_kub;					// Давление в колоне (20 - 22 мм рт. ст.)
	floatParam dt;						// Дельта (0,45 С)
	byteParam T1;							// Время работы колоны самой на себя (1 мин) во время стопа
	byteParam T2;							// Время работы колоны самой на себя (2 мин) во время стопа при Т в кубе > 92C
	byteParam t_kub_ss;				// Температура начала работы старт/стопа.
	byteParam t_kub_off;			// Температура отключения процеса ректификации (98С)
	uintParam Q_head;					// Расчетный объем голов, 300 мл
	uintParam Q_sub_head;			// Расчетный объем подголовников, мл
	uintParam V_get;					// Номинальная скорость отбора питьевой фракции, 1950 мл/ч	
} rectSettings;

typedef struct {
	byteParam T_cleaning;					// Время пропарки колоны
	byteParam t_open_valve;			  // Температура начала открывания клапанов
} cleanSettings;

typedef struct {
	byteParam t_coolant;					// Температура воды в контуре охлаждения
	uintParam R_heater1;					// Сопротивление нагревателя 0,1 Om
	uintParam R_heater2;					// Сопротивление нагревателя 0,1 Om
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
	uint16_t eeprom_ver; 					// Версия eeprom для обновления
	OWI_device id_t_kub; 					// id датчика температуры в кубе
	OWI_device id_t_kolona_n;			// id датчика температуры колоны них
	OWI_device id_t_kolona_v;			// id датчика температуры колоны верх
	OWI_device id_t_after_def;		// id датчика температуры послле дефлегматора
	
	floatParam flow_koeff;				// коэффициент расхода через клапан мл/с
	uintParam adc_min;						// мин АЦП для датчика давления
	uintParam adc_max;						// макс АЦП для датчика давления
	intParam	p_min;							// давление минимум
	intParam  p_max;							// давление максимум
	
	lcdConfig lcd_conf;						// Параметры калибровки дисплея
	
	distSettings dist_set;				// Настройки дистилляции
	rectSettings rect_set;				// Настроки ректификации
	cleanSettings clean_set;			// Настроки пропарки колоны
	powerSettings pwr_block_set;	// Параметры силового блока
} settingsStruct;

typedef struct {
		float t_kub; 					// Температура в кубе
		float t_kolona_n; 		// Температура колона низ
		float t_kolona_v; 		// Температура колона верх
		float t_after_def;		// Температура воды после дефлегматора
		float p_kub;					// Давление в кубе, мм рт. ст.
		float heater1_pow;		// Мощность на ТЭН 1
		float heater2_pow;		// Мощность на ТЭН 2,3
		float heater1_cur;		// Сила тока на ТЭН 1
		float heater2_cur;		// Сила тока на ТЭН 2, 3
		uint8_t t_coolant;  	// Температура воды после радиатора
		float f_coolant;			// Расход воды через дефлегматор
		uint8_t spirt_al;			// Превышение концентрации спирта
		uint8_t f_coolant_al; // Отсутсвие расхода
		uint8_t l_coolant_al; // Состояние датчка уровня в расширительном бачке
		uint16_t cpu_load;		// загрузка процессора
} sensorValuesStruct;


typedef struct {
	uint8_t valve_num; // номер включаемого клапана
	uint8_t T_elapsed; // время до окончания процесса
} stateCleaning;

typedef struct {
	uint16_t pulseCount; // Количество необходимых доз
	uint16_t Q_fact;		 // Фактически отобранный объем
	float Q_calc;		 // Объем отобранный на данном этапе, расчетный	
} stateValveCalibration;

typedef struct {
	float Q_fact; // фактически отобранные головы 
	uint16_t T_elapsed; // оставшееся время работы на себя
	uint8_t lastSubMode; // Предидущее состояние
	float V_get_now; // текущая скорость отбора
	timeParam time;    // Время от начала процесса
} stateDistillation;

typedef struct {
	float Q_fact; // фактически отобранные головы
	uint16_t T_elapsed; // оставшееся время работы на себя
	uint8_t lastSubMode; // Предидущее состояние
	float V_get_now; // текущая скорость отбора
	timeParam time;    // Время от начала процесса
	uint16_t ss_count; // Количество стопов колоны
	uint8_t skip_head; // Пропуск отбора голов
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
