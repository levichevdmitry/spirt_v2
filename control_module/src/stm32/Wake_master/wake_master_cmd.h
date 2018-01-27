//----------------------------------------------------------------------------

//Модуль реализации выполнения команд: header file

//----------------------------------------------------------------------------

#ifndef _WAKE_CMD_H_
#define _WAKE_CMD_H_

#include <wake.h>

//-------------------------------- Константы: --------------------------------

//Коды специальных команд:

#define CMD_SETPAR     6 //установка 1-го параметра
#define CMD_GETPAR     7 //чтение 1-го параметра
#define CMD_SETPARS    8 //запись N параметров
#define CMD_GETPARS    9 //чтение N параметров

//-------------------------------- Переменные: -------------------------------

//extern uint8_t Exchange_Sta; //статус обмена с подчиненным устройством

//---------------------------- Прототипы функций: ----------------------------

void WakeMaster_Commands_Exe(void); //выполнение команд
uint8_t WakeMaster_Busy(void);
uint8_t WakeMaster_Timeout(void);
void WakeMaster_Tick(void);
void WakeMaster_GetParam(uint8_t index);
void WakeMaster_GetParams(uint8_t index, uint8_t count);
void WakeMaster_SetParam(uint8_t index);
void WakeMaster_SetParams(uint8_t index, uint8_t count);

//----------------------------------------------------------------------------

#endif
