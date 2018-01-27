//----------------------------------------------------------------------------

//Модуль реализации выполнения команд

//----------------------------------------------------------------------------

#include <wake_master_cmd.h>
#include <wake_master_exchange.h>
#include <wake.h>
#include <registers.h>

//------------------------------ Константы: ----------------------------------


//----------------------------- Переменные: ----------------------------------

uint8_t Exchange_Sta = EXCH_IDLE; //статус обмена с подчиненным устройством
uint16_t Exchange_Timeout = 500; // ms
uint16_t timer = 0;
uint8_t lastErrCode = 0;

//--------------------------- Прототипы функций: -----------------------------

void TxMaster_Replay(char n, char err); //передача ответа на команду

//---------------------- Передача ответа на команду: -------------------------

void TxMaster_Replay(char n, char err)
{
  Tx_Ma_Nbt = n;                     //количество байт
  Tx_Ma_Dat[0] = err;                //код ошибка
  Tx_Ma_Cmd = Master_Command;               //команда
  WakeMaster_Tx();                      //инициализация передачи
  Master_Command = CMD_NOP;              //команда обработана
}

//----------------------- Получение статуса запроса --------------------------

uint8_t WakeMaster_Busy(void){
	if (Exchange_Sta != EXCH_IDLE) { 
		return 1;
	} else {
		return 0;
	}
}

uint8_t WakeMaster_Timeout(void){
	if (timer > Exchange_Timeout){
		return 1;
	} else {
		return 0;
	}
}

//------------------------- Счетчик тиков для таймаута ------------------------

void WakeMaster_Tick(void){
	
	if (Exchange_Sta != EXCH_IDLE){
		if (timer <= Exchange_Timeout) {
			timer ++;
		} else {
			Exchange_Sta = EXCH_IDLE; // если кончился таймаут, то переходим в режим ожидания
		}
	} else {
		//timer = 0;
	}
}

//------------------------- Request functions --------------------------------

void WakeMaster_GetParam(uint8_t index){
	Tx_Ma_Dat[0] = index;
	Exchange_Sta = EXCH_REQUEST;
	Master_Command = CMD_GETPAR;
}

void WakeMaster_GetParams(uint8_t index, uint8_t count){
	if (count > (FRAME - 2) / 2){
		count = (FRAME - 2) / 2;
	}
	Tx_Ma_Dat[0] = index;
	Tx_Ma_Dat[1] = count;
	Exchange_Sta = EXCH_REQUEST;
	Master_Command = CMD_GETPARS;
}

void WakeMaster_SetParam(uint8_t index){
	Tx_Ma_Dat[0] = index;
	Exchange_Sta = EXCH_REQUEST;
	Master_Command = CMD_SETPAR;
}

void WakeMaster_SetParams(uint8_t index, uint8_t count){
	if (count > (FRAME - 2) / 2){
		count = (FRAME - 2) / 2;
	}
	Tx_Ma_Dat[0] = index;
	Tx_Ma_Dat[1] = count;
	Exchange_Sta = EXCH_REQUEST;
	Master_Command = CMD_SETPARS;
}

//---------------------------- Выполнение команд: ----------------------------

void WakeMaster_Commands_Exe(void)
{
  //char i;
  switch(Master_Command)
  {
  case CMD_ERR: //обработка ошибки
    {
			lastErrCode = ERR_TX;
			Master_Command = CMD_NOP;
      break;
    }
  case CMD_ECHO: //команда "эхо"
    {
      //for(i = 0; i < Rx_Ma_Nbt && i < FRAME; i++)
      //  Tx_Ma_Dat[i] = Rx_Ma_Dat[i];
      //TxMaster_Replay(Rx_Ma_Nbt, Tx_Ma_Dat[0]);
      break;
    }
  case CMD_INFO: //команда "инфо"
    {
      if (Exchange_Sta == EXCH_REQUEST){
				TxMaster_Replay(1, ERR_NO);
				lastErrCode = 0;
				timer = 0;
				Exchange_Sta = EXCH_ANSWER;
			} else if (Exchange_Sta == EXCH_ANSWER) {
				
				lastErrCode = Rx_Ma_Dat[0];
				Exchange_Sta = EXCH_IDLE;
			}
			//char ch = 1;
      //for(i = 0; i < FRAME && ch; i++)
      //  ch = Tx_Ma_Dat[i] = Info[i];
      //TxMaster_Replay(i, Tx_Ma_Dat[0]);
      break;
    }
		
  case CMD_SETPAR: //установка параметра на slave
    {
			if (Exchange_Sta == EXCH_REQUEST){
				read_reg(Tx_Ma_Dat[0], &Tx_Ma_Dat[2], &Tx_Ma_Dat[1] ); // чтение параметра
				TxMaster_Replay(3, Tx_Ma_Dat[0]);
				lastErrCode = 0;
				timer = 0;
				Exchange_Sta = EXCH_ANSWER;
			} else if (Exchange_Sta == EXCH_ANSWER) {
				lastErrCode = Rx_Ma_Dat[0];
				Exchange_Sta = EXCH_IDLE;
			}
      break;
    }
  case CMD_GETPAR: //чтение параметра из slave
    {
			if (Exchange_Sta == EXCH_REQUEST){
				TxMaster_Replay(1, Tx_Ma_Dat[0]);
				lastErrCode = 0;
				timer = 0;
				Exchange_Sta = EXCH_ANSWER;
			} else if (Exchange_Sta == EXCH_ANSWER) {
				write_reg_ma(Rx_Ma_Dat[0], Rx_Ma_Dat[2], Rx_Ma_Dat[1]); //установка параметра 
				Exchange_Sta = EXCH_IDLE;
			}
      break;
    }
		
		case CMD_SETPARS: //установка параметров на slave
    {
			if (Exchange_Sta == EXCH_REQUEST){
				read_n_reg(Tx_Ma_Dat[0], &Tx_Ma_Dat[1], &Tx_Ma_Dat[2] ); //чтение N параметров
        TxMaster_Replay(Tx_Ma_Dat[1] * 2 + 2, Tx_Ma_Dat[0]);
				lastErrCode = 0;
				timer = 0;
				Exchange_Sta = EXCH_ANSWER;
			} else if (Exchange_Sta == EXCH_ANSWER) {
				lastErrCode = Rx_Ma_Dat[0];
				Exchange_Sta = EXCH_IDLE;
			}
			
      break;
    }
		case CMD_GETPARS: //чтение параметров из slave
    {
			if (Exchange_Sta == EXCH_REQUEST){
				TxMaster_Replay(2, Tx_Ma_Dat[0]);
				lastErrCode = 0;
				timer = 0;
				Exchange_Sta = EXCH_ANSWER;
			} else if (Exchange_Sta == EXCH_ANSWER) {
				write_n_reg_ma(Rx_Ma_Dat[0], Rx_Ma_Dat[1], &Rx_Ma_Dat[2]); //запись N параметров 
				Exchange_Sta = EXCH_IDLE;
			}
			
      break;
    }
  }
}

//----------------------------------------------------------------------------
