//----------------------------------------------------------------------------

//ћодуль реализации выполнени€ команд

//----------------------------------------------------------------------------

#include <wake_cmd.h>
#include <wake_exchange.h>
#include <wake.h>
#include <registers.h>

//------------------------------  онстанты: ----------------------------------

const char Info[] = {"SPM v1\0"}; //им€ устройства

//----------------------------- ѕеременные: ----------------------------------


//--------------------------- ѕрототипы функций: -----------------------------

void Tx_Replay(char n, char err); //передача ответа на команду

//---------------------- ѕередача ответа на команду: -------------------------

void Tx_Replay(char n, char err)
{
  Tx_Nbt = n;                     //количество байт
  Tx_Dat[0] = err;                //код ошибка
  Tx_Cmd = Command;               //команда
  Wake_Tx();                      //инициализаци€ передачи
  Command = CMD_NOP;              //команда обработана
}

//---------------------------- ¬ыполнение команд: ----------------------------

void Wake_Commands_Exe(void)
{
  char i;
  switch(Command)
  {
  case CMD_ERR: //обработка ошибки
    {
      Tx_Replay(1, ERR_TX);
      break;
    }
  case CMD_ECHO: //команда "эхо"
    {
      for(i = 0; i < Rx_Nbt && i < FRAME; i++)
        Tx_Dat[i] = Rx_Dat[i];
      Tx_Replay(Rx_Nbt, Tx_Dat[0]);
      break;
    }
  case CMD_INFO: //команда "инфо"
    {
      char ch = 1;
      for(i = 0; i < FRAME && ch; i++)
        ch = Tx_Dat[i] = Info[i];
      Tx_Replay(i, Tx_Dat[0]);
      break;
    }
		
  case CMD_SETPAR: //установка параметра
    {
			write_reg(Rx_Dat[0], Rx_Dat[2], Rx_Dat[1]); //установка параметра 
      Tx_Replay(1, ERR_NO);
      break;
    }
  case CMD_GETPAR: //чтение параметра
    {
			//Rx_Dat[0] reg index
			read_reg(Rx_Dat[0], &Tx_Dat[2], &Tx_Dat[1] ); // чтение параметра
      Tx_Replay(3, Rx_Dat[0]);
      break;
    }
		
		case CMD_SETPARS: //установка параметра
    {
			write_n_reg(Rx_Dat[0], Rx_Dat[1], &Rx_Dat[2]); //запись N параметров 
      Tx_Replay(1, ERR_NO);
      break;
    }
  case CMD_GETPARS: //чтение параметра
    {
			//Rx_Dat[0] reg index
			Tx_Dat[1] = Rx_Dat[1]; //количество запрашиваемых регистров
			read_n_reg(Rx_Dat[0], &Tx_Dat[1], &Tx_Dat[2] ); //чтение N параметров
      Tx_Replay(Tx_Dat[1] * 2 + 2, Rx_Dat[0]);
      break;
    }
		
  }
}

//----------------------------------------------------------------------------
