//----------------------------------------------------------------------------

//Модуль реализации протокола Wake

//----------------------------------------------------------------------------

#include <wake_exchange.h>
#include <wake.h>

//------------------------------- Константы: ---------------------------------


//------------------------------- Переменные: --------------------------------

uint8_t Rx_Sta,        //состояние процесса приема пакета
     Rx_Pre,        //предыдущий принятый байт
     Rx_Add,        //адрес, с которым сравнивается принятый
     Rx_Cmd,        //принятая команда
     Rx_Nbt,        //принятое количество байт в пакете
     Rx_Dat[FRAME], //массив принятых данных
     Rx_Crc,        //контрольная сумма принимаемого пакета
     Rx_Ptr;        //указатель на массив принимаемых данных

uint8_t Command;       //код команды на выполнение

uint8_t Tx_Sta,        //состояние процесса передачи пакета
     Tx_Pre,        //предыдущий переданный байт
     Tx_Add,        //адрес, передававемый в пакете
     Tx_Cmd,        //команда, передаваемая в пакете
     Tx_Nbt,        //количество байт данных в пакете
     Tx_Dat[FRAME], //массив данных для передачи
     Tx_Crc,        //контрольная сумма передаваемого пакета
     Tx_Ptr;        //указатель на массив передаваемых данных
		 
uint8_t wake_rx_buf;
uint8_t wake_tx_buf;

UART_HandleTypeDef *wake_huart;

//-------------------------- Прототипы функций: ------------------------------

void Do_Crc8(uint8_t b, uint8_t *crc);      //вычисление контрольной суммы

//------------------------- Инициализация UART: ------------------------------

void Wake_Init(UART_HandleTypeDef *huart)
{
  wake_huart = huart;
	HAL_UART_Receive_IT(huart, &wake_rx_buf, sizeof(wake_rx_buf));
  Rx_Add = 0;                         //адрес на прием
  Tx_Add = 0;                         //адрес на передачу
  Rx_Sta = WAIT_FEND;                 //ожидание пакета
  Tx_Sta = SEND_IDLE;                 //ничего пока не передаем
  Command = CMD_NOP;                  //нет команды на выполнение
}

//------------------- Прерывание UART после приема байта: --------------------

void Wake_UART_Rx(void)
{
	uint8_t error_flags = wake_huart->ErrorCode; 				//чтение флагов ошибок
  uint8_t data_byte = wake_rx_buf;               			//чтение данных
	HAL_UART_Receive_IT(wake_huart, &wake_rx_buf, sizeof(wake_rx_buf));
	
  if(error_flags)                     //если обнаружены ошибки при приеме байта
  {
    Rx_Sta = WAIT_FEND;               //ожидание нового пакета
    Command = CMD_ERR;                //сообщаем об ошибке
    return;
  }

  if(data_byte == (uint8_t)FEND)      //если обнаружено начало фрейма,
  {
    Rx_Pre = data_byte;               //то сохранение пре-байта,
    Rx_Crc = CRC_INIT;                //инициализация CRC,
    Rx_Sta = WAIT_ADDR;               //сброс указателя данных,
    Do_Crc8(data_byte, &Rx_Crc);      //обновление CRC,
    return;                           //выход
  }

  if(Rx_Sta == WAIT_FEND)             //-----> если ожидание FEND,
    return;                           //то выход

  uint8_t Pre = Rx_Pre;               //сохранение старого пре-байта
  Rx_Pre = data_byte;                 //обновление пре-байта
  if(Pre == FESC)                     //если пре-байт равен FESC,
  {
    if(data_byte == TFESC)            //а байт данных равен TFESC,
      data_byte = FESC;               //то заменить его на FESC
    else if(data_byte == TFEND)       //если байт данных равен TFEND,
           data_byte = FEND;          //то заменить его на FEND
         else
         {
           Rx_Sta = WAIT_FEND;        //для всех других значений байта данных,
           Command = CMD_ERR;         //следующего за FESC, ошибка
           return;
         }
  }
  else
  {
    if(data_byte == FESC)             //если байт данных равен FESC, он просто
      return;                         //запоминается в пре-байте
  }

  switch(Rx_Sta)
  {
  case WAIT_ADDR:                     //-----> ожидание приема адреса
    {
      if(data_byte & 0x80)            //если data_byte.7 = 1, то это адрес
      {
        data_byte = data_byte & 0x7F; //обнуляем бит 7, получаем истинный адрес
        if(!data_byte || data_byte == Rx_Add) //если нулевой или верный адрес,
        {
          Do_Crc8(data_byte, &Rx_Crc);//то обновление CRC и
          Rx_Sta = WAIT_CMD;          //переходим к приему команды
          break;
        }
        Rx_Sta = WAIT_FEND;           //адрес не совпал, ожидание нового пакета
        break;
      }                               //если data_byte.7 = 0, то
      Rx_Sta = WAIT_CMD;              //сразу переходим к приему команды
    }
  case WAIT_CMD:                      //-----> ожидание приема команды
    {
      if(data_byte & 0x80)            //проверка бита 7 данных
      {
        Rx_Sta = WAIT_FEND;           //если бит 7 не равен нулю,
        Command = CMD_ERR;            //то ошибка
        break;
      }
      Rx_Cmd = data_byte;             //сохранение команды
      Do_Crc8(data_byte, &Rx_Crc);    //обновление CRC
      Rx_Sta = WAIT_NBT;              //переходим к приему количества байт
      break;
    }
  case WAIT_NBT:                      //-----> ожидание приема количества байт
    {
      if(data_byte > FRAME)           //если количество байт > FRAME,
      {
        Rx_Sta = WAIT_FEND;
        Command = CMD_ERR;            //то ошибка
        break;
      }
      Rx_Nbt = data_byte;
      Do_Crc8(data_byte, &Rx_Crc);    //обновление CRC
      Rx_Ptr = 0;                     //обнуляем указатель данных
      Rx_Sta = WAIT_DATA;             //переходим к приему данных
      break;
    }
  case WAIT_DATA:                     //-----> ожидание приема данных
    {
      if(Rx_Ptr < Rx_Nbt)             //если не все данные приняты,
      {
        Rx_Dat[Rx_Ptr++] = data_byte; //то сохранение байта данных,
        Do_Crc8(data_byte, &Rx_Crc);  //обновление CRC
        break;
      }
      if(data_byte != Rx_Crc)         //если приняты все данные, то проверка CRC
      {
        Rx_Sta = WAIT_FEND;           //если CRC не совпадает,
        Command = CMD_ERR;            //то ошибка
        break;
      }
      Rx_Sta = WAIT_FEND;             //прием пакета завершен,
      Command = Rx_Cmd;               //загрузка команды на выполнение
      break;
    }
  }
}

//------------------ Прерывание UART после передачи байта: -------------------

void Wake_UART_Tx(void)
{
  //uint8_t data_byte;

  if(Tx_Pre == FEND)                  //если производится стаффинг,
  {
    wake_tx_buf = TFEND;                //передача TFEND вместо FEND
    Tx_Pre = wake_tx_buf;
		HAL_UART_Transmit_IT(wake_huart,&wake_tx_buf, sizeof(wake_tx_buf));
    return;
  }
  if(Tx_Pre == FESC)                  //если производится стаффинг,
  {
    wake_tx_buf = TFESC;                //передача TFESC вместо FESC
    Tx_Pre = wake_tx_buf;
		HAL_UART_Transmit_IT(wake_huart,&wake_tx_buf, sizeof(wake_tx_buf));
    return;
  }

  switch(Tx_Sta)
  {
  case SEND_ADDR:                     //-----> передача адреса
    {
      if(Tx_Add)                      //если адрес не равен нулю, передаем его
      {
        wake_tx_buf = Tx_Add;
        Do_Crc8(wake_tx_buf, &Tx_Crc);  //вычисление CRC для истинного адреса
        wake_tx_buf |= 0x80;            //установка бита 7 для передачи адреса
        Tx_Sta = SEND_CMD;
        break;
      }                               //иначе сразу передаем команду
    }
  case SEND_CMD:                      //-----> передача команды
    {
      wake_tx_buf = Tx_Cmd & 0x7F;
      Tx_Sta = SEND_NBT;
      break;
    }
  case SEND_NBT:                      //-----> передача количества байт
    {
      wake_tx_buf = Tx_Nbt;
      Tx_Sta = SEND_DATA;
      Tx_Ptr = 0;                     //обнуление указателя данных для передачи
      break;
    }
  case SEND_DATA:                     //-----> передача данных
    {
      if(Tx_Ptr < Tx_Nbt)
        wake_tx_buf = Tx_Dat[Tx_Ptr++];
      else
      {
        wake_tx_buf = Tx_Crc;           //передача CRC
        Tx_Sta = SEND_CRC;
      }
      break;
    }
  default:
    {
      Tx_Sta = SEND_IDLE;             //передача пакета завершена
      return;
    }
  }

  if(Tx_Sta != SEND_CMD)              //если не передача адреса, то
    Do_Crc8(wake_tx_buf, &Tx_Crc);      //вычисление CRC
  Tx_Pre = wake_tx_buf;                 //сохранение пре-байта
  if(wake_tx_buf == FEND || wake_tx_buf == FESC)
    wake_tx_buf = FESC;                 //передача FESC, если нужен стаффинг
	HAL_UART_Transmit_IT(wake_huart,&wake_tx_buf, sizeof(wake_tx_buf));
}

//--------------------- Вычисление контрольной суммы: ------------------------

void Do_Crc8(uint8_t b, uint8_t *crc)
{
  for(char i = 0; i < 8; b = b >> 1, i++)
    if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
     else *crc = (*crc >> 1) & ~0x80;
}

//---------------------------- Передача пакета: ------------------------------

void Wake_Tx()
{
  //uint8_t data_byte = FEND;
	wake_tx_buf = FEND;
  Tx_Crc = CRC_INIT;                  //инициализация CRC,
  Do_Crc8(wake_tx_buf, &Tx_Crc);        //обновление CRC
  Tx_Sta = SEND_ADDR;
  Tx_Pre = TFEND;
	HAL_UART_Transmit_IT(wake_huart, &wake_tx_buf, sizeof(wake_tx_buf));
}

//---------------------- Проверка окончания передачи: ------------------------
uint8_t Wake_TxDone(void)
{
  return(Tx_Sta == SEND_IDLE);
}

//----------------------------------------------------------------------------
