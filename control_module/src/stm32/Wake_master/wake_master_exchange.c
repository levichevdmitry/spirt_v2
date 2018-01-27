//----------------------------------------------------------------------------

//Модуль реализации протокола Wake

//----------------------------------------------------------------------------

#include <wake_master_exchange.h>
#include <wake.h>

//------------------------------- Константы: ---------------------------------


//------------------------------- Переменные: --------------------------------

uint8_t Rx_Ma_Sta,        //состояние процесса приема пакета
     Rx_Ma_Pre,        //предыдущий принятый байт
     Rx_Ma_Add,        //адрес, с которым сравнивается принятый
     Rx_Ma_Cmd,        //принятая команда
     Rx_Ma_Nbt,        //принятое количество байт в пакете
     Rx_Ma_Dat[FRAME], //массив принятых данных
     Rx_Ma_Crc,        //контрольная сумма принимаемого пакета
     Rx_Ma_Ptr;        //указатель на массив принимаемых данных

uint8_t Master_Command;       //код команды на выполнение

uint8_t Tx_Ma_Sta,        //состояние процесса передачи пакета
     Tx_Ma_Pre,        //предыдущий переданный байт
     Tx_Ma_Add,        //адрес, передававемый в пакете
     Tx_Ma_Cmd,        //команда, передаваемая в пакете
     Tx_Ma_Nbt,        //количество байт данных в пакете
     Tx_Ma_Dat[FRAME], //массив данных для передачи
     Tx_Ma_Crc,        //контрольная сумма передаваемого пакета
     Tx_Ma_Ptr;        //указатель на массив передаваемых данных
		 
uint8_t wake_ma_rx_buf;

UART_HandleTypeDef *wake_master_huart;

//-------------------------- Прототипы функций: ------------------------------

void Do_Crc8(uint8_t b, uint8_t *crc);      //вычисление контрольной суммы

//------------------------- Инициализация UART: ------------------------------

void WakeMaster_Init(UART_HandleTypeDef *huart)
{
  wake_master_huart = huart;
	HAL_UART_Receive_IT(huart, &wake_ma_rx_buf, sizeof(wake_ma_rx_buf));
  Rx_Ma_Add = 0;                         //адрес на прием
  Tx_Ma_Add = 0;                         //адрес на передачу
  Rx_Ma_Sta = WAIT_FEND;                 //ожидание пакета
  Tx_Ma_Sta = SEND_IDLE;                 //ничего пока не передаем
  Master_Command = CMD_NOP;                  //нет команды на выполнение
}

//------------------- Прерывание UART после приема байта: --------------------

void WakeMaster_UART_Rx(void)
{
	volatile uint8_t error_flags = wake_master_huart->ErrorCode; 				//чтение флагов ошибок
	uint8_t data_byte = wake_ma_rx_buf;               			//чтение данных
	HAL_UART_Receive_IT(wake_master_huart, &wake_ma_rx_buf, sizeof(wake_ma_rx_buf));
	
  if(error_flags != 0)                     //если обнаружены ошибки при приеме байта
  {
    Rx_Ma_Sta = WAIT_FEND;            //ожидание нового пакета
    Master_Command = CMD_ERR;                //сообщаем об ошибке
    return;
  }

  if(data_byte == (uint8_t)FEND)      //если обнаружено начало фрейма,
  {
    Rx_Ma_Pre = data_byte;               //то сохранение пре-байта,
    Rx_Ma_Crc = CRC_INIT;                //инициализация CRC,
    Rx_Ma_Sta = WAIT_ADDR;               //сброс указателя данных,
    Do_Crc8(data_byte, &Rx_Ma_Crc);      //обновление CRC,
    return;                           //выход
  }

  if(Rx_Ma_Sta == WAIT_FEND)             //-----> если ожидание FEND,
    return;                           //то выход

  uint8_t Pre = Rx_Ma_Pre;               //сохранение старого пре-байта
  Rx_Ma_Pre = data_byte;                 //обновление пре-байта
  if(Pre == FESC)                     //если пре-байт равен FESC,
  {
    if(data_byte == TFESC)            //а байт данных равен TFESC,
      data_byte = FESC;               //то заменить его на FESC
    else if(data_byte == TFEND)       //если байт данных равен TFEND,
           data_byte = FEND;          //то заменить его на FEND
         else
         {
           Rx_Ma_Sta = WAIT_FEND;        //для всех других значений байта данных,
           Master_Command = CMD_ERR;         //следующего за FESC, ошибка
           return;
         }
  }
  else
  {
    if(data_byte == FESC)             //если байт данных равен FESC, он просто
      return;                         //запоминается в пре-байте
  }

  switch(Rx_Ma_Sta)
  {
  case WAIT_ADDR:                     //-----> ожидание приема адреса
    {
      if(data_byte & 0x80)            //если data_byte.7 = 1, то это адрес
      {
        data_byte = data_byte & 0x7F; //обнуляем бит 7, получаем истинный адрес
        if(!data_byte || data_byte == Rx_Ma_Add) //если нулевой или верный адрес,
        {
          Do_Crc8(data_byte, &Rx_Ma_Crc);//то обновление CRC и
          Rx_Ma_Sta = WAIT_CMD;          //переходим к приему команды
          break;
        }
        Rx_Ma_Sta = WAIT_FEND;           //адрес не совпал, ожидание нового пакета
        break;
      }                               //если data_byte.7 = 0, то
      Rx_Ma_Sta = WAIT_CMD;              //сразу переходим к приему команды
    }
  case WAIT_CMD:                      //-----> ожидание приема команды
    {
      if(data_byte & 0x80)            //проверка бита 7 данных
      {
        Rx_Ma_Sta = WAIT_FEND;           //если бит 7 не равен нулю,
        Master_Command = CMD_ERR;            //то ошибка
        break;
      }
      Rx_Ma_Cmd = data_byte;             //сохранение команды
      Do_Crc8(data_byte, &Rx_Ma_Crc);    //обновление CRC
      Rx_Ma_Sta = WAIT_NBT;              //переходим к приему количества байт
      break;
    }
  case WAIT_NBT:                      //-----> ожидание приема количества байт
    {
      if(data_byte > FRAME)           //если количество байт > FRAME,
      {
        Rx_Ma_Sta = WAIT_FEND;
        Master_Command = CMD_ERR;            //то ошибка
        break;
      }
      Rx_Ma_Nbt = data_byte;
      Do_Crc8(data_byte, &Rx_Ma_Crc);    //обновление CRC
      Rx_Ma_Ptr = 0;                     //обнуляем указатель данных
      Rx_Ma_Sta = WAIT_DATA;             //переходим к приему данных
      break;
    }
  case WAIT_DATA:                     //-----> ожидание приема данных
    {
      if(Rx_Ma_Ptr < Rx_Ma_Nbt)             //если не все данные приняты,
      {
        Rx_Ma_Dat[Rx_Ma_Ptr++] = data_byte; //то сохранение байта данных,
        Do_Crc8(data_byte, &Rx_Ma_Crc);  //обновление CRC
        break;
      }
      if(data_byte != Rx_Ma_Crc)         //если приняты все данные, то проверка CRC
      {
        Rx_Ma_Sta = WAIT_FEND;           //если CRC не совпадает,
        Master_Command = CMD_ERR;            //то ошибка
        break;
      }
      Rx_Ma_Sta = WAIT_FEND;             //прием пакета завершен,
      Master_Command = Rx_Ma_Cmd;               //загрузка команды на выполнение
      break;
    }
  }
}

//------------------ Прерывание UART после передачи байта: -------------------

void WakeMaster_UART_Tx(void)
{
  uint8_t data_byte;

  if(Tx_Ma_Pre == FEND)                  //если производится стаффинг,
  {
    data_byte = TFEND;                //передача TFEND вместо FEND
    Tx_Ma_Pre = data_byte;
		HAL_UART_Transmit_IT(wake_master_huart,&data_byte, sizeof(data_byte));
    return;
  }
  if(Tx_Ma_Pre == FESC)                  //если производится стаффинг,
  {
    data_byte = TFESC;                //передача TFESC вместо FESC
    Tx_Ma_Pre = data_byte;
		HAL_UART_Transmit_IT(wake_master_huart,&data_byte, sizeof(data_byte));
    return;
  }

  switch(Tx_Ma_Sta)
  {
  case SEND_ADDR:                     //-----> передача адреса
    {
      if(Tx_Ma_Add)                      //если адрес не равен нулю, передаем его
      {
        data_byte = Tx_Ma_Add;
        Do_Crc8(data_byte, &Tx_Ma_Crc);  //вычисление CRC для истинного адреса
        data_byte |= 0x80;            //установка бита 7 для передачи адреса
        Tx_Ma_Sta = SEND_CMD;
        break;
      }                               //иначе сразу передаем команду
    }
  case SEND_CMD:                      //-----> передача команды
    {
      data_byte = Tx_Ma_Cmd & 0x7F;
      Tx_Ma_Sta = SEND_NBT;
      break;
    }
  case SEND_NBT:                      //-----> передача количества байт
    {
      data_byte = Tx_Ma_Nbt;
      Tx_Ma_Sta = SEND_DATA;
      Tx_Ma_Ptr = 0;                     //обнуление указателя данных для передачи
      break;
    }
  case SEND_DATA:                     //-----> передача данных
    {
      if(Tx_Ma_Ptr < Tx_Ma_Nbt)
        data_byte = Tx_Ma_Dat[Tx_Ma_Ptr++];
      else
      {
        data_byte = Tx_Ma_Crc;           //передача CRC
        Tx_Ma_Sta = SEND_CRC;
      }
      break;
    }
  default:
    {
      Tx_Ma_Sta = SEND_IDLE;             //передача пакета завершена
      return;
    }
  }

  if(Tx_Ma_Sta != SEND_CMD)              //если не передача адреса, то
    Do_Crc8(data_byte, &Tx_Ma_Crc);      //вычисление CRC
  Tx_Ma_Pre = data_byte;                 //сохранение пре-байта
  if(data_byte == FEND || data_byte == FESC)
    data_byte = FESC;                 //передача FESC, если нужен стаффинг
	HAL_UART_Transmit_IT(wake_master_huart,&data_byte, sizeof(data_byte));
}

//--------------------- Вычисление контрольной суммы: ------------------------

void Do_Crc8(uint8_t b, uint8_t *crc)
{
  for(char i = 0; i < 8; b = b >> 1, i++)
    if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
     else *crc = (*crc >> 1) & ~0x80;
}

//---------------------------- Передача пакета: ------------------------------

void WakeMaster_Tx()
{
  uint8_t data_byte = FEND;
  Tx_Ma_Crc = CRC_INIT;                  //инициализация CRC,
  Do_Crc8(data_byte, &Tx_Ma_Crc);        //обновление CRC
  Tx_Ma_Sta = SEND_ADDR;
  Tx_Ma_Pre = TFEND;
	HAL_UART_Transmit_IT(wake_master_huart, &data_byte, sizeof(data_byte));
}

//---------------------- Проверка окончания передачи: ------------------------
uint8_t WakeMaster_TxDone(void)
{
  return(Tx_Ma_Sta == SEND_IDLE);
}

//----------------------------------------------------------------------------
