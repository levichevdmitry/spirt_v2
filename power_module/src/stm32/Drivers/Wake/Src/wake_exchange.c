//----------------------------------------------------------------------------

//������ ���������� ��������� Wake

//----------------------------------------------------------------------------

#include <wake_exchange.h>
#include <wake.h>

//------------------------------- ���������: ---------------------------------


//------------------------------- ����������: --------------------------------

uint8_t Rx_Sta,        //��������� �������� ������ ������
     Rx_Pre,        //���������� �������� ����
     Rx_Add,        //�����, � ������� ������������ ��������
     Rx_Cmd,        //�������� �������
     Rx_Nbt,        //�������� ���������� ���� � ������
     Rx_Dat[FRAME], //������ �������� ������
     Rx_Crc,        //����������� ����� ������������ ������
     Rx_Ptr;        //��������� �� ������ ����������� ������

uint8_t Command;       //��� ������� �� ����������

uint8_t Tx_Sta,        //��������� �������� �������� ������
     Tx_Pre,        //���������� ���������� ����
     Tx_Add,        //�����, ������������� � ������
     Tx_Cmd,        //�������, ������������ � ������
     Tx_Nbt,        //���������� ���� ������ � ������
     Tx_Dat[FRAME], //������ ������ ��� ��������
     Tx_Crc,        //����������� ����� ������������� ������
     Tx_Ptr;        //��������� �� ������ ������������ ������
		 
uint8_t wake_rx_buf;
uint8_t wake_tx_buf;

UART_HandleTypeDef *wake_huart;

//-------------------------- ��������� �������: ------------------------------

void Do_Crc8(uint8_t b, uint8_t *crc);      //���������� ����������� �����

//------------------------- ������������� UART: ------------------------------

void Wake_Init(UART_HandleTypeDef *huart)
{
  wake_huart = huart;
	HAL_UART_Receive_IT(huart, &wake_rx_buf, sizeof(wake_rx_buf));
  Rx_Add = 0;                         //����� �� �����
  Tx_Add = 0;                         //����� �� ��������
  Rx_Sta = WAIT_FEND;                 //�������� ������
  Tx_Sta = SEND_IDLE;                 //������ ���� �� ��������
  Command = CMD_NOP;                  //��� ������� �� ����������
}

//------------------- ���������� UART ����� ������ �����: --------------------

void Wake_UART_Rx(void)
{
	uint8_t error_flags = wake_huart->ErrorCode; 				//������ ������ ������
  uint8_t data_byte = wake_rx_buf;               			//������ ������
	HAL_UART_Receive_IT(wake_huart, &wake_rx_buf, sizeof(wake_rx_buf));
	
  if(error_flags)                     //���� ���������� ������ ��� ������ �����
  {
    Rx_Sta = WAIT_FEND;               //�������� ������ ������
    Command = CMD_ERR;                //�������� �� ������
    return;
  }

  if(data_byte == (uint8_t)FEND)      //���� ���������� ������ ������,
  {
    Rx_Pre = data_byte;               //�� ���������� ���-�����,
    Rx_Crc = CRC_INIT;                //������������� CRC,
    Rx_Sta = WAIT_ADDR;               //����� ��������� ������,
    Do_Crc8(data_byte, &Rx_Crc);      //���������� CRC,
    return;                           //�����
  }

  if(Rx_Sta == WAIT_FEND)             //-----> ���� �������� FEND,
    return;                           //�� �����

  uint8_t Pre = Rx_Pre;               //���������� ������� ���-�����
  Rx_Pre = data_byte;                 //���������� ���-�����
  if(Pre == FESC)                     //���� ���-���� ����� FESC,
  {
    if(data_byte == TFESC)            //� ���� ������ ����� TFESC,
      data_byte = FESC;               //�� �������� ��� �� FESC
    else if(data_byte == TFEND)       //���� ���� ������ ����� TFEND,
           data_byte = FEND;          //�� �������� ��� �� FEND
         else
         {
           Rx_Sta = WAIT_FEND;        //��� ���� ������ �������� ����� ������,
           Command = CMD_ERR;         //���������� �� FESC, ������
           return;
         }
  }
  else
  {
    if(data_byte == FESC)             //���� ���� ������ ����� FESC, �� ������
      return;                         //������������ � ���-�����
  }

  switch(Rx_Sta)
  {
  case WAIT_ADDR:                     //-----> �������� ������ ������
    {
      if(data_byte & 0x80)            //���� data_byte.7 = 1, �� ��� �����
      {
        data_byte = data_byte & 0x7F; //�������� ��� 7, �������� �������� �����
        if(!data_byte || data_byte == Rx_Add) //���� ������� ��� ������ �����,
        {
          Do_Crc8(data_byte, &Rx_Crc);//�� ���������� CRC �
          Rx_Sta = WAIT_CMD;          //��������� � ������ �������
          break;
        }
        Rx_Sta = WAIT_FEND;           //����� �� ������, �������� ������ ������
        break;
      }                               //���� data_byte.7 = 0, ��
      Rx_Sta = WAIT_CMD;              //����� ��������� � ������ �������
    }
  case WAIT_CMD:                      //-----> �������� ������ �������
    {
      if(data_byte & 0x80)            //�������� ���� 7 ������
      {
        Rx_Sta = WAIT_FEND;           //���� ��� 7 �� ����� ����,
        Command = CMD_ERR;            //�� ������
        break;
      }
      Rx_Cmd = data_byte;             //���������� �������
      Do_Crc8(data_byte, &Rx_Crc);    //���������� CRC
      Rx_Sta = WAIT_NBT;              //��������� � ������ ���������� ����
      break;
    }
  case WAIT_NBT:                      //-----> �������� ������ ���������� ����
    {
      if(data_byte > FRAME)           //���� ���������� ���� > FRAME,
      {
        Rx_Sta = WAIT_FEND;
        Command = CMD_ERR;            //�� ������
        break;
      }
      Rx_Nbt = data_byte;
      Do_Crc8(data_byte, &Rx_Crc);    //���������� CRC
      Rx_Ptr = 0;                     //�������� ��������� ������
      Rx_Sta = WAIT_DATA;             //��������� � ������ ������
      break;
    }
  case WAIT_DATA:                     //-----> �������� ������ ������
    {
      if(Rx_Ptr < Rx_Nbt)             //���� �� ��� ������ �������,
      {
        Rx_Dat[Rx_Ptr++] = data_byte; //�� ���������� ����� ������,
        Do_Crc8(data_byte, &Rx_Crc);  //���������� CRC
        break;
      }
      if(data_byte != Rx_Crc)         //���� ������� ��� ������, �� �������� CRC
      {
        Rx_Sta = WAIT_FEND;           //���� CRC �� ���������,
        Command = CMD_ERR;            //�� ������
        break;
      }
      Rx_Sta = WAIT_FEND;             //����� ������ ��������,
      Command = Rx_Cmd;               //�������� ������� �� ����������
      break;
    }
  }
}

//------------------ ���������� UART ����� �������� �����: -------------------

void Wake_UART_Tx(void)
{
  //uint8_t data_byte;

  if(Tx_Pre == FEND)                  //���� ������������ ��������,
  {
    wake_tx_buf = TFEND;                //�������� TFEND ������ FEND
    Tx_Pre = wake_tx_buf;
		HAL_UART_Transmit_IT(wake_huart,&wake_tx_buf, sizeof(wake_tx_buf));
    return;
  }
  if(Tx_Pre == FESC)                  //���� ������������ ��������,
  {
    wake_tx_buf = TFESC;                //�������� TFESC ������ FESC
    Tx_Pre = wake_tx_buf;
		HAL_UART_Transmit_IT(wake_huart,&wake_tx_buf, sizeof(wake_tx_buf));
    return;
  }

  switch(Tx_Sta)
  {
  case SEND_ADDR:                     //-----> �������� ������
    {
      if(Tx_Add)                      //���� ����� �� ����� ����, �������� ���
      {
        wake_tx_buf = Tx_Add;
        Do_Crc8(wake_tx_buf, &Tx_Crc);  //���������� CRC ��� ��������� ������
        wake_tx_buf |= 0x80;            //��������� ���� 7 ��� �������� ������
        Tx_Sta = SEND_CMD;
        break;
      }                               //����� ����� �������� �������
    }
  case SEND_CMD:                      //-----> �������� �������
    {
      wake_tx_buf = Tx_Cmd & 0x7F;
      Tx_Sta = SEND_NBT;
      break;
    }
  case SEND_NBT:                      //-----> �������� ���������� ����
    {
      wake_tx_buf = Tx_Nbt;
      Tx_Sta = SEND_DATA;
      Tx_Ptr = 0;                     //��������� ��������� ������ ��� ��������
      break;
    }
  case SEND_DATA:                     //-----> �������� ������
    {
      if(Tx_Ptr < Tx_Nbt)
        wake_tx_buf = Tx_Dat[Tx_Ptr++];
      else
      {
        wake_tx_buf = Tx_Crc;           //�������� CRC
        Tx_Sta = SEND_CRC;
      }
      break;
    }
  default:
    {
      Tx_Sta = SEND_IDLE;             //�������� ������ ���������
      return;
    }
  }

  if(Tx_Sta != SEND_CMD)              //���� �� �������� ������, ��
    Do_Crc8(wake_tx_buf, &Tx_Crc);      //���������� CRC
  Tx_Pre = wake_tx_buf;                 //���������� ���-�����
  if(wake_tx_buf == FEND || wake_tx_buf == FESC)
    wake_tx_buf = FESC;                 //�������� FESC, ���� ����� ��������
	HAL_UART_Transmit_IT(wake_huart,&wake_tx_buf, sizeof(wake_tx_buf));
}

//--------------------- ���������� ����������� �����: ------------------------

void Do_Crc8(uint8_t b, uint8_t *crc)
{
  for(char i = 0; i < 8; b = b >> 1, i++)
    if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
     else *crc = (*crc >> 1) & ~0x80;
}

//---------------------------- �������� ������: ------------------------------

void Wake_Tx()
{
  //uint8_t data_byte = FEND;
	wake_tx_buf = FEND;
  Tx_Crc = CRC_INIT;                  //������������� CRC,
  Do_Crc8(wake_tx_buf, &Tx_Crc);        //���������� CRC
  Tx_Sta = SEND_ADDR;
  Tx_Pre = TFEND;
	HAL_UART_Transmit_IT(wake_huart, &wake_tx_buf, sizeof(wake_tx_buf));
}

//---------------------- �������� ��������� ��������: ------------------------
uint8_t Wake_TxDone(void)
{
  return(Tx_Sta == SEND_IDLE);
}

//----------------------------------------------------------------------------
