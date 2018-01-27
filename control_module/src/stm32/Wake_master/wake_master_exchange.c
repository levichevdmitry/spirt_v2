//----------------------------------------------------------------------------

//������ ���������� ��������� Wake

//----------------------------------------------------------------------------

#include <wake_master_exchange.h>
#include <wake.h>

//------------------------------- ���������: ---------------------------------


//------------------------------- ����������: --------------------------------

uint8_t Rx_Ma_Sta,        //��������� �������� ������ ������
     Rx_Ma_Pre,        //���������� �������� ����
     Rx_Ma_Add,        //�����, � ������� ������������ ��������
     Rx_Ma_Cmd,        //�������� �������
     Rx_Ma_Nbt,        //�������� ���������� ���� � ������
     Rx_Ma_Dat[FRAME], //������ �������� ������
     Rx_Ma_Crc,        //����������� ����� ������������ ������
     Rx_Ma_Ptr;        //��������� �� ������ ����������� ������

uint8_t Master_Command;       //��� ������� �� ����������

uint8_t Tx_Ma_Sta,        //��������� �������� �������� ������
     Tx_Ma_Pre,        //���������� ���������� ����
     Tx_Ma_Add,        //�����, ������������� � ������
     Tx_Ma_Cmd,        //�������, ������������ � ������
     Tx_Ma_Nbt,        //���������� ���� ������ � ������
     Tx_Ma_Dat[FRAME], //������ ������ ��� ��������
     Tx_Ma_Crc,        //����������� ����� ������������� ������
     Tx_Ma_Ptr;        //��������� �� ������ ������������ ������
		 
uint8_t wake_ma_rx_buf;

UART_HandleTypeDef *wake_master_huart;

//-------------------------- ��������� �������: ------------------------------

void Do_Crc8(uint8_t b, uint8_t *crc);      //���������� ����������� �����

//------------------------- ������������� UART: ------------------------------

void WakeMaster_Init(UART_HandleTypeDef *huart)
{
  wake_master_huart = huart;
	HAL_UART_Receive_IT(huart, &wake_ma_rx_buf, sizeof(wake_ma_rx_buf));
  Rx_Ma_Add = 0;                         //����� �� �����
  Tx_Ma_Add = 0;                         //����� �� ��������
  Rx_Ma_Sta = WAIT_FEND;                 //�������� ������
  Tx_Ma_Sta = SEND_IDLE;                 //������ ���� �� ��������
  Master_Command = CMD_NOP;                  //��� ������� �� ����������
}

//------------------- ���������� UART ����� ������ �����: --------------------

void WakeMaster_UART_Rx(void)
{
	volatile uint8_t error_flags = wake_master_huart->ErrorCode; 				//������ ������ ������
	uint8_t data_byte = wake_ma_rx_buf;               			//������ ������
	HAL_UART_Receive_IT(wake_master_huart, &wake_ma_rx_buf, sizeof(wake_ma_rx_buf));
	
  if(error_flags != 0)                     //���� ���������� ������ ��� ������ �����
  {
    Rx_Ma_Sta = WAIT_FEND;            //�������� ������ ������
    Master_Command = CMD_ERR;                //�������� �� ������
    return;
  }

  if(data_byte == (uint8_t)FEND)      //���� ���������� ������ ������,
  {
    Rx_Ma_Pre = data_byte;               //�� ���������� ���-�����,
    Rx_Ma_Crc = CRC_INIT;                //������������� CRC,
    Rx_Ma_Sta = WAIT_ADDR;               //����� ��������� ������,
    Do_Crc8(data_byte, &Rx_Ma_Crc);      //���������� CRC,
    return;                           //�����
  }

  if(Rx_Ma_Sta == WAIT_FEND)             //-----> ���� �������� FEND,
    return;                           //�� �����

  uint8_t Pre = Rx_Ma_Pre;               //���������� ������� ���-�����
  Rx_Ma_Pre = data_byte;                 //���������� ���-�����
  if(Pre == FESC)                     //���� ���-���� ����� FESC,
  {
    if(data_byte == TFESC)            //� ���� ������ ����� TFESC,
      data_byte = FESC;               //�� �������� ��� �� FESC
    else if(data_byte == TFEND)       //���� ���� ������ ����� TFEND,
           data_byte = FEND;          //�� �������� ��� �� FEND
         else
         {
           Rx_Ma_Sta = WAIT_FEND;        //��� ���� ������ �������� ����� ������,
           Master_Command = CMD_ERR;         //���������� �� FESC, ������
           return;
         }
  }
  else
  {
    if(data_byte == FESC)             //���� ���� ������ ����� FESC, �� ������
      return;                         //������������ � ���-�����
  }

  switch(Rx_Ma_Sta)
  {
  case WAIT_ADDR:                     //-----> �������� ������ ������
    {
      if(data_byte & 0x80)            //���� data_byte.7 = 1, �� ��� �����
      {
        data_byte = data_byte & 0x7F; //�������� ��� 7, �������� �������� �����
        if(!data_byte || data_byte == Rx_Ma_Add) //���� ������� ��� ������ �����,
        {
          Do_Crc8(data_byte, &Rx_Ma_Crc);//�� ���������� CRC �
          Rx_Ma_Sta = WAIT_CMD;          //��������� � ������ �������
          break;
        }
        Rx_Ma_Sta = WAIT_FEND;           //����� �� ������, �������� ������ ������
        break;
      }                               //���� data_byte.7 = 0, ��
      Rx_Ma_Sta = WAIT_CMD;              //����� ��������� � ������ �������
    }
  case WAIT_CMD:                      //-----> �������� ������ �������
    {
      if(data_byte & 0x80)            //�������� ���� 7 ������
      {
        Rx_Ma_Sta = WAIT_FEND;           //���� ��� 7 �� ����� ����,
        Master_Command = CMD_ERR;            //�� ������
        break;
      }
      Rx_Ma_Cmd = data_byte;             //���������� �������
      Do_Crc8(data_byte, &Rx_Ma_Crc);    //���������� CRC
      Rx_Ma_Sta = WAIT_NBT;              //��������� � ������ ���������� ����
      break;
    }
  case WAIT_NBT:                      //-----> �������� ������ ���������� ����
    {
      if(data_byte > FRAME)           //���� ���������� ���� > FRAME,
      {
        Rx_Ma_Sta = WAIT_FEND;
        Master_Command = CMD_ERR;            //�� ������
        break;
      }
      Rx_Ma_Nbt = data_byte;
      Do_Crc8(data_byte, &Rx_Ma_Crc);    //���������� CRC
      Rx_Ma_Ptr = 0;                     //�������� ��������� ������
      Rx_Ma_Sta = WAIT_DATA;             //��������� � ������ ������
      break;
    }
  case WAIT_DATA:                     //-----> �������� ������ ������
    {
      if(Rx_Ma_Ptr < Rx_Ma_Nbt)             //���� �� ��� ������ �������,
      {
        Rx_Ma_Dat[Rx_Ma_Ptr++] = data_byte; //�� ���������� ����� ������,
        Do_Crc8(data_byte, &Rx_Ma_Crc);  //���������� CRC
        break;
      }
      if(data_byte != Rx_Ma_Crc)         //���� ������� ��� ������, �� �������� CRC
      {
        Rx_Ma_Sta = WAIT_FEND;           //���� CRC �� ���������,
        Master_Command = CMD_ERR;            //�� ������
        break;
      }
      Rx_Ma_Sta = WAIT_FEND;             //����� ������ ��������,
      Master_Command = Rx_Ma_Cmd;               //�������� ������� �� ����������
      break;
    }
  }
}

//------------------ ���������� UART ����� �������� �����: -------------------

void WakeMaster_UART_Tx(void)
{
  uint8_t data_byte;

  if(Tx_Ma_Pre == FEND)                  //���� ������������ ��������,
  {
    data_byte = TFEND;                //�������� TFEND ������ FEND
    Tx_Ma_Pre = data_byte;
		HAL_UART_Transmit_IT(wake_master_huart,&data_byte, sizeof(data_byte));
    return;
  }
  if(Tx_Ma_Pre == FESC)                  //���� ������������ ��������,
  {
    data_byte = TFESC;                //�������� TFESC ������ FESC
    Tx_Ma_Pre = data_byte;
		HAL_UART_Transmit_IT(wake_master_huart,&data_byte, sizeof(data_byte));
    return;
  }

  switch(Tx_Ma_Sta)
  {
  case SEND_ADDR:                     //-----> �������� ������
    {
      if(Tx_Ma_Add)                      //���� ����� �� ����� ����, �������� ���
      {
        data_byte = Tx_Ma_Add;
        Do_Crc8(data_byte, &Tx_Ma_Crc);  //���������� CRC ��� ��������� ������
        data_byte |= 0x80;            //��������� ���� 7 ��� �������� ������
        Tx_Ma_Sta = SEND_CMD;
        break;
      }                               //����� ����� �������� �������
    }
  case SEND_CMD:                      //-----> �������� �������
    {
      data_byte = Tx_Ma_Cmd & 0x7F;
      Tx_Ma_Sta = SEND_NBT;
      break;
    }
  case SEND_NBT:                      //-----> �������� ���������� ����
    {
      data_byte = Tx_Ma_Nbt;
      Tx_Ma_Sta = SEND_DATA;
      Tx_Ma_Ptr = 0;                     //��������� ��������� ������ ��� ��������
      break;
    }
  case SEND_DATA:                     //-----> �������� ������
    {
      if(Tx_Ma_Ptr < Tx_Ma_Nbt)
        data_byte = Tx_Ma_Dat[Tx_Ma_Ptr++];
      else
      {
        data_byte = Tx_Ma_Crc;           //�������� CRC
        Tx_Ma_Sta = SEND_CRC;
      }
      break;
    }
  default:
    {
      Tx_Ma_Sta = SEND_IDLE;             //�������� ������ ���������
      return;
    }
  }

  if(Tx_Ma_Sta != SEND_CMD)              //���� �� �������� ������, ��
    Do_Crc8(data_byte, &Tx_Ma_Crc);      //���������� CRC
  Tx_Ma_Pre = data_byte;                 //���������� ���-�����
  if(data_byte == FEND || data_byte == FESC)
    data_byte = FESC;                 //�������� FESC, ���� ����� ��������
	HAL_UART_Transmit_IT(wake_master_huart,&data_byte, sizeof(data_byte));
}

//--------------------- ���������� ����������� �����: ------------------------

void Do_Crc8(uint8_t b, uint8_t *crc)
{
  for(char i = 0; i < 8; b = b >> 1, i++)
    if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
     else *crc = (*crc >> 1) & ~0x80;
}

//---------------------------- �������� ������: ------------------------------

void WakeMaster_Tx()
{
  uint8_t data_byte = FEND;
  Tx_Ma_Crc = CRC_INIT;                  //������������� CRC,
  Do_Crc8(data_byte, &Tx_Ma_Crc);        //���������� CRC
  Tx_Ma_Sta = SEND_ADDR;
  Tx_Ma_Pre = TFEND;
	HAL_UART_Transmit_IT(wake_master_huart, &data_byte, sizeof(data_byte));
}

//---------------------- �������� ��������� ��������: ------------------------
uint8_t WakeMaster_TxDone(void)
{
  return(Tx_Ma_Sta == SEND_IDLE);
}

//----------------------------------------------------------------------------
