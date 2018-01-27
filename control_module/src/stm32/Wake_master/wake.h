//----------------------------------------------------------------------------

//��������� ��������� WAKE:

//----------------------------------------------------------------------------

#ifndef _WAKE_H_
#define _WAKE_H_

#include "stm32f4xx_hal.h"

//#include <wake_cmd.h>
//#include <wake_exchange.h>

#include <wake_master_cmd.h>
#include <wake_master_exchange.h>

//----------------------------------------------------------------------------

#define FEND  0xC0    //Frame END
#define FESC  0xDB    //Frame ESCape
#define TFEND 0xDC    //Transposed Frame END
#define TFESC 0xDD    //Transposed Frame ESCape

#define CRC_INIT 0xDE //Innitial CRC value
//#define FRAME 	 16      //������������ ����� ������ see wake_master_exhange.h and wake_exhange.h

//RX process states:

enum { WAIT_FEND,     //�������� ������ FEND
       WAIT_ADDR,     //�������� ������ ������
       WAIT_CMD,      //�������� ������ �������
       WAIT_NBT,      //�������� ������ ���������� ���� � ������
       WAIT_DATA,     //����� ������
       WAIT_CRC,      //�������� ��������� ������ CRC
       WAIT_CARR };   //�������� �������

//TX process states:

enum { SEND_IDLE,     //��������� �����������
       SEND_ADDR,     //�������� ������
       SEND_CMD,      //�������� �������
       SEND_NBT,      //�������� ���������� ���� � ������
       SEND_DATA,     //�������� ������
       SEND_CRC,      //�������� CRC
       SEND_END };    //��������� �������� ������

// EXCHANGE process states:
enum {
			 EXCH_REQUEST,
			 EXCH_ANSWER,
			 EXCH_IDLE
};

//���� ������������� ������:

#define CMD_NOP     0 //��� ��������
#define CMD_ERR     1 //������ ������ ������
#define CMD_ECHO    2 //�������� ���
#define CMD_INFO    3 //�������� ���������� �� ����������
#define CMD_SETADDR 4 //���������� �����
#define CMD_GETADDR 5 //��������� �����

//���� ������:

#define ERR_NO 0x00   //no error
#define ERR_TX 0x01   //Rx/Tx error
#define ERR_BU 0x02   //device busy error
#define ERR_RE 0x03   //device not ready error
#define ERR_PA 0x04   //parameters value error
#define ERR_NR 0x05   //no replay
#define ERR_NC 0x06   //no carrier

//----------------------------------------------------------------------------

#define WORD(byte2, byte1)  ( (((uint16_t)byte2) << 8) | ((uint16_t)byte1) )
#define WORD_BYTE1(dword) ((dword)& 0x00FF)
#define WORD_BYTE2(dword) (((dword) >> 8)& 0x00FF )

//----------------------------------------------------------------------------

//extern uint8_t wake_rx_buf;

#endif
