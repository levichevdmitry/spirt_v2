//----------------------------------------------------------------------------

//Константы протокола WAKE:

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
//#define FRAME 	 16      //максимальная длина пакета see wake_master_exhange.h and wake_exhange.h

//RX process states:

enum { WAIT_FEND,     //ожидание приема FEND
       WAIT_ADDR,     //ожидание приема адреса
       WAIT_CMD,      //ожидание приема команды
       WAIT_NBT,      //ожидание приема количества байт в пакете
       WAIT_DATA,     //прием данных
       WAIT_CRC,      //ожидание окончания приема CRC
       WAIT_CARR };   //ожидание несущей

//TX process states:

enum { SEND_IDLE,     //состояние бездействия
       SEND_ADDR,     //передача адреса
       SEND_CMD,      //передача команды
       SEND_NBT,      //передача количества байт в пакете
       SEND_DATA,     //передача данных
       SEND_CRC,      //передача CRC
       SEND_END };    //окончание передачи пакета

// EXCHANGE process states:
enum {
			 EXCH_REQUEST,
			 EXCH_ANSWER,
			 EXCH_IDLE
};

//Коды универсальных команд:

#define CMD_NOP     0 //нет операции
#define CMD_ERR     1 //ошибка приема пакета
#define CMD_ECHO    2 //передать эхо
#define CMD_INFO    3 //передать информацию об устройстве
#define CMD_SETADDR 4 //установить адрес
#define CMD_GETADDR 5 //прочитать адрес

//Коды ошибок:

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
