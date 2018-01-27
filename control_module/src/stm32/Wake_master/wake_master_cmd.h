//----------------------------------------------------------------------------

//������ ���������� ���������� ������: header file

//----------------------------------------------------------------------------

#ifndef _WAKE_CMD_H_
#define _WAKE_CMD_H_

#include <wake.h>

//-------------------------------- ���������: --------------------------------

//���� ����������� ������:

#define CMD_SETPAR     6 //��������� 1-�� ���������
#define CMD_GETPAR     7 //������ 1-�� ���������
#define CMD_SETPARS    8 //������ N ����������
#define CMD_GETPARS    9 //������ N ����������

//-------------------------------- ����������: -------------------------------

//extern uint8_t Exchange_Sta; //������ ������ � ����������� �����������

//---------------------------- ��������� �������: ----------------------------

void WakeMaster_Commands_Exe(void); //���������� ������
uint8_t WakeMaster_Busy(void);
uint8_t WakeMaster_Timeout(void);
void WakeMaster_Tick(void);
void WakeMaster_GetParam(uint8_t index);
void WakeMaster_GetParams(uint8_t index, uint8_t count);
void WakeMaster_SetParam(uint8_t index);
void WakeMaster_SetParams(uint8_t index, uint8_t count);

//----------------------------------------------------------------------------

#endif
