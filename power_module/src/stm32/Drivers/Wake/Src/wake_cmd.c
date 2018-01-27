//----------------------------------------------------------------------------

//������ ���������� ���������� ������

//----------------------------------------------------------------------------

#include <wake_cmd.h>
#include <wake_exchange.h>
#include <wake.h>
#include <registers.h>

//------------------------------ ���������: ----------------------------------

const char Info[] = {"SPM v1\0"}; //��� ����������

//----------------------------- ����������: ----------------------------------


//--------------------------- ��������� �������: -----------------------------

void Tx_Replay(char n, char err); //�������� ������ �� �������

//---------------------- �������� ������ �� �������: -------------------------

void Tx_Replay(char n, char err)
{
  Tx_Nbt = n;                     //���������� ����
  Tx_Dat[0] = err;                //��� ������
  Tx_Cmd = Command;               //�������
  Wake_Tx();                      //������������� ��������
  Command = CMD_NOP;              //������� ����������
}

//---------------------------- ���������� ������: ----------------------------

void Wake_Commands_Exe(void)
{
  char i;
  switch(Command)
  {
  case CMD_ERR: //��������� ������
    {
      Tx_Replay(1, ERR_TX);
      break;
    }
  case CMD_ECHO: //������� "���"
    {
      for(i = 0; i < Rx_Nbt && i < FRAME; i++)
        Tx_Dat[i] = Rx_Dat[i];
      Tx_Replay(Rx_Nbt, Tx_Dat[0]);
      break;
    }
  case CMD_INFO: //������� "����"
    {
      char ch = 1;
      for(i = 0; i < FRAME && ch; i++)
        ch = Tx_Dat[i] = Info[i];
      Tx_Replay(i, Tx_Dat[0]);
      break;
    }
		
  case CMD_SETPAR: //��������� ���������
    {
			write_reg(Rx_Dat[0], Rx_Dat[2], Rx_Dat[1]); //��������� ��������� 
      Tx_Replay(1, ERR_NO);
      break;
    }
  case CMD_GETPAR: //������ ���������
    {
			//Rx_Dat[0] reg index
			read_reg(Rx_Dat[0], &Tx_Dat[2], &Tx_Dat[1] ); // ������ ���������
      Tx_Replay(3, Rx_Dat[0]);
      break;
    }
		
		case CMD_SETPARS: //��������� ���������
    {
			write_n_reg(Rx_Dat[0], Rx_Dat[1], &Rx_Dat[2]); //������ N ���������� 
      Tx_Replay(1, ERR_NO);
      break;
    }
  case CMD_GETPARS: //������ ���������
    {
			//Rx_Dat[0] reg index
			Tx_Dat[1] = Rx_Dat[1]; //���������� ������������� ���������
			read_n_reg(Rx_Dat[0], &Tx_Dat[1], &Tx_Dat[2] ); //������ N ����������
      Tx_Replay(Tx_Dat[1] * 2 + 2, Rx_Dat[0]);
      break;
    }
		
  }
}

//----------------------------------------------------------------------------
