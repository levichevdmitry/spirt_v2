/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.32                          *
*        Compiled Oct  8 2015, 11:59:02                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "windows.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0  (GUI_ID_USER + 0x00)
#define ID_BUTTON_0  (GUI_ID_USER + 0x01)
#define ID_TEXT_0  (GUI_ID_USER + 0x02)
#define ID_TEXT_11  (GUI_ID_USER + 0x03)
#define ID_TEXT_1  (GUI_ID_USER + 0x04)
#define ID_TEXT_2  (GUI_ID_USER + 0x05)
#define ID_TEXT_3  (GUI_ID_USER + 0x06)
#define ID_TEXT_4  (GUI_ID_USER + 0x07)
#define ID_TEXT_5  (GUI_ID_USER + 0x08)
#define ID_TEXT_6  (GUI_ID_USER + 0x09)
#define ID_TEXT_7  (GUI_ID_USER + 0x0A)
#define ID_TEXT_8  (GUI_ID_USER + 0x0B)
#define ID_TEXT_9  (GUI_ID_USER + 0x0C)
#define ID_TEXT_10  (GUI_ID_USER + 0x0D)
#define ID_EDIT_0  (GUI_ID_USER + 0x0E)
#define ID_EDIT_1  (GUI_ID_USER + 0x0F)
#define ID_EDIT_2  (GUI_ID_USER + 0x10)
#define ID_EDIT_3  (GUI_ID_USER + 0x11)
#define ID_EDIT_4  (GUI_ID_USER + 0x12)
#define ID_EDIT_5  (GUI_ID_USER + 0x13)
#define ID_EDIT_6  (GUI_ID_USER + 0x14)
#define ID_EDIT_7  (GUI_ID_USER + 0x15)
#define ID_EDIT_8  (GUI_ID_USER + 0x16)
#define ID_EDIT_9  (GUI_ID_USER + 0x17)
#define ID_TIMER_CONVERT_MONITOR (GUI_ID_USER + 0x18)




// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Tests", ID_WINDOW_0, 0, 1, 800, 480, 0, 0x0, 0 },
	{ BUTTON_CreateIndirect, "Button", ID_BUTTON_0, 600, 350, 150, 80, 0, 0x0, 0 }, // Exit button
  { TEXT_CreateIndirect, "Text", ID_TEXT_0, 5, 80, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_1, 5, 120, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_2, 5, 160, 360, 30, 0, 0x64, 0 },
 // { TEXT_CreateIndirect, "Text", ID_TEXT_3, 5, 200, 360, 30, 0, 0x64, 0 },
 // { TEXT_CreateIndirect, "Text", ID_TEXT_4, 5, 220, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_5, 5, 240, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_6, 5, 280, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_7, 5, 320, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_8, 5, 360, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_9, 5, 400, 360, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_10, 5, 40, 360, 30, 0, 0x64, 0 },
	{ TEXT_CreateIndirect, "Text", ID_TEXT_11, 250, 5, 300, 30, 0, 0x64, 0 }, // Header text
  { EDIT_CreateIndirect, "Edit", ID_EDIT_0, 400, 40, 80, 30, 0, 0x4, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_1, 400, 80, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_2, 400, 120, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_3, 400, 160, 80, 30, 0, 0x64, 0 },
 // { EDIT_CreateIndirect, "Edit", ID_EDIT_4, 400, 200, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_5, 400, 240, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_6, 400, 280, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_7, 400, 320, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_8, 400, 360, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_9, 400, 400, 80, 30, 0, 0x64, 0 },
	
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
WM_HTIMER hTimerSen;

// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Tests'
    //
    hItem = pMsg->hWin;
	
		hTimerSen = WM_CreateTimer(hItem, ID_TIMER_CONVERT_MONITOR, 100, 0);
	
    WINDOW_SetBkColor(hItem, 0x00C08000);
    //
    // Initialization of 'Header text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_11);
    TEXT_SetText(hItem, "������������ ��������");
    TEXT_SetFont(hItem, &GUI_FontArial32);
	
		//
		// Initialization of 'Exit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
		//BUTTON_SetBkColor(hItem, BUTTON_CI_UNPRESSED, 0x0068f7f9);
    BUTTON_SetText(hItem, "�����");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetText(hItem, "� ������ ���, C");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetText(hItem, "� ������ ����, C");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetText(hItem, "����������� ��, C");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
		/*
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetText(hItem, "� ����� ������������, C");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    */
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
    TEXT_SetText(hItem, "������ ������� ������ ��");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
    TEXT_SetText(hItem, "�������� �����");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_7);
    TEXT_SetText(hItem, "�������� � ����, �� ��.�");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_8);
    TEXT_SetText(hItem, "������ ��, �/���");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_9);
    TEXT_SetText(hItem, "��� ������");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_10);
    TEXT_SetText(hItem, "����������� � ����, �");
    TEXT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 't kub'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
		EDIT_SetFloatMode(hItem, Sensors.t_kub, 0.0, 125.0, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES); 
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 't kolona niz'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1); // 
    EDIT_SetFloatMode(hItem, Sensors.t_kolona_n, 0.0, 125.0, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 't kolona verh'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_2);
    EDIT_SetFloatMode(hItem, Sensors.t_kolona_v, 0.0, 125.0, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 't coolant'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_3);
		EDIT_SetDecMode(hItem, Sensors.t_coolant, 0, 125, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 't after def'
    //
		/*
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_4);
    EDIT_SetFloatMode(hItem, Sensors.t_after_def, 0.0, 125.0, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
		*/
    //
    // Initialization of 'Level sens'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_5);
    EDIT_SetText(hItem, "����.");
		EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_GREEN);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'Time outs'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_6); // time outs
		EDIT_SetDecMode(hItem, wakeMaster_timeout, 0, 65000, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'p kub'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_7);
    EDIT_SetFloatMode(hItem, Sensors.p_kub, Settings.p_min.value, Settings.p_max.value, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'water flow'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_8);
		EDIT_SetFloatMode(hItem, Sensors.f_coolant, 0.0, 30.0, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //EDIT_SetText(hItem, "����.");
		//EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_GREEN);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    //
    // Initialization of 'spirt concentration'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_9);
    EDIT_SetText(hItem, "����.");
		EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_GREEN);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
		
	case WM_TIMER:
		
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
			EDIT_SetFloatValue(hItem, Sensors.t_kub);					
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);  
			EDIT_SetFloatValue(hItem, Sensors.t_kolona_n);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_2);
			EDIT_SetFloatValue(hItem, Sensors.t_kolona_v);
	
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_3);
			EDIT_SetValue(hItem, Sensors.t_coolant);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_8);
			EDIT_SetValue(hItem, Sensors.f_coolant);
	
			//hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_4);
      //EDIT_SetFloatValue(hItem, Sensors.t_after_def);
		
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_5);
			if (Sensors.l_coolant_al) {
				EDIT_SetText(hItem, "���.");
				EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_RED);
			} else {
				EDIT_SetText(hItem, "����.");
				EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_GREEN);
			}
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_6); // time outs
			EDIT_SetValue(hItem, wakeMaster_timeout);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_7);
			EDIT_SetFloatValue(hItem, Sensors.p_kub);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_8);
			EDIT_SetFloatValue(hItem, Sensors.f_coolant);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_9);
			if (Sensors.spirt_al) {
				EDIT_SetText(hItem, "����.");
				EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_RED);
			} else {
				EDIT_SetText(hItem, "����.");
				EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_GREEN);
			}
			
	
			WM_RestartTimer(pMsg->Data.v, 250);
	
		break;
	
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    
     
		case ID_BUTTON_0: // Notifications sent by 'Exit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
					GUI_EndDialog ( pMsg->hWin, SCR_TEST_SEL);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_0: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_1: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_2: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_3: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_4: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_5: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_6: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_7: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_8: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_9: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateTests
*/
WM_HWIN CreateSenTests(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
