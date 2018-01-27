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
#include "string.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0   (GUI_ID_USER + 0x00)
#define ID_TEXT_0   (GUI_ID_USER + 0x01)
#define ID_TEXT_1   (GUI_ID_USER + 0x02)
#define ID_TEXT_2   (GUI_ID_USER + 0x03)
#define ID_TEXT_3   (GUI_ID_USER + 0x04)
#define ID_TEXT_4   (GUI_ID_USER + 0x05)
#define ID_TEXT_5   (GUI_ID_USER + 0x06)
#define ID_TEXT_6   (GUI_ID_USER + 0x07)
#define ID_EDIT_0   (GUI_ID_USER + 0x08)
#define ID_EDIT_1   (GUI_ID_USER + 0x09)
#define ID_EDIT_2   (GUI_ID_USER + 0x0A)
#define ID_SPINBOX_0   (GUI_ID_USER + 0x0B)
#define ID_SPINBOX_1   (GUI_ID_USER + 0x0C)
#define ID_SPINBOX_2   (GUI_ID_USER + 0x0D)
#define ID_BUTTON_0   (GUI_ID_USER + 0x0E)
#define ID_BUTTON_1   (GUI_ID_USER + 0x0F)
#define ID_BUTTON_2   (GUI_ID_USER + 0x10)
#define ID_BUTTON_3   (GUI_ID_USER + 0x11)
#define ID_EDIT_3   (GUI_ID_USER + 0x12)
#define ID_TIMER_CONVERT_MONITOR (GUI_ID_USER + 0x13)


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
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, 800, 480, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_0, 250, 5, 300, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_1, 5, 100, 300, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_2, 5, 160, 300, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_3, 5, 220, 300, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_4, 370, 100, 300, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_5, 370, 160, 300, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_6, 370, 220, 300, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_0, 260, 100, 100, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_1, 260, 160, 100, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_2, 260, 220, 100, 30, 0, 0x64, 0 },
  { SPINBOX_CreateIndirect, "Spinbox", ID_SPINBOX_0, 665, 90, 120, 50, 0, 0x0, 0 },
  { SPINBOX_CreateIndirect, "Spinbox", ID_SPINBOX_1, 665, 150, 120, 50, 0, 0x0, 0 },
  { SPINBOX_CreateIndirect, "Spinbox", ID_SPINBOX_2, 665, 210, 120, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "StartHeadBtn", ID_BUTTON_0, 40, 270, 200, 80, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "StartBoddyBtn", ID_BUTTON_1, 40, 370, 200, 80, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "StopBtn", ID_BUTTON_2, 270, 282, 160, 160, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "BackBtn", ID_BUTTON_3, 620, 380, 160, 80, 0, 0x0, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_3, 10, 50, 780, 30, 0, 0x64, 0 },
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
WM_HTIMER hTimerDistRun;
char buf[48];
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  EDIT_Handle hEdit;
	WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Window'
    //
    hItem = pMsg->hWin;
    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(0x00C08000));
		
		hTimerDistRun = WM_CreateTimer(hItem, ID_TIMER_CONVERT_MONITOR, 250, 0);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "�����������");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "� ����, �");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "� ������� ����, ���");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "������ ������");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "�������� ������, ��/�");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "����� �����, ��");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "� ��� 2, %");
    //
    // Initialization of 't kub'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    EDIT_SetFloatMode(hItem, Sensors.t_kub, 0.0, 125.0, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of 'P TEN sum'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);
		EDIT_SetFloatMode(hItem, Sensors.heater1_pow + Sensors.heater2_pow, 0.0, 10.0, 2, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of 'P TEN sum'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_2);
    EDIT_SetText(hItem, "����");
    EDIT_SetFont(hItem, &GUI_FontArial32);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of 'V get'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_0);
    SPINBOX_SetFont(hItem, &GUI_FontArial32);
		hEdit = SPINBOX_GetEditHandle(hItem);
		EDIT_SetDecMode(hEdit, Settings.dist_set.V_get.value, Settings.dist_set.V_get.min_val, Settings.dist_set.V_get.max_val, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'Q head'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_1);
    SPINBOX_SetFont(hItem, &GUI_FontArial32);
		hEdit = SPINBOX_GetEditHandle(hItem);
		EDIT_SetDecMode(hEdit, Settings.dist_set.Q_head.value, Settings.dist_set.Q_head.min_val, Settings.dist_set.Q_head.max_val, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'P ten 2'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_2);
    SPINBOX_SetFont(hItem, &GUI_FontArial32);
		SPINBOX_SetStep(hItem, 10);
    hEdit = SPINBOX_GetEditHandle(hItem);
		EDIT_SetDecMode(hEdit, Settings.dist_set.pow_heater.value, Settings.dist_set.pow_heater.min_val, Settings.dist_set.pow_heater.max_val, 1, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'StartHeadBtn'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "����� �����");
    //
    // Initialization of 'StartBoddyBtn'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "����� ����");
    //
    // Initialization of 'StopBtn'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "����������\n�����");
    //
    // Initialization of 'BackBtn'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_3);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "�����");
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_3);
    EDIT_SetFont(hItem, &GUI_FontArial32);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    EDIT_SetText(hItem, "��������");
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
	case WM_TIMER:
			hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0); // header
			sprintf(buf, "����������� %02i:%02i:%02i", globalState.dist_state.time.hour, globalState.dist_state.time.min, globalState.dist_state.time.sec);
			TEXT_SetText(hItem, buf);
	
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0); // t kub
			EDIT_SetFloatValue(hItem, Sensors.t_kub);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1); // power of ten
			EDIT_SetFloatValue(hItem, Sensors.heater1_pow + Sensors.heater2_pow);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_2); // pump state
			if (GetCoolingPumpState()) {
				if ( Sensors.f_coolant < 0.5F) {
					EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_RED);
				} 
				if ( Sensors.f_coolant >= 1.5F) {
					EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, GUI_GREEN);
				}
				EDIT_SetText(hItem, "���");
			} else {
				EDIT_SetBkColor(hItem, EDIT_CI_ENABLED, EDIT_GetDefaultBkColor(EDIT_CI_ENABLED));
				EDIT_SetText(hItem, "����");
			}
			
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_3); // status
			
			switch(GetSubMode()){
					case SMODE_DIST_PREPARE:					// ����������
						EDIT_SetText(hItem, "����������");
						break;
					case SMODE_DIST_PREHEAT:					// ����������
						EDIT_SetText(hItem, "����������");
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_BLUE);
						break;
					case SMODE_DIST_HEATING:					// ��������	
						EDIT_SetText(hItem, "��������");
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_MAGENTA);
						break;
					case SMODE_DIST_GET_HEAD:					// ����� �����
						sprintf(buf, "����� \"�����\" %4.1f ��", globalState.dist_state.Q_fact);
						EDIT_SetText(hItem, buf);
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_ORANGE);
						break;
					case SMODE_DIST_WORK_YOUSELF:			// ������ �� ����
						sprintf(buf, "������ �� ���� %i ���", globalState.dist_state.T_elapsed);
						EDIT_SetText(hItem, buf);
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_ORANGE);
						break;
					case SMODE_DIST_GET_BODY:					// ����� ����
						sprintf(buf, "����� \"����\" �������� %4.0f ��/�", globalState.dist_state.V_get_now);
						EDIT_SetText(hItem, buf);
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_ORANGE);
						break;
					case SMODE_DIST_GET_TAIL:					// ����� �������
						sprintf(buf, "����� \"�������\" �������� %4.0f ��/�", globalState.dist_state.V_get_now);
						EDIT_SetText(hItem, buf);
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_ORANGE);
						break;
					case SMODE_DIST_END:							// ��������� ��������
						sprintf(buf, "������� ������� %i ���", globalState.dist_state.T_elapsed);
						EDIT_SetText(hItem, buf);
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_GREEN);
						break;
					case SMODE_DIST_GET_PAUSE:				// ����� ������
						EDIT_SetText(hItem, "����� ������");
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_YELLOW);
						break;
					case SMODE_EMSTOP:	
						EDIT_SetText(hItem, "��������� ����!!!");
						EDIT_SetBkColor(hItem, EDIT_CI_ENABLED,  GUI_RED);
						break;
					}			
			
			WM_RestartTimer(pMsg->Data.v, 250);
		break;
		
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
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
    case ID_SPINBOX_0: // Notifications sent by 'V get'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_MOVED_OUT:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
					hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_0);
					Settings.dist_set.V_get.value = SPINBOX_GetValue(hItem);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SPINBOX_1: // Notifications sent by 'Q head'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_MOVED_OUT:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
					hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_1);
					Settings.dist_set.Q_head.value = SPINBOX_GetValue(hItem);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SPINBOX_2: // Notifications sent by 'power ten 2'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_MOVED_OUT:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
					hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_2);
					Settings.dist_set.pow_heater.value = SPINBOX_GetValue(hItem);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_0: // Notifications sent by 'StartHeadBtn'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				if (GetSubMode() == SMODE_DIST_HEATING) { // !w ����� ���� ����������������
					SetSubMode(SMODE_DIST_GET_HEAD);
				}
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
    case ID_BUTTON_1: // Notifications sent by 'StartBoddyBtn'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
					if (GetSubMode() == SMODE_DIST_WORK_YOUSELF) { // !w ����� ���� ����������������
					SetSubMode(SMODE_DIST_GET_BODY);
					}
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
    case ID_BUTTON_2: // Notifications sent by 'StopBtn'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
				if (GetSubMode() == SMODE_DIST_GET_HEAD || GetSubMode() == SMODE_DIST_GET_BODY) {
						globalState.dist_state.lastSubMode = GetSubMode();
						SetSubMode(SMODE_DIST_GET_PAUSE);
						BUTTON_SetText(hItem, "�����������");
					} else if (GetSubMode() == SMODE_DIST_GET_PAUSE) {
						SetSubMode(globalState.dist_state.lastSubMode);
						BUTTON_SetText(hItem, "����������\n�����");
				}
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
    case ID_BUTTON_3: // Notifications sent by 'BackBtn'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				//SetStateMode(MODE_NOTHING);
				SetSubMode(SMODE_DIST_EXIT);
				GUI_EndDialog(pMsg->hWin, SCR_MAIN);
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
*       CreateWindow
*/

WM_HWIN CreateRunDistWnd(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/