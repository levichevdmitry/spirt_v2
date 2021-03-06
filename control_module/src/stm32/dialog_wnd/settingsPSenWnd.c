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
#define ID_WINDOW_0    (GUI_ID_USER + 0x00)
#define ID_TEXT_0    (GUI_ID_USER + 0x01)
#define ID_TEXT_1    (GUI_ID_USER + 0x02)
#define ID_TEXT_2    (GUI_ID_USER + 0x03)
#define ID_TEXT_3    (GUI_ID_USER + 0x04)
#define ID_TEXT_4    (GUI_ID_USER + 0x05)
#define ID_BUTTON_0    (GUI_ID_USER + 0x06)
#define ID_SPINBOX_0    (GUI_ID_USER + 0x07)
#define ID_SPINBOX_1    (GUI_ID_USER + 0x08)
#define ID_EDIT_0    (GUI_ID_USER + 0x09)
#define ID_EDIT_1    (GUI_ID_USER + 0x0A)
#define ID_BUTTON_1    (GUI_ID_USER + 0x0B)
#define ID_BUTTON_2    (GUI_ID_USER + 0x0C)
#define ID_TIMER_CONVERT_MONITOR	(GUI_ID_USER + 0x0D)


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
  { TEXT_CreateIndirect, "Text", ID_TEXT_0, 200, 10, 400, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_1, 10, 60, 200, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_2, 10, 120, 200, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_3, 10, 200, 200, 30, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_4, 10, 260, 200, 30, 0, 0x64, 0 },
  { BUTTON_CreateIndirect, "BackBtn", ID_BUTTON_0, 585, 361, 200, 100, 0, 0x0, 0 },
  { SPINBOX_CreateIndirect, "Spinbox", ID_SPINBOX_0, 250, 50, 120, 50, 0, 0x0, 0 },
  { SPINBOX_CreateIndirect, "Spinbox", ID_SPINBOX_1, 250, 110, 120, 50, 0, 0x0, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_0, 250, 200, 80, 30, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_1, 250, 260, 80, 30, 0, 0x64, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_1, 360, 190, 140, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_2, 360, 250, 140, 50, 0, 0x0, 0 },
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
WM_HTIMER hTimerPSen;
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
		
		hTimerPSen = WM_CreateTimer(hItem, ID_TIMER_CONVERT_MONITOR, 500, 0);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetText(hItem, "���������� ������� ��������");
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "���. ����.");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "����. ����.");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "���. ���");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
    TEXT_SetFont(hItem, &GUI_FontArial32);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "����. ���");
    //
    // Initialization of 'BackBtn'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "���������");
    //
    // Initialization of 'Spinbox'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_0);
    SPINBOX_SetFont(hItem, &GUI_FontArial32);
		hEdit = SPINBOX_GetEditHandle(hItem);
		EDIT_SetTextAlign(hEdit, GUI_TA_VCENTER);
		EDIT_SetDecMode(hEdit, Settings.p_min.value, Settings.p_min.min_val, Settings.p_min.max_val, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'Spinbox'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SPINBOX_1);
    SPINBOX_SetFont(hItem, &GUI_FontArial32);
		hEdit = SPINBOX_GetEditHandle(hItem);
		EDIT_SetTextAlign(hEdit, GUI_TA_VCENTER);
		EDIT_SetDecMode(hEdit, Settings.p_max.value, Settings.p_max.min_val, Settings.p_max.max_val, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    EDIT_SetText(hItem, "123");
    EDIT_SetFont(hItem, &GUI_FontArial32);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		EDIT_SetDecMode(hItem, Settings.adc_min.value, Settings.adc_min.min_val, Settings.adc_min.max_val, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);
    EDIT_SetText(hItem, "123");
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    EDIT_SetFont(hItem, &GUI_FontArial32);
		EDIT_SetDecMode(hItem, Settings.adc_max.value, Settings.adc_max.min_val, Settings.adc_max.max_val, 0, GUI_EDIT_SUPPRESS_LEADING_ZEROES);
    //
    // Initialization of 'Button'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "���. ���.");
    //
    // Initialization of 'Button'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
    BUTTON_SetFont(hItem, &GUI_FontArial32);
    BUTTON_SetText(hItem, "���. ����.");
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
		
	case WM_TIMER:
		
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
			EDIT_SetValue(hItem, Settings.adc_min.value);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);
			EDIT_SetValue(hItem, Settings.adc_max.value);
			
			WM_RestartTimer(pMsg->Data.v, 500);
	
		break;
	
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'BackBtn'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
					BeeperOn();
					SaveSettings(); // Save settings to eeprom
					BeeperOff();
					GUI_EndDialog(pMsg->hWin, SCR_SETTINGS_SEL_CAL);
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
    case ID_SPINBOX_0: // Notifications sent by 'Spinbox'
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
				Settings.p_min.value = SPINBOX_GetValue(hItem);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SPINBOX_1: // Notifications sent by 'Spinbox'
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
				Settings.p_max.value = SPINBOX_GetValue(hItem);
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
    case ID_BUTTON_1: // Notifications sent by 'Set min ADC'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				Settings.adc_min.value = pressure_capture;
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
    case ID_BUTTON_2: // Notifications sent by 'Set max ADC'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				Settings.adc_max.value = pressure_capture;
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

WM_HWIN CreatePSenWnd(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
