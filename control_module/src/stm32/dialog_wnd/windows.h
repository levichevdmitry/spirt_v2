#ifndef WINDOWDLG_H
#define WINDOWDLG_H

#include "DIALOG.h"
#include "registers.h"
#include "settings.h"
#include "beeper.h"
#include "valve.h"
#include "control.h"
#include "stm32f4xx_hal.h"
#include "eeprom.h"
#include "utils.h"
#include "wifi.h"

extern GUI_CONST_STORAGE GUI_FONT GUI_FontArial32;

WM_HWIN CreateSelTests(void);
WM_HWIN CreateEmTests(void);
WM_HWIN CreateSenTests(void);
WM_HWIN CreateSelSettings(void);
WM_HWIN CreateMainWindow(void);
WM_HWIN CreateSettingsTempWnd(void);
WM_HWIN CreateSelCalibrations(void);
WM_HWIN CreateSettingsPmWnd(void);
WM_HWIN CreatePSenWnd(void);
WM_HWIN CreateCAlibrationValveWnd(void);
WM_HWIN CreateDistSetWnd(void);
WM_HWIN CreateRectSetWnd(void);
WM_HWIN CreateRunClrWnd(void);
WM_HWIN CreateRunDistWnd(void);
WM_HWIN CreateRunRectWnd(void);

#endif

