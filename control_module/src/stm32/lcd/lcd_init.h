#ifndef LCD_INIT_H
#define LCD_INIT_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

typedef struct
{
  volatile uint16_t LCD_REG;
  volatile uint16_t LCD_RAM;
} __LCD_TypeDef;

// Note: LCD /CS is NE1 - Bank 1 of NOR/SRAM Bank 1~4
#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0001fffE))
#define __LCD                ((__LCD_TypeDef *) LCD_BASE)

#define DISP_WIDTH  800
#define DISP_HEIGHT 480

#define PIXEL_COUNT (DISP_WIDTH+1)*DISP_HEIGHT

#define HDP		(DISP_WIDTH - 1)
#define HT 		928
#define HPS		46
#define LPS		15
#define HPW		48

#define VDP		(DISP_HEIGHT - 1)
#define VT		525
#define VPS		16
#define FPS		8
#define VPW		16


void SSD1963_Init(void);

void FSMC_LcdWriteCmd (uint16_t val);
void FSMC_LcdWriteData (uint16_t val);

#endif
