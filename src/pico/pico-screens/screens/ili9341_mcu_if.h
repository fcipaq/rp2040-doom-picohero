// DESCRIPTION:
// LED / LCD screen-specific logic

#ifndef __ILI9341_MCU__
#define __ILI9341_MCU__

#include "shared.h"

#include "pico.h"

#include <stdlib.h>
#include "pico/ili9341_mcu.h"

#define MEMORY_WIDTH 320
#define MEMORY_HEIGHT 200

#define LCD_WIDTH 320
#define LCD_HEIGHT 240

#define SCREEN_WIDTH_OFFSET 0

void ili9341_mcu_if_initScreen(void);
void ili9341_mcu_if_handleFrameStart(uint8_t frame);
void ili9341_mcu_if_handleScanline(uint16_t *line, int scanline);

#endif
