/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_ILI9341_MCU_H_
#define _PICO_ILI9341_MCU_H_

/* ========================== includes ========================== */
#include "pico/ili9341_hwcfg.h"

/* ======================== definitions ========================= */

typedef enum {
  LCD_SUCCESS = 0,  		/**< @brief Command completed successfully */
  LCD_UNKNOWN_ERROR = -1,   /**< @brief An unknown error occured. */
  LCD_NOT_INIT = -3,   		/**< @brief The LCD interface has not yet
                             * been setup.*/
  LCD_DMA_ERR = -4,			/**< @brief An error has occcured setup up or
                             * using the DMA.*/
  LCD_PIO_ERR = -5          /**< @brief An error has occcured setup up or
                             * using the PIO.*/
} lcd_error_t ; 

/* --------------------- screen mode handling --------------------*/

#define lcd_get_screen_width() (SCREEN_WIDTH)
#define lcd_get_screen_height() (SCREEN_HEIGHT)
#define lcd_get_screen_bpp() (LCD_COLORDEPTH)

/* ====================== function declarations ====================== */
lcd_error_t  lcd_init();
void lcd_set_speed(uint32_t freq);

/* ---------------------- LCD data transmission ---------------------- */
lcd_error_t  lcd_send_framebuffer(void* buf, uint32_t buffersize);
void lcd_wait_ready();
int  lcd_check_ready();

/* ------------------------ LCD hardware ctrl -------------------------*/
void lcd_set_backlight(uint8_t level);

void lcd_set_addr(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void lcd_enable_te();
void lcd_disable_te();
bool lcd_get_vblank();


/* ======================== defines ============================= */
/* ---------------------- screen dimensions ----------------------*/
// physical values
#define PHYS_SCREEN_WIDTH   240
#define PHYS_SCREEN_HEIGHT  320

/* --------------------- screen mode handling --------------------*/
// Generic commands used by ILI9341_eSPI.cpp
#define ILI9341_NOP     0x00
#define ILI9341_SWRST   0x01

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C

#define ILI9341_RAMRD   0x2E
#define ILI9341_IDXRD   0xDD // ILI9341 only, indexed control register read

#define ILI9341_MADCTL  0x36
#define ILI9341_MAD_MY  0x80
#define ILI9341_MAD_MX  0x40
#define ILI9341_MAD_MV  0x20
#define ILI9341_MAD_ML  0x10
#define ILI9341_MAD_BGR 0x08
#define ILI9341_MAD_MH  0x04
#define ILI9341_MAD_RGB 0x00

#ifdef LCD_PIN_INVERSION
  #define ILI9341_MAD_COLOR_ORDER ILI9341_MAD_RGB
#else
  #define ILI9341_MAD_COLOR_ORDER ILI9341_MAD_BGR
#endif

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21


// All ILI9341 specific commands some are used by init()
#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_TEOFF   0x34
#define ILI9341_TEON    0x35
#define ILI9341_MADCTL  0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT  0x3A

#define ILI9341_WRDISBV  0x51
#define ILI9341_RDDISBV  0x52
#define ILI9341_WRCTRLD  0x53

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID4   0xD3
#define ILI9341_RDINDEX 0xD9
#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDIDX   0xDD // TBC

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04

#define ILI9341_COLOR_SET  0x2D

struct ili9341_mcu_config {
    uint gpio_clk;
    uint gpio_rst;
    uint gpio_dc;
    uint gpio_pin0;
};

void ili9341_mcu_init(const struct ili9341_mcu_config* config, uint16_t width, uint16_t height);
void ili9341_mcu_write(const void* data, size_t len);
void ili9341_mcu_put(uint16_t pixel);
void ili9341_mcu_fill(uint16_t pixel);
void ili9341_mcu_set_cursor(uint16_t x, uint16_t y);
void ili9341_mcu_vertical_scroll(uint16_t row);
void ili9341_mcu_partial_area(uint16_t start, uint16_t end);

#endif
