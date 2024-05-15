/* Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <string.h>

#include "hardware/gpio.h"

#include "pico/ili9341_hwcfg.h"

#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "pico/lcd_pio16.h"

/* ==================== forward declarations ==================== */
void lcd_controller_init();

/* ========================= variables ========================== */
/* ----------------------------- NN -----------------------------*/
bool lcd_init_complete = false;

/* ----------------------- backlight -----------------------------*/
pwm_config lcd_bl_pwm_cfg;

/* ----------------------- scanout -----------------------------*/
volatile uint16_t dma_scanline = 0;
volatile uint8_t scanline_even = 0;

volatile uint16_t* cur_scanout_buf = NULL;

/* ----------------------------- PIO -----------------------------*/
PIO lcd_pio = pio0;
int8_t lcd_pio_tft_sm = 0;

// SM stalled mask
uint32_t pio_pull_stall_mask = 0;

// SM jump instructions to change SM behaviour
uint32_t pio_instr_jmp8 = 0;

/* ----------------------------- DMA -----------------------------*/
#define LCD_NUM_DMA 1

int32_t lcd_dma_chan[LCD_NUM_DMA];
dma_channel_config lcd_dma_config[LCD_NUM_DMA];

uint8_t lcd_dma_enabled = 0;

bool lcd_dma_busy(void) {
  if (!lcd_dma_enabled)
    return false;

  if (dma_channel_is_busy(lcd_dma_chan[0]))
    return true;
  else
    return false;
}  // lcd_dma_busy

void lcd_dma_wait(void) {
  if (!lcd_dma_enabled)
    return;

  while (dma_channel_is_busy(lcd_dma_chan[0]));
}

lcd_error_t lcd_dma_init() {
  if (lcd_dma_enabled)
    return LCD_SUCCESS;

  for (int h = 0; h < LCD_NUM_DMA; h++) {
    lcd_dma_chan[h] = dma_claim_unused_channel(true);

    if (lcd_dma_chan[h] < 0)
      return LCD_DMA_ERR;
  }

  for (int h = 0; h < LCD_NUM_DMA; h++)
    lcd_dma_config[h] = dma_channel_get_default_config(lcd_dma_chan[h]);

  // WARNING: if DMA size is changed here, transmission length in lcd_show_framebuffer/lcd_send_framebuffer must also be revised
  channel_config_set_transfer_data_size(&lcd_dma_config[0], DMA_SIZE_16);
  channel_config_set_dreq(&lcd_dma_config[0], pio_get_dreq(lcd_pio, lcd_pio_tft_sm, true));
  channel_config_set_bswap(&lcd_dma_config[0], true);

  lcd_dma_enabled = true;

  return LCD_SUCCESS;
}  // dmaInit

void lcd_dma_shutdown(void) {
  if (!lcd_dma_enabled)
    return;

  dma_channel_unclaim(lcd_dma_chan[0]);

  lcd_dma_enabled = false;
}  // lcd_dma_shutdown

int lcd_check_ready() {

  return lcd_dma_busy();

}  // lcd_check_ready

void lcd_wait_ready() {
  lcd_dma_wait();
}  // lcd_wait_ready

void lcd_set_speed(uint32_t freq) {
  //  if (!initComplete)
  //    return LCD_NOT_INIT;

  uint32_t fcpu = clock_get_hz(clk_sys);  // broken

  uint16_t clock_div = fcpu / freq;
  uint16_t fract_div = (uint16_t)(((float)fcpu / (float)freq - (float)clock_div) * 256.0);

  // max speed exceeded
  if (clock_div == 0) {
    clock_div = 1;
    fract_div = 0;
  }

  // Set clock divider and fractional divider
  pio_sm_set_enabled(lcd_pio, lcd_pio_tft_sm, false);

  // automatic frequency calculation is broken
  clock_div = 8;
  fract_div = 47;
  
  pio_sm_set_clkdiv_int_frac(lcd_pio, lcd_pio_tft_sm, clock_div, fract_div);
  pio_sm_set_enabled(lcd_pio, lcd_pio_tft_sm, true);
  
}  // lcd_set_speed

lcd_error_t lcd_pio_init() {

  lcd_pio_tft_sm = pio_claim_unused_sm(lcd_pio, false);

  if (lcd_pio_tft_sm < 0)
    return LCD_PIO_ERR;

  uint32_t program_offset = pio_add_program(lcd_pio, &lcd_output_program);

  pio_gpio_init(lcd_pio, PIN_LCD_WR);

  for (int i = 0; i < 8; i++)
    pio_gpio_init(lcd_pio, PIN_LCD_D0 + i);

  pio_sm_set_consecutive_pindirs(lcd_pio, lcd_pio_tft_sm, PIN_LCD_WR, 1, true);
  pio_sm_set_consecutive_pindirs(lcd_pio, lcd_pio_tft_sm, PIN_LCD_D0, 8, true);

  pio_sm_config c = lcd_output_program_get_default_config(program_offset);

  sm_config_set_sideset_pins(&c, PIN_LCD_WR);
  sm_config_set_out_pins(&c, PIN_LCD_D0, 8);
  sm_config_set_clkdiv_int_frac(&c, 10, 1);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  sm_config_set_out_shift(&c, true, true, 16);  // OSR shifts out to the right, autopull, pull threshold is 16 bits

  pio_sm_init(lcd_pio, lcd_pio_tft_sm, program_offset + lcd_output_offset_tx_dat, &c);  // ori
  pio_sm_set_enabled(lcd_pio, lcd_pio_tft_sm, true);

  pio_pull_stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + lcd_pio_tft_sm);  // ori
  pio_instr_jmp8 = pio_encode_jmp(program_offset + lcd_output_offset_tx_cmd);

  return LCD_SUCCESS;
}  // pioInit

void lcd_pio_wait() {
  // Wait for the PIO to stall (SM pull request finds no data in TX FIFO)
  // This is used to detect when the SM is idle and hence ready for a jump instruction
  lcd_pio->fdebug = pio_pull_stall_mask;
  while (!(lcd_pio->fdebug & pio_pull_stall_mask));
}  // lcd_pio_wait

void set_rs(uint8_t value) {
  gpio_put(PIN_LCD_DC, value);
}

void set_rst(uint8_t value) {
  gpio_put(PIN_LCD_RST, value);
}

void lcd_send_dat_byte(uint8_t dat) {
  // jump to pio_instr_jmp8
  lcd_pio->sm[lcd_pio_tft_sm].instr = pio_instr_jmp8;

  // write dat to SM FIFO
  lcd_pio->txf[lcd_pio_tft_sm] = dat;

  lcd_pio_wait();
}  // lcd_send_dat_byte

void lcd_send_cmd_byte(uint8_t cmd) {
  set_rs(0);

  // jump to pio_instr_jmp8
  lcd_pio->sm[lcd_pio_tft_sm].instr = pio_instr_jmp8;

  // write cmd to SM FIFO
  lcd_pio->txf[lcd_pio_tft_sm] = cmd;

  lcd_pio_wait();

  set_rs(1);
}  // lcd_send_cmd_byte

uint32_t testy;

// send a buffer of DATA bytes to the display
lcd_error_t lcd_send_framebuffer(void* buf, uint32_t buffersize) {
  if ((buffersize == 0) || (!lcd_dma_enabled))
    return LCD_NOT_INIT;

  lcd_dma_wait();
  lcd_pio_wait();

  dma_channel_configure(lcd_dma_chan[0], &lcd_dma_config[0], &lcd_pio->txf[lcd_pio_tft_sm], buf, buffersize, true);

  return LCD_SUCCESS;
}  // lcd_send_framebuffer

void lcd_set_backlight(uint8_t level) {
  pwm_set_gpio_level(PIN_LCD_BL_PWM, level);
}

lcd_error_t lcd_init() {
  if (lcd_init_complete)
    return LCD_SUCCESS;

  // configure pin modes
  gpio_init(PIN_LCD_RST);
  gpio_set_dir(PIN_LCD_RST, GPIO_OUT);

  gpio_init(PIN_LCD_DC);
  gpio_set_dir(PIN_LCD_DC, GPIO_OUT);

  if (lcd_pio_init() != LCD_SUCCESS)
    return LCD_PIO_ERR;

  lcd_set_speed(LCD_PIO_SPEED * 1000000);

  // LCD controller setup (hardware dependent)
  lcd_controller_init();

  if (lcd_dma_init() != LCD_SUCCESS)
    return LCD_DMA_ERR;

  // setup backlight PWM
//  gpio_init(PIN_LCD_BL_PWM);
//  gpio_set_dir(PIN_LCD_BL_PWM, GPIO_OUT);
//  gpio_put(PIN_LCD_BL_PWM, 1);
  
  lcd_bl_pwm_cfg = pwm_get_default_config();
  gpio_set_function(PIN_LCD_BL_PWM, GPIO_FUNC_PWM);
  uint slice_num_backlight = pwm_gpio_to_slice_num(PIN_LCD_BL_PWM);
  pwm_clear_irq(slice_num_backlight);
  pwm_config_set_clkdiv(&lcd_bl_pwm_cfg, 1000000.0f);
  pwm_config_set_wrap(&lcd_bl_pwm_cfg, 100);
  pwm_init(slice_num_backlight, &lcd_bl_pwm_cfg, true);

  lcd_set_backlight(20);

  lcd_init_complete = true;
 
  return LCD_SUCCESS;
}  // lcd_init

void lcd_enable_te() {
  //lcd_send_cmd_byte(ILI9341_TEON);
  //lcd_send_dat_byte(0x00); // V-blank info only
}

void lcd_set_addr(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {

  lcd_dma_wait();
  lcd_pio_wait();

  uint8_t coordinates[8] = {
    (uint8_t) (x1 >> 8),
    (uint8_t) (x1 & 0xff),
    (uint8_t) (x2 >> 8),
    (uint8_t) (x2 & 0xff),
    (uint8_t) (y1 >> 8),
    (uint8_t) (y1 & 0xff),
    (uint8_t) (y2 >> 8),
    (uint8_t) (y2 & 0xff)
  };
  
  lcd_send_cmd_byte(ILI9341_CASET); // column address set
  for (int i = 0; i < 4; i++)
    lcd_send_dat_byte(coordinates[i]);

  lcd_send_cmd_byte(ILI9341_PASET);  // page address set
  for (int i = 0; i < 4; i++)
    lcd_send_dat_byte(coordinates[i + 4]);

  lcd_send_cmd_byte(ILI9341_RAMWR); //memory write 

  // WA for a PIO program bug that causes the state transistion
  // being too ealry before the pins have been written (?)
  for (int i = 0; i < 2; i++)
    lcd_send_dat_byte(0x00);
}

void lcd_controller_init() {
  gpio_init(PIN_LCD_TE);
  gpio_set_dir(PIN_LCD_TE, GPIO_IN);

  set_rst(1);
  sleep_ms(20);
  set_rst(0);
  sleep_ms(20);
  set_rst(1);

  sleep_ms(50);
  
  lcd_send_cmd_byte(0xEF);
  lcd_send_dat_byte(0x03);
  lcd_send_dat_byte(0x80);
  lcd_send_dat_byte(0x02);

  lcd_send_cmd_byte(0xCF);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0XC1);
  lcd_send_dat_byte(0X30);

  lcd_send_cmd_byte(0xED);
  lcd_send_dat_byte(0x64);
  lcd_send_dat_byte(0x03);
  lcd_send_dat_byte(0X12);
  lcd_send_dat_byte(0X81);

  lcd_send_cmd_byte(0xE8);
  lcd_send_dat_byte(0x85);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0x78);

  lcd_send_cmd_byte(0xCB);
  lcd_send_dat_byte(0x39);
  lcd_send_dat_byte(0x2C);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0x34);
  lcd_send_dat_byte(0x02);

  lcd_send_cmd_byte(0xF7);
  lcd_send_dat_byte(0x20);

  lcd_send_cmd_byte(0xEA);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0x00);

/*
  lcd_send_cmd_byte(ILI9341_PWCTR1);    //Power control
  lcd_send_dat_byte(0x23);   //VRH[5:0]

  lcd_send_cmd_byte(ILI9341_PWCTR2);    //Power control
  lcd_send_dat_byte(0x10);   //SAP[2:0];BT[3:0]

  lcd_send_cmd_byte(ILI9341_VMCTR1);    //VCM control
  lcd_send_dat_byte(0x3e);
  lcd_send_dat_byte(0x28);
*/

  lcd_send_cmd_byte(ILI9341_PWCTR1);    //Power control
  lcd_send_dat_byte(0x0b);   // 3.4V reference
  
  lcd_send_cmd_byte(ILI9341_PWCTR2);    //Power control
  lcd_send_dat_byte(0x10);   //SAP[2:0];BT[3:0]

  lcd_send_cmd_byte(ILI9341_VMCTR1);    //VCM control
  lcd_send_dat_byte(0x1c);  // 3.4V VCOMH
  lcd_send_dat_byte(0x28);  // -0.5V VCOML
  
  lcd_send_cmd_byte(ILI9341_VMCTR2);    //VCM control2
  lcd_send_dat_byte(0x86);  //--

  lcd_send_cmd_byte(ILI9341_PIXFMT);
  lcd_send_dat_byte(0x55);

  lcd_send_cmd_byte(ILI9341_FRMCTR1);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0x13); // 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz

  lcd_send_cmd_byte(ILI9341_DFUNCTR);    // Display Function Control
  lcd_send_dat_byte(0x08);
  lcd_send_dat_byte(0x82);
  lcd_send_dat_byte(0x27);

  lcd_send_cmd_byte(0xF2);    // 3Gamma Function Disable
  lcd_send_dat_byte(0x00);

  lcd_send_cmd_byte(ILI9341_GAMMASET);    //Gamma curve selected
  lcd_send_dat_byte(0x01);

  lcd_send_cmd_byte(ILI9341_GMCTRP1);    //Set Gamma
  lcd_send_dat_byte(0x0F);
  lcd_send_dat_byte(0x31);
  lcd_send_dat_byte(0x2B);
  lcd_send_dat_byte(0x0C);
  lcd_send_dat_byte(0x0E);
  lcd_send_dat_byte(0x08);
  lcd_send_dat_byte(0x4E);
  lcd_send_dat_byte(0xF1);
  lcd_send_dat_byte(0x37);
  lcd_send_dat_byte(0x07);
  lcd_send_dat_byte(0x10);
  lcd_send_dat_byte(0x03);
  lcd_send_dat_byte(0x0E);
  lcd_send_dat_byte(0x09);
  lcd_send_dat_byte(0x00);

  lcd_send_cmd_byte(ILI9341_GMCTRN1);    //Set Gamma
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0x0E);
  lcd_send_dat_byte(0x14);
  lcd_send_dat_byte(0x03);
  lcd_send_dat_byte(0x11);
  lcd_send_dat_byte(0x07);
  lcd_send_dat_byte(0x31);
  lcd_send_dat_byte(0xC1);
  lcd_send_dat_byte(0x48);
  lcd_send_dat_byte(0x08);
  lcd_send_dat_byte(0x0F);
  lcd_send_dat_byte(0x0C);
  lcd_send_dat_byte(0x31);
  lcd_send_dat_byte(0x36);
  lcd_send_dat_byte(0x0F);

  lcd_send_cmd_byte(ILI9341_SLPOUT);    //Exit Sleep
 
  set_rs(0);
  sleep_ms(120);
  set_rs(1);
  
  //lcd_enable_te();
  
  lcd_send_cmd_byte(ILI9341_DISPON);    //Display on

  lcd_send_cmd_byte(ILI9341_MADCTL);    // Memory Access Control
  
  #if LCD_ROTATION==0
    lcd_send_dat_byte(ILI9341_MAD_MX | ILI9341_MAD_COLOR_ORDER); // Rotation 0 (portrait mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_WIDTH - 1, PHYS_SCREEN_HEIGHT - 1);
  #elif LCD_ROTATION==1
    lcd_send_dat_byte(ILI9341_MAD_MV | ILI9341_MAD_MX | ILI9341_MAD_MY | ILI9341_MAD_COLOR_ORDER); // Rotation 90 (landscape mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_HEIGHT - 1, PHYS_SCREEN_WIDTH - 1);
  #elif LCD_ROTATION==2
    lcd_send_dat_byte(ILI9341_MAD_MY | ILI9341_MAD_COLOR_ORDER); // Rotation 180 (portrait mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_WIDTH - 1, PHYS_SCREEN_HEIGHT - 1);
  #elif LCD_ROTATION==3
    lcd_send_dat_byte(ILI9341_MAD_MV | ILI9341_MAD_COLOR_ORDER); // Rotation 270 (landscape mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_HEIGHT - 1, PHYS_SCREEN_WIDTH - 1);
  #endif

  // clear screen to black
  for (uint32_t jj = 0; jj < PHYS_SCREEN_WIDTH * PHYS_SCREEN_HEIGHT * 2; jj++)
    lcd_send_dat_byte(0x3a);
	
}

static struct ili9341_mcu_config ili9341_mcu_cfg;
static uint16_t ili9341_mcu_width;
static uint16_t ili9341_mcu_height;
static bool ili9341_mcu_data_mode = false;

bool ili9341_mcu_get_vblank() {
//  return digitalRead(PIN_LCD_TE);
  return true;
}

static void ili9341_mcu_cmd(uint8_t cmd, const uint8_t* data, size_t len) {
  lcd_send_cmd_byte(cmd);
  
  for (int i = 0; i < len; i++)
    lcd_send_cmd_byte(data[i]);  
}

void ili9341_mcu_caset(uint16_t xs, uint16_t xe)
{
}

void ili9341_mcu_raset(uint16_t ys, uint16_t ye)
{
}

void ili9341_mcu_init(const struct ili9341_mcu_config* config, uint16_t width, uint16_t height)
{
    memcpy(&ili9341_mcu_cfg, config, sizeof(ili9341_mcu_cfg));
    ili9341_mcu_width = width;
    ili9341_mcu_height = height;

    lcd_init();  
}

void ili9341_mcu_ramwr()
{
  lcd_send_cmd_byte(ILI9341_RAMWR); //memory write 
}

void ili9341_mcu_write(const void* data, size_t len)
{
  lcd_send_framebuffer((void*) data, (uint32_t) len);
}

void ili9341_mcu_put(uint16_t pixel)
{
    ili9341_mcu_write(&pixel, sizeof(pixel));
}

void ili9341_mcu_fill(uint16_t pixel)
{
}

void ili9341_mcu_set_cursor(uint16_t x, uint16_t y)
{   
}

void ili9341_mcu_vertical_scroll(uint16_t row)
{
}

void ili9341_mcu_partial_area(uint16_t start, uint16_t end)
{
}
