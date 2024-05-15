/*
 * pplib - a library for the Pico Held handheld
 *
 * Copyright (C) 2023 Daniel Kammer (daniel.kammer@web.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma GCC optimize("Ofast")

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/bootrom.h"

#include "controls.h"
#include "pico/ili9341_hwcfg.h"

uint16_t _analog_x_0 = 1600;
uint16_t _analog_y_0 = 1600;
uint16_t _analog_max_x = 2400;
uint16_t _analog_min_x = 800;
uint16_t _analog_max_y = 2400;
uint16_t _analog_min_y = 800;

int _ctrl_lst_ret_x;
int _ctrl_lst_ret_y;

// Values below this threshold are considered as zero
#define zeroThreshold 25   // 5^2
// Decrement the max. amplitude by this value, so that 100% can be reached
#define minmaxTolerance 10

void ctrl_recalib_analog() {
  adc_select_input(0); // 26
  _analog_x_0 = adc_read();
  adc_select_input(1); // 27
  _analog_y_0 = adc_read();
  _analog_max_x = _analog_x_0 + 800;
  _analog_min_x = _analog_x_0 - 800;
  _analog_max_y = _analog_y_0 + 800;
  _analog_min_y = _analog_y_0 - 800;
}

int8_t ctrl_analog_x_state() {
  int8_t ret;
  adc_select_input(0); // 26
  uint16_t cur_x = adc_read();

  if (cur_x > _analog_max_x)
    _analog_max_x = cur_x - minmaxTolerance;

  if (cur_x < _analog_min_x)
    _analog_min_x = cur_x + minmaxTolerance;

  if (cur_x > _analog_x_0)    
    ret = (cur_x - _analog_x_0) * 100 / (_analog_max_x - _analog_x_0);
  else
    ret = (cur_x - _analog_x_0) * 100 / (_analog_x_0 - _analog_min_x);

   // if |ret| < zeroThreshold^2 return zero
  if (ret * ret < zeroThreshold)
   ret = 0;
  
  // if the new value differs less than 10% from the old one it's being discarded
  if ((ret - _ctrl_lst_ret_x) * (ret - _ctrl_lst_ret_x) > zeroThreshold*zeroThreshold) {
    _ctrl_lst_ret_x = ret;
  } else {
    ret = _ctrl_lst_ret_x;
  }

  if (ret < -100)
    ret = -100;
  if (ret > 100)
    ret = 100;

  return (ret / 10) * -10;
}

int8_t ctrl_analog_y_state() {
  int8_t ret;
  adc_select_input(7); // 27
  uint16_t cur_y = adc_read();

  if (cur_y > _analog_max_y)
    _analog_max_y = cur_y - minmaxTolerance;

  if (cur_y < _analog_min_y)
    _analog_min_y = cur_y + minmaxTolerance;

  if (cur_y > _analog_y_0)    
    ret = (cur_y - _analog_y_0) * 100 / (_analog_max_y - _analog_y_0);
  else
    ret = (cur_y - _analog_y_0) * 100 / (_analog_y_0 - _analog_min_y);

  // if |ret| < zeroThreshold^2 return zero
  if (ret * ret < zeroThreshold)
   ret = 0;

  // if the new value differs less than 10% from the old one it's being discarded
  if ((ret - _ctrl_lst_ret_y) * (ret - _ctrl_lst_ret_y) > zeroThreshold*zeroThreshold) {
    _ctrl_lst_ret_y = ret;
  } else {
    ret = _ctrl_lst_ret_y;
  }

  if (ret < -100)
    ret = -100;
  if (ret > 100)
    ret = 100;

  return (ret / 10) * 10;
}

uint16_t ctrl_dpad_state() {
  adc_select_input(0); // 26
  uint16_t cur_x = adc_read();
  adc_select_input(1); // 27
  uint16_t cur_y = adc_read();

  uint16_t value = 0;

  if (cur_x > _analog_max_x)
    _analog_max_x = cur_x;

  if (cur_y > _analog_max_y)
    _analog_max_y = cur_y;

  if (cur_x < _analog_min_x)
    _analog_min_x = cur_x;

  if (cur_y < _analog_min_y)
    _analog_min_y = cur_y; 

  if (cur_x > 2600)
    value |= DPAD_LEFT;

  if (cur_x < 1200)
    value |= DPAD_RIGHT;

  if (cur_y < 1200)
    value |= DPAD_UP;

  if (cur_y > 2600)
    value |= DPAD_DOWN;

  return value;
}

// Enters bootloader in case all buttons are pressed during system reset
bool bl_check_selection() {
  uint16_t buttons = ctrl_button_state();

  if ( (buttons & BUTTON_1) && 
       (buttons & BUTTON_2) && 
	   (buttons & BUTTON_3) )
    return true;
  else
	return false;
}

// Launches the bootloader
void bl_launch_bl() {
	reset_usb_boot(0, 0);
}

uint16_t ctrl_button_state() {
  uint16_t value = 0;

  if (gpio_get(PIN_BUTTON_1) == BUTTON_PRESSED)
    value |= BUTTON_1;

  if (gpio_get(PIN_BUTTON_2) == BUTTON_PRESSED)
    value |= BUTTON_2;

  if (gpio_get(PIN_BUTTON_3) == BUTTON_PRESSED)
    value |= BUTTON_3;

  return value;
}

int ctrl_init() {

  adc_init();
  
  adc_gpio_init(PIN_ANALOG_X); // 26
  adc_gpio_init(PIN_ANALOG_Y); // 27
  
  ctrl_recalib_analog();

  gpio_init(PIN_BUTTON_1);
  gpio_set_dir(PIN_BUTTON_1, GPIO_IN);
  gpio_pull_down(PIN_BUTTON_1);
  
  gpio_init(PIN_BUTTON_2);
  gpio_set_dir(PIN_BUTTON_2, GPIO_IN);
  gpio_pull_down(PIN_BUTTON_2);
  
  gpio_init(PIN_BUTTON_3);
  gpio_set_dir(PIN_BUTTON_3, GPIO_IN);
  gpio_pull_down(PIN_BUTTON_3);

  if  (bl_check_selection())
    bl_launch_bl();
  
  return 0;
}
