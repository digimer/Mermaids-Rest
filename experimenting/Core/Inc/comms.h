/**
  ******************************************************************************
  * @file           : comms.h
  * @brief          : Header for comms.c file.
  *                   This file contains the display (serial, LCD, etc) defines
  *                   of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 The Digital Mermaid
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __COMMS_H__
#define __COMMS_H__

#include <inttypes.h>

/**
  * @brief Stores the switch settings of the system for display.
  * @param main: main switch (LCD display)
  * @param throttleStop: throttle stop switch
  * @param speed10kwHigh High speed switch for 10kw motor
  * @param speed10kwLow Low speed switch for 10kw motor
  * @param speed5kwHigh High speed switch for 5kw motor
  * @param speed5kwLow Low speed switch for 5kw motor
  * @retval None
  */
void lcdSetGlobalSwitches(uint8_t main, uint8_t throttleStop, uint8_t speed10kwHigh, uint8_t speed10kwLow, uint8_t speed5kwHigh, uint8_t speed5kwLow);

/**
  * @brief  Prints data to serial, LCDs, LED displays, etc.
  * @retval None  
  * @note   All but serial are disabled if the main switch is off.
  */
void commsPrint();

/**
  * @brief  Initialize Comms; LCD, serial, LED displays, etc.
  * @retval None  
*/
void commsInit();

#endif
