/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 The Digital Mermaid.
  * All rights reserved.
  *
  * This software is licensed under the GPL v3, the terms can be found in the 
  * LICENSE file in the root directory of this software component.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "nd_lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

// External variables
extern I2C_HandleTypeDef hi2c_m10kw;

// Local functions
void nd_lcd_send_packet(unsigned int);
void nd_lcd_i2c_busy(unsigned int);

// Initalize LCD
void nd_lcd_init(void) {
  	HAL_Delay(100);			// waiting for LCD to initialize
		
	nd_lcd_clear_screen();
// Set Contrast
	tx_packet[0] = 0xFE;
	tx_packet[1] = 0x52;
	tx_packet[2] = 40;		// contrast 1 - 50
	nd_lcd_send_packet(3);

// Set Backlight Level
	tx_packet[1] = 0x53;
	tx_packet[2] = 15;		// backlight 1 - 15
	nd_lcd_send_packet(3);

	tx_packet[1] = 0x48;	// underline cursor off
	nd_lcd_send_packet(2);
	
	HAL_Delay(100);
}

// Set the cursor position
void nd_lcd_set_cursor(unsigned int mode) {
  if (mode == 1) {
    // Show cursor
  } else {
    // Hide cursor
  }
}

// Clear the screen 
void nd_lcd_clear_screen(void) {

}

// ??
//void nd_lcd_i2c_busy(unsigned int){}

// Clear one line on the screen
void nd_lcd_clear_line(unsigned int line) {

}


