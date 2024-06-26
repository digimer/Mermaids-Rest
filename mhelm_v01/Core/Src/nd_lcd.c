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

#include "stm32f0xx_hal.h"
#include "main.h"
#include "nd_lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/********************************************************************
 function protocol:
 0xFE, 0x41 - display on        
 0xFE, 0x42 - display off      
 0xFE, 0x45 - set cursor position  
 0xFE, 0x46 - home cursor      
 0xFE, 0x47 - underline cursor on  
 0xFE, 0x48 - underline cursor off    
 0xFE, 0x49 - move cursor left 1 space  
 0xFE, 0x4A - move cursor right 1 space
 0xFE, 0x4B - blinking cursor on    
 0xFE, 0x4C - blinking cursor off  
 0xFE, 0x4E - back space.
 0xFE, 0x51 - clear screen        
 0xFE, 0x52 - set contrast  (1-50)    
 0xFE, 0x53 - set backlight brightness   (1-16)
 0xFE, 0x54 - load custom characters  

 0xFE, 0x55 - move screen left 1 space  
 0xFE, 0x56 - move screen right 1 space
 0xFE, 0x61 - change BAUD rate (1 - 8)  
 0xFE, 0x62 - change I2C address (0-255)
 0xFE, 0x70 - display version number  
 0xFE, 0x71 - display RS232 BAUD rate
 0xFE, 0x72 - display I2C address  
 0xFE, 0xFE - send next byte to command register  

 other built-in functions:
 - when either I2C or SPI is selected, the RS232 returns to 9600
   BAUD rate.  Vise versa, when RS232 is selected, the I2C address
   returnes to 50.

- if both I2C and SPI are selected, the module will display the
  firmware version number, then RS232 BAUD rate and I2C address.
  each item is displayed for 2 seconds and loops continuously.
*/

// External variables
extern I2C_HandleTypeDef hi2c_lcd;

// Local functions
void nd_lcd_send_packet(uint16_t);
void nd_lcd_i2c_busy(unsigned int);

// LCD data
uint8_t nc_lcd_tx_buf[32];

// Initalize LCD
void nd_lcd_init(void) {
  HAL_Delay(100);			// waiting for LCD to initialize
		
	//nd_lcd_clear_screen();

  // Display on
	nc_lcd_tx_buf[0] = 0xFE;
	nc_lcd_tx_buf[1] = 0x41;
	nd_lcd_send_packet(2);

  // Set Contrast
	nc_lcd_tx_buf[0] = 0xFE;
	nc_lcd_tx_buf[1] = 0x52;
	nc_lcd_tx_buf[2] = 40;		// contrast 1 - 50
	nd_lcd_send_packet(3);

  // Set Backlight Level
	nc_lcd_tx_buf[1] = 0x53;
	nc_lcd_tx_buf[2] = 15;		// backlight 1 - 15
	nd_lcd_send_packet(3);

	nc_lcd_tx_buf[1] = 0x48;	// underline cursor off
	nd_lcd_send_packet(2);
	
  //nd_lcd_set_cursor_position(0);
  nd_lcd_set_cursor(0);
  //nd_lcd_clear_screen();
  
  printString(0, " Mermaid's Rest", "-= mhelm v0.1 =-");
  //printString(" Mermaid's Rest");  
  //nd_lcd_set_cursor_position(0x40);
  //printString("-= mhelm v0.1 =-");  
	HAL_Delay(100);
}

// Set the cursor position
void nd_lcd_set_cursor(unsigned int mode) {
  if (mode == 1) {
    // Show cursor
	  nc_lcd_tx_buf[1] = 0x4B;
  	nd_lcd_send_packet(2);
  } else {
    // Hide cursor
	  nc_lcd_tx_buf[1] = 0x4C;
  	nd_lcd_send_packet(2);
  }
}

// Set the cursor position
void nd_lcd_set_cursor_position(uint8_t pos) {
  	  nc_lcd_tx_buf[1] = 0x45;
      nc_lcd_tx_buf[2] = pos;
  	  nd_lcd_send_packet(3);
}


// Clear the screen 
void nd_lcd_clear_screen(void) {
  nc_lcd_tx_buf[1] = 0x51;
  nd_lcd_send_packet(2);
}

// ??
//void nd_lcd_i2c_busy(unsigned int){}

// Clear one line on the screen
void nd_lcd_clear_line(unsigned int line) {}

void printString(uint8_t lcd, const char *line1, const char *line2) {
  uint16_t LCD_I2CADDR;
  // TODO: Make this an array so that we can use 'LCD_I2CADDR   = lcdAddress[lcd]'
  if (lcd == 0) {
    LCD_I2CADDR = LCD0_I2CADDR;
  } else if (lcd == 1) {
    LCD_I2CADDR = LCD1_I2CADDR;
  } else if (lcd == 2) {
    LCD_I2CADDR = LCD2_I2CADDR;
  } else if (lcd == 3) {
    LCD_I2CADDR = LCD3_I2CADDR;
  }

  // Clear and init
  nd_lcd_set_cursor_position(0);
  nd_lcd_clear_screen();
  
  // Line 1
  HAL_I2C_Master_Transmit(&hi2c_lcd, LCD_I2CADDR, line1, strlen(line1), 1000);    

  // Line 2
  nd_lcd_set_cursor_position(0x40);
  HAL_I2C_Master_Transmit(&hi2c_lcd, LCD_I2CADDR, line2, strlen(line2), 1000);    
}

// Local functions
void nd_lcd_send_packet(uint16_t size) {
  HAL_Delay(1);
  HAL_I2C_Master_Transmit(&hi2c_lcd, LCD1_I2CADDR, nc_lcd_tx_buf, size, 1000);
  HAL_Delay(1);
  
}

void nd_lcd_i2c_busy(unsigned int) {}
