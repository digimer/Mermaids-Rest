
/*****************************************************************************
* Copyright (c) 2018, Newhaven Display International
*
* This code is provided as an example only and is not guaranteed by Newhaven Display.
* Newhaven Display accepts no responsibility for any issues resulting from its use.
* The developer of the final application incorporating any parts of this
* sample code is responsible for ensuring its safe and correct operation
* and for any consequences resulting from its use.
*****************************************************************************/
// *                      August 15th, 2007
// * Definitions for serial LCD tester program
// * CCS Compiler
// *
// * Last Update : April 1st, 2024
// * Updated for use by mhelm_v01
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

char const text6[] = {"_Digital Mermaid_"};
char const text7[] = {"-] mhelm v0.1. [-"};
//char const text6[] = {"NewHaven Display"};
//char const text7[] = {"Serial LCD Demo."};

/* DEFINE VARIABLES */
/*
unsigned int tick_100m;     // 100 millisecond counter
unsigned int tick_1s;     // 1 second counter
unsigned int tick_wait10;   // 10 millisecond delay counter
unsigned int tick_wait100;    // 100 millisecond delay counter
unsigned int temp;
unsigned int tx_packet[20];   // transmit data packet
unsigned int com_mode;
unsigned int i, j;
unsigned int loop_hi, loop_lo;
unsigned int adc_result;
unsigned int contrast;
unsigned int contrast_save;
unsigned int bright;
unsigned int bright_save;
*/

/* prototype functions */
void nd_lcd_init(void);
void nd_lcd_set_cursor(unsigned int mode);
void nd_lcd_clear_screen(void);
void nd_lcd_clear_line(unsigned int line);

