/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/*
    GPIO:
    - GPIO  - port + pin

    Peripheral Init:
    - 


*/

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// Reset pin built into the Nucleo-F030R8
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

// Generic debug pins
#define DEBUG_0_Pin GPIO_PIN_0
#define DEBUG_0_GPIO_Port GPIOC
#define DEBUG_1_Pin GPIO_PIN_1
#define DEBUG_1_GPIO_Port GPIOC

// Motor select switch
#define MOTOR_5KW_SELECT_Pin GPIO_PIN_5
#define MOTOR_5KW_SELECT_GPIO_Port GPIOB

// Starboard Throttle pot
#define THROTTLE_STBD_POT_Pin GPIO_PIN_0
#define THROTTLE_STBD_POT_GPIO_Port GPIOA

// Port Throttle pot
#define THROTTLE_PORT_POT_Pin GPIO_PIN_3
#define THROTTLE_PORT_POT_GPIO_Port GPIOC

// Throttle Select
#define THROTTLE_PORT_SELECT_Pin GPIO_PIN_14
#define THROTTLE_PORT_SELECT_GPIO_Port GPIOB
#define THROTTLE_STBD_SELECT_Pin GPIO_PIN_13
#define THROTTLE_STBD_SELECT_GPIO_Port GPIOB

// Regen pot
#define REGEN_POT_Pin GPIO_PIN_1
#define REGEN_POT_GPIO_Port GPIOA

// I2C to the LCDs
#define I2C_LCD_SCL_Pin GPIO_PIN_10
#define I2C_LCD_SCL_GPIO_Port GPIOB
#define I2C_LCD_SDA_Pin GPIO_PIN_11
#define I2C_LCD_SDA_GPIO_Port GPIOB

// I2C to DACs
#define I2C_DAC_SCL_Pin GPIO_PIN_6
#define I2C_DAC_SCL_GPIO_Port GPIOB
#define I2C_DAC_SDA_Pin GPIO_PIN_7
#define I2C_DAC_SDA_GPIO_Port GPIOB

// DAC Address Pins
#define M10KW_THROTTLE_DAC_A0_Pin GPIO_PIN_10
#define M10KW_THROTTLE_DAC_A0_GPIO_Port GPIOC
#define M10KW_REGEN_DAC_A0_Pin GPIO_PIN_12
#define M10KW_REGEN_DAC_A0_GPIO_Port GPIOC
#define M5KW_THROTTLE_DAC_A0_Pin GPIO_PIN_8
#define M5KW_THROTTLE_DAC_A0_GPIO_Port GPIOB
#define M5KW_REGEN_DAC_A0_Pin GPIO_PIN_9
#define M5KW_REGEN_DAC_A0_GPIO_Port GPIOB

// MCP4725 definitions
#define MCP4725_SET_ADDRESS_62     1
#define MCP4725_SET_ADDRESS_63     0
#define MCP4725_COMMON_I2CADDR     (0x62) << 1 // This is set before the main loop by asserting all A0 pins to 1
#define MCP4725_ACTIVE_I2CADDR     (0x63) << 1 // This is set by asserting 0 to the target DAC's A0 pin prior to write
#define MCP4725_CMD_WRITEDAC       (0x40) // Writes data to the DAC
#define MCP4725_CMD_WRITEDACEEPROM (0x60) // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)

// Newhaven Display (LCD) definitions
#define LCD1_I2CADDR (0x50) << 1 // The default I2C address is 80 (50 hex) when counting the R/W bit, and 40 (28 hex) if not.

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LCD1_BL_RED_Pin GPIO_PIN_6
#define LCD1_BL_RED_GPIO_Port GPIOC
#define LCD1_BL_GREEN_Pin GPIO_PIN_7
#define LCD1_BL_GREEN_GPIO_Port GPIOC
#define LCD1_BL_BLUE_Pin GPIO_PIN_8
#define LCD1_BL_BLUE_GPIO_Port GPIOC

// I2C for all LCDs
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB

// Genereal switches
// Helm main switch
#define MAIN_SW_IN_Pin GPIO_PIN_4
#define MAIN_SW_IN_GPIO_Port GPIOA
// Throttle enable ("oh poo" switch)
#define THROTTLE_STOP_SW_IN_Pin GPIO_PIN_6
#define THROTTLE_STOP_SW_IN_GPIO_Port GPIOA
// Speed select switches
#define SPEED_HIGH_SW_IN_Pin GPIO_PIN_7
#define SPEED_HIGH_SW_IN_GPIO_Port GPIOA
#define SPEED_LOW_SW_IN_Pin GPIO_PIN_8
#define SPEED_LOW_SW_IN_GPIO_Port GPIOA
// LCD1 "previous" and "next" page select
#define LCD1_SELECT_PREV_Pin GPIO_PIN_1
#define LCD1_SELECT_PREV_GPIO_Port GPIOB
#define LCD1_SELECT_NEXT_Pin GPIO_PIN_2
#define LCD1_SELECT_NEXT_GPIO_Port GPIOB

// Output signals (signals to ULN2003A)
// Enable/disable the controller
#define FOOT_SW_OUT_Pin GPIO_PIN_9
#define FOOT_SW_OUT_GPIO_Port GPIOA
// Tell the controller to go to high, Low (or medium for neither) speed
#define SPEED_HIGH_SW_OUT_Pin GPIO_PIN_10
#define SPEED_HIGH_SW_OUT_GPIO_Port GPIOA
#define SPEED_LOW_SW_OUT_Pin GPIO_PIN_11
#define SPEED_LOW_SW_OUT_GPIO_Port GPIOA

// 
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

// Static Values 
// Calibrated Values - These need to be per-sensor, and ideally read from a user-editable field
// DAC values (common regardless of the throttle in use, but needs to be tested/adjusted per DAC)
// 10kw motor DACs
#define THROTTLE_DAC_M10KW_FORWARD_SLOWEST 2130 // Slowest forward, 1965 on this DAC is (2.4?)v
#define THROTTLE_DAC_M10KW_FORWARD_FASTEST 3975 // Fastest forward, TPS Dead Low is set to 0.05vDC, 6 on this DAC is 5.73mvDC
#define THROTTLE_DAC_M10KW_NEUTRAL         2020 // The value that sets the motor to neutral, 2.5v
#define THROTTLE_DAC_M10KW_REVERSE_SLOWEST 1965 // Slowest reverse, 2130 on this DAC is (2.6?)v
#define THROTTLE_DAC_M10KW_REVERSE_FASTEST 75   // Fastest reverse, TPS Dead High is set to 4.95vDC, 4090 on this DAC is 4993.4mvDC
// 5kw motor DACs
#define THROTTLE_DAC_M5KW_FORWARD_SLOWEST 2130 // Slowest forward, 1965 on this DAC is (2.4?)v
#define THROTTLE_DAC_M5KW_FORWARD_FASTEST 3950 // Fastest forward, TPS Dead Low is set to 0.05vDC, 6 on this DAC is 5.73mvDC
#define THROTTLE_DAC_M5KW_NEUTRAL         2010 // The value that sets the motor to neutral, 2.5v
#define THROTTLE_DAC_M5KW_REVERSE_SLOWEST 1965 // Slowest reverse, 2130 on this DAC is (2.6?)v
#define THROTTLE_DAC_M5KW_REVERSE_FASTEST 75   // Fastest reverse, TPS Dead High is set to 4.95vDC, 4090 on this DAC is 4993.4mvDC

// Regen 
#define REGEN_DAC_M10KW_MINIMUM 45          // The TPS Dead Low is set to 0.05vDC, 6  5.73 mvDC
#define REGEN_DAC_M10KW_MAXIMUM 3975        // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC
#define REGEN_DAC_M5KW_MINIMUM  45          // The TPS Dead Low is set to 0.05vDC, 6  5.73 mvDC
#define REGEN_DAC_M5KW_MAXIMUM  3950        // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC
#define REGEN_AVERAGE_OVER      8           // Average the reading over this many reads. Must be proportional to delayTime - Max 255
#define REGEN_POT_MINIMUM       0
#define REGEN_POT_MAXIMUM       4030        // By math, this should be 4095, but is less in practice and needs to be tested per pot
// Others
#define MAIN_DELAY_TIME 250             // Time between loops

// Throttle - Starboard
#define THROTTLE_STBD_AVERAGE_OVER 8        // Average the reading over this many reads. Must be proportional to delayTime - Max 255
#define THROTTLE_STBD_REVERSE_MAX  1140     // The throttle pot value equalling the fastest speed in reverse
#define THROTTLE_STBD_REVERSE_MIN  1850     // The throttle pot value equalling the slowest speed in reverse
#define THROTTLE_STBD_FORWARD_MIN  2040     // The throttle pot value equalling the slowest speed forward
#define THROTTLE_STBD_FORWARD_MAX  2740     // The throttle pot value equalling the fastest speed forward
#define REVERSE_STBD_SENSOR_STEPS  (THROTTLE_STBD_REVERSE_MIN - THROTTLE_STBD_REVERSE_MAX)
#define FORWARD_STBD_SENSOR_STEPS  (THROTTLE_STBD_FORWARD_MAX - THROTTLE_STBD_FORWARD_MIN)
#define NEUTRAL_STBD_MID_POINT     (((THROTTLE_STBD_FORWARD_MIN - THROTTLE_STBD_REVERSE_MIN) / 2) + THROTTLE_STBD_REVERSE_MIN)
#define REVERSE_STBD_PERCENT_STEPS 100 / REVERSE_STBD_SENSOR_STEPS
#define FORWARD_STBD_PERCENT_STEPS 100 / FORWARD_STBD_SENSOR_STEPS

// Throttle - Port
#define THROTTLE_PORT_AVERAGE_OVER 8        // Average the reading over this many reads. Must be proportional to delayTime - Max 255
#define THROTTLE_PORT_REVERSE_MAX  1450     // The throttle pot value equalling the fastest speed in reverse
#define THROTTLE_PORT_REVERSE_MIN  1920     // The throttle pot value equalling the slowest speed in reverse
#define THROTTLE_PORT_FORWARD_MIN  2100     // The throttle pot value equalling the slowest speed forward
#define THROTTLE_PORT_FORWARD_MAX  2520     // The throttle pot value equalling the fastest speed forward
#define REVERSE_PORT_SENSOR_STEPS  (THROTTLE_PORT_REVERSE_MIN - THROTTLE_PORT_REVERSE_MAX)
#define FORWARD_PORT_SENSOR_STEPS  (THROTTLE_PORT_FORWARD_MAX - THROTTLE_PORT_FORWARD_MIN)
#define NEUTRAL_PORT_MID_POINT     (((THROTTLE_PORT_FORWARD_MIN - THROTTLE_PORT_REVERSE_MIN) / 2) + THROTTLE_PORT_REVERSE_MIN)
#define REVERSE_PORT_PERCENT_STEPS 100 / REVERSE_PORT_SENSOR_STEPS
#define FORWARD_PORT_PERCENT_STEPS 100 / FORWARD_PORT_SENSOR_STEPS

/* USER CODE BEGIN Private defines */
// main timer task
void main_timer_task(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
