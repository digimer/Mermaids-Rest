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

// I2C to both Throttle and Regen DAC
#define DAC_SCL_Pin GPIO_PIN_6
#define DAC_SCL_GPIO_Port GPIOB
#define DAC_SDA_Pin GPIO_PIN_7
#define DAC_SDA_GPIO_Port GPIOB

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
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
// Tell the controller to go to high speed
#define SPEED_HIGH_SW_OUT_Pin GPIO_PIN_10
#define SPEED_HIGH_SW_OUT_GPIO_Port GPIOA
// Tell the controller to go to low speed
#define SPEED_LOW_SW_OUT_Pin GPIO_PIN_11
#define SPEED_LOW_SW_OUT_GPIO_Port GPIOA

// 
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

// DACs
#define DAC_SCL_Pin GPIO_PIN_6
#define DAC_SCL_GPIO_Port GPIOB
#define DAC_SDA_Pin GPIO_PIN_7
#define DAC_SDA_GPIO_Port GPIOB

// Static Values 
// Throttle - Starboard
#define THROTTLE_STBD_DAC_MINIMUM         6        // The TPS Dead Low is set to 0.05vDC, 6  5.73vDC
#define THROTTLE_STBD_DAC_MAXIMUM         4090     // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC
#define THROTTLE_STBD_STEP_VOLTAGE        1.2207f  // float representing the voltage per step (5000/4096)
#define THROTTLE_STBD_AVERAGE_OVER        8        // Average the reading over this many reads. Must be proportional to delayTime
#define THROTTLE_STBD_REVERSE_MAX         2520     // The throttle pot value equalling the fastest speed in reverse
#define THROTTLE_STBD_REVERSE_MIN         2100     // The throttle pot value equalling the slowest speed in reverse
#define THROTTLE_STBD_FORWARD_MIN         1920     // The throttle pot value equalling the slowest speed forward
#define THROTTLE_STBD_FORWARD_MAX         1450     // The throttle pot value equalling the fastest speed forward
#define REVERSE_STBD_SENSOR_STEPS         (THROTTLE_STBD_REVERSE_MIN - THROTTLE_STBD_REVERSE_MAX);
#define FORWARD_STBD_SENSOR_STEPS         (THROTTLE_STBD_FORWARD_MAX - THROTTLE_STBD_FORWARD_MIN);
#define NEUTRAL_STBD_MID_POINT            (((THROTTLE_STBD_FORWARD_MIN - THROTTLE_STBD_REVERSE_MIN) / 2) + THROTTLE_STBD_REVERSE_MIN);
#define REVERSE_STBD_PERCENT_STEPS        100 / REVERSE_STBD_SENSOR_STEPS;
#define FORWARD_STBD_PERCENT_STEPS        100 / FORWARD_STBD_SENSOR_STEPS;
#define THROTTLE_STBD_SMOOTHED_POT_VALUE  NEUTRAL_STBD_MID_POINT;
// Throttle - Port
#define THROTTLE_PORT_DAC_MINIMUM         6        // The TPS Dead Low is set to 0.05vDC, 6  5.73vDC
#define THROTTLE_PORT_DAC_MAXIMUM         4090     // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC
#define THROTTLE_PORT_STEP_VOLTAGE        1.2207f  // float representing the voltage per step (5000/4096)
#define THROTTLE_PORT_AVERAGE_OVER        8        // Average the reading over this many reads. Must be proportional to delayTime
#define THROTTLE_PORT_REVERSE_MAX         2520     // The throttle pot value equalling the fastest speed in reverse
#define THROTTLE_PORT_REVERSE_MIN         2100     // The throttle pot value equalling the slowest speed in reverse
#define THROTTLE_PORT_FORWARD_MIN         1920     // The throttle pot value equalling the slowest speed forward
#define THROTTLE_PORT_FORWARD_MAX         1450     // The throttle pot value equalling the fastest speed forward
#define REVERSE_PORT_SENSOR_STEPS         (THROTTLE_PORT_REVERSE_MIN - THROTTLE_PORT_REVERSE_MAX);
#define FORWARD_PORT_SENSOR_STEPS         (THROTTLE_PORT_FORWARD_MAX - THROTTLE_PORT_FORWARD_MIN);
#define NEUTRAL_PORT_MID_POINT            (((THROTTLE_PORT_FORWARD_MIN - THROTTLE_PORT_REVERSE_MIN) / 2) + THROTTLE_PORT_REVERSE_MIN);
#define REVERSE_PORT_PERCENT_STEPS        100 / REVERSE_PORT_SENSOR_STEPS;
#define FORWARD_PORT_PERCENT_STEPS        100 / FORWARD_PORT_SENSOR_STEPS;
#define THROTTLE_PORT_SMOOTHED_POT_VALUE  NEUTRAL_PORT_MID_POINT;


// Regen 
#define REGEN_DAC_MINIMUM   20           // The TPS Dead Low is set to 0.05vDC, 6  5.73vDC
#define REGEN_DAC_MAXIMUM   4090        // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC
#define REGEN_STEP_VOLTAGE  1.2207f     // float representing the voltage per step (5000/4096)
#define REGEN_AVERAGE_OVER  8           // Average the reading over this many reads. Must be proportional to delayTime
#define REGEN_POT_MINIMUM   0
#define REGEN_POT_MAXIMUM   4030        // By math, this should be 4095, but is less in practice and needs to be tested per pot
// Others
#define MAIN_DELAY_TIME 250                  // Time between loops

// Calibrated Values - These need to be per-sensor, and ideally read from a user-editable field
// Starboard
#define THROTTLE_STBD_DAC_NEUTRAL       2010   // Tested on this DAC, not mathmatically accurate
#define THROTTLE_STBD_DAC_NEUTRAL_START 1965
#define THROTTLE_STBD_DAC_NEUTRAL_END   2130
// Port
#define THROTTLE_PORT_DAC_NEUTRAL       2010   // Tested on this DAC, not mathmatically accurate
#define THROTTLE_PORT_DAC_NEUTRAL_START 1965
#define THROTTLE_PORT_DAC_NEUTRAL_END   2130

/* USER CODE BEGIN Private defines */
// main timer task
void main_timer_task(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
