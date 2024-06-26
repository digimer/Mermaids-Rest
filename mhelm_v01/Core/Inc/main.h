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
#include "lcd.h"
#include "motor.h"
#include "i2cHelpers.h"
#include "throttle.h"

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

// Throttle enable ("oh poo" switch)
#define THROTTLE_STOP_SW_IN_Pin GPIO_PIN_6
#define THROTTLE_STOP_SW_IN_GPIO_Port GPIOA

// Newhaven Display (LCD) definitions
#define LCD0_I2CADDR (0x50)  // New LCDs before changing their address
#define LCD1_I2CADDR (0x51)
#define LCD2_I2CADDR (0x52)
#define LCD3_I2CADDR (0x53)

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA

// I2C for all LCDs
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB

// Genereal switches
// Helm main switch
#define MAIN_SW_IN_Pin GPIO_PIN_4
#define MAIN_SW_IN_GPIO_Port GPIOA
// Speed select switches
#define SPEED_HIGH_SW_IN_Pin GPIO_PIN_7
#define SPEED_HIGH_SW_IN_GPIO_Port GPIOA
#define SPEED_LOW_SW_IN_Pin GPIO_PIN_8
#define SPEED_LOW_SW_IN_GPIO_Port GPIOA

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

// Others
#define MAIN_DELAY_TIME 250             // Time between loops

/* USER CODE BEGIN Private defines */
// main timer task
void main_timer_task(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
