/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// Define pins
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define THROTTLE_STBD_POT_Pin GPIO_PIN_0
#define THROTTLE_STBD_POT_GPIO_Port GPIOA
#define REGEN_POT_Pin GPIO_PIN_1
#define REGEN_POT_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define M5KW_BRAKE_OUT_Pin GPIO_PIN_4
#define M5KW_BRAKE_OUT_GPIO_Port GPIOF
#define M5KW_SPEED_LOW_OUT_Pin GPIO_PIN_5
#define M5KW_SPEED_LOW_OUT_GPIO_Port GPIOF
#define THROTTLE_PORT_POT_Pin GPIO_PIN_4
#define THROTTLE_PORT_POT_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LCD_BL_RED_Pin GPIO_PIN_6
#define LCD_BL_RED_GPIO_Port GPIOA
#define LCD_BL_GREEN_Pin GPIO_PIN_7
#define LCD_BL_GREEN_GPIO_Port GPIOA
#define M5KW_SPEED_HIGH_OUT_Pin GPIO_PIN_4
#define M5KW_SPEED_HIGH_OUT_GPIO_Port GPIOC
#define LCD_BL_BLUE_Pin GPIO_PIN_0
#define LCD_BL_BLUE_GPIO_Port GPIOB
#define PANEL_CS_Pin GPIO_PIN_10
#define PANEL_CS_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
#define M10KW_BRAKE_OUT_Pin GPIO_PIN_13
#define M10KW_BRAKE_OUT_GPIO_Port GPIOB
#define M10KW_SPEED_LOW_OUT_Pin GPIO_PIN_14
#define M10KW_SPEED_LOW_OUT_GPIO_Port GPIOB
#define M10KW_SPEED_HIGH_OUT_Pin GPIO_PIN_15
#define M10KW_SPEED_HIGH_OUT_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_6
#define LCD_SCL_GPIO_Port GPIOF
#define ESTOP_SW_IN_Pin GPIO_PIN_7
#define ESTOP_SW_IN_GPIO_Port GPIOF
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BACKUP_KEY_SW_IN_Pin GPIO_PIN_15
#define BACKUP_KEY_SW_IN_GPIO_Port GPIOA
#define M5KW_REGEN_DAC_A0_Pin GPIO_PIN_10
#define M5KW_REGEN_DAC_A0_GPIO_Port GPIOC
#define M10KW_REGEN_DAC_A0_Pin GPIO_PIN_11
#define M10KW_REGEN_DAC_A0_GPIO_Port GPIOC
#define M5KW_THROTTLE_DAC_A0_Pin GPIO_PIN_12
#define M5KW_THROTTLE_DAC_A0_GPIO_Port GPIOC
#define M10KW_THROTTLE_DAC_A0_Pin GPIO_PIN_2
#define M10KW_THROTTLE_DAC_A0_GPIO_Port GPIOD
#define DAC_SDA_Pin GPIO_PIN_7
#define DAC_SDA_GPIO_Port GPIOB
#define DAC_SCL_Pin GPIO_PIN_8
#define DAC_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// main timer task
void main_timer_task(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
