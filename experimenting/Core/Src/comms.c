/**
  ******************************************************************************
  * @file           : comms.c
  * @brief          : Code for displaying data on the LCD screens, serial 
  *                   output, LED displays, etc.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 The Digital Mermaid.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
**/
#include <stdio.h>
#include <string.h>

#include "comms.h"
#include "main.h"

// This struct contains all the data that is required for the output
// If something of that is DEBUG only I'd mask it for the release version to
// safe some memory It's packed to keep its size small.
typedef struct {
  // Switches
  uint8_t mainSwitch;
  uint8_t throttleStopSwitch;
  uint8_t speed10kwHighSwitch;
  uint8_t speed10kwLowSwitch;
  uint8_t speed5kwHighSwitch;
  uint8_t speed5kwLowSwitch;
  uint8_t motorSelect;

  // Throttle
  uint8_t throttlePortSelect;
  uint8_t throttleStarboardSelect;

  uint16_t throttleValueRaw;
  uint16_t throttleValuePerMille;
  uint16_t throttleValuePerMilleSmooth;
  uint16_t throttleValueDac;

  // Regen
  uint16_t regenValueRaw;
  uint16_t regenValuePerMille;
  uint16_t regenValuePerMilleSmooth;
  uint16_t regenValueDac;

} StatusData_t;

static UART_HandleTypeDef huart2;
static StatusData_t status;

/**
 * @brief Get the helm switch positions.
 * @param None
 * @retval None
 */
void lcdSetGlobalSwitches(uint8_t main, uint8_t throttleStop, uint8_t speed10kwHigh, uint8_t speed10kwLow, uint8_t speed5kwHigh, uint8_t speed5kwLow) {
  status.mainSwitch          = main;
  status.throttleStopSwitch  = throttleStop;
  status.speed10kwHighSwitch = speed10kwHigh;
  status.speed10kwLowSwitch  = speed10kwLow;
  status.speed5kwHighSwitch  = speed5kwHigh;
  status.speed5kwLowSwitch   = speed5kwLow;
}

/**
 * @brief Prepares the data to display to various outputs; Serial, LCDs, LEDs,
 *        etc.
 * @param None
 * @retval None
 */
void commsPrint() {
  static uint8_t counter = 0;
  char buffer[256];
  
  // Test output
  sprintf(buffer, "%d - This is serial\n\r", counter);

  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000); // Sending in normal mode
  counter++;
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = 38400;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

void commsInit() {
  // Ground the red pin, set high the green and blue pins
  //HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LCD1_BL_GREEN_GPIO_Port, LCD1_BL_GREEN_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(LCD1_BL_BLUE_GPIO_Port, LCD1_BL_BLUE_Pin, GPIO_PIN_SET);
  //memset(&status, 0, sizeof(status));

  MX_USART2_UART_Init();

  //nd_lcd_init();
}
