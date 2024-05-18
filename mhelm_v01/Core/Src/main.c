/* USER CODE BEGIN Header */
/**
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

#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "ioctl.h"
#include "main.h"
#include "config.h"
#include "i2cHelpers.h"
#include "throttle.h"
#include "motor.h"
#include "lcd.h"

//#include "nd_lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>

//#include "Wire.h"
//#include "math.h"
//#include "Adafruit_MCP4725.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Remove this define in order to stop outputting to serial

// Set this value to 9, 8, 7, 6 or 5 to adjust the resolution

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;



/* USER CODE BEGIN PV */
int map(int, int, int, int, int); // Like arduino's map() function
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// Forward declaration of init functionCore/Src/nd_lcd.c

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  // TODO: Uncomment these out
  MX_ADC_Init();
  
  // Initialize all I2C stuff
  I2cInit();
  
  //MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Make a call to write the throttle DAC's EEPROM to their ~2.5v position and the regen DACs to 0.05v

  //uint8_t count = 0;
  uint8_t mainSwitchValue         = 0; // This turns the helm displays on (LEDs, etc, does NOT disable throttle controls)
  uint8_t speedHighSwitchValue    = 0; // 0 = Off, 1 = Selected (inverted at GPIO read)
  uint8_t speedLowSwitchValue     = 0; // 0 = Off, 1 = Selected (inverted at GPIO read)
  uint8_t throttleStopSwitchValue = 0; // 0 = Run, 1 = Stop - This is the E-Stop
  
    
  uint16_t throttlePerMilleValue = 500;
  uint16_t regenPerMilleValue = 500;

 
  /* USER CODE END 2 */
  ioctl_init();

  // start timers
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE BEGIN WHILE */
  

  // Set up the throttle data
  initThrottle();

  // Set initial values for the motors
  initMotor();  
  
  // Initialize lcd
  lcdInit();

  /* Infinite loop */
  while (1)
  {
    // Read the switch states (1 - X inverts the read value so that '1' is 'On')
    
      mainSwitchValue = 1 - HAL_GPIO_ReadPin(MAIN_SW_IN_GPIO_Port, MAIN_SW_IN_Pin);

      // Read Throttle emergency off
      throttleStopSwitchValue = 1 - HAL_GPIO_ReadPin(THROTTLE_STOP_SW_IN_GPIO_Port, THROTTLE_STOP_SW_IN_Pin);
     
       // Calculate throttle values
      throttlePerMilleValue = handleThrottle(throttleStopSwitchValue);
     
      // Calculate regen values
      regenPerMilleValue = handleRegen(throttleStopSwitchValue);

     if(throttleStopSwitchValue == 1) {
      // Shut everything down! Press the foot switch, change the speed to Low, put the throttle to neutral
      // and Max regen
      speedHighSwitchValue = 0;
      speedLowSwitchValue  = 1;
     } else {
      // Do normal logic
      // Read in the speed switches
      speedHighSwitchValue   = 1 - HAL_GPIO_ReadPin(SPEED_HIGH_SW_IN_GPIO_Port, SPEED_HIGH_SW_IN_Pin);
      speedLowSwitchValue    = 1 - HAL_GPIO_ReadPin(SPEED_LOW_SW_IN_GPIO_Port, SPEED_LOW_SW_IN_Pin);      
    }
    
    // Change output switch values 
    HAL_GPIO_WritePin(FOOT_SW_OUT_GPIO_Port, FOOT_SW_OUT_Pin, throttleStopSwitchValue);
    HAL_GPIO_WritePin(SPEED_HIGH_SW_OUT_GPIO_Port, SPEED_HIGH_SW_OUT_Pin, speedHighSwitchValue);
    HAL_GPIO_WritePin(SPEED_LOW_SW_OUT_GPIO_Port, SPEED_LOW_SW_OUT_Pin, speedLowSwitchValue);

    // And send the stuff to the motors
    // Does it make sense to send this even if the foot switch is off?
    handleMotor(throttlePerMilleValue,regenPerMilleValue);
    
    lcdSetGlobalSwitches(mainSwitchValue, throttleStopSwitchValue, speedHighSwitchValue, speedLowSwitchValue);
    lcdHandlePageButtons();
    lcdPrint();
    
    HAL_Delay(MAIN_DELAY_TIME);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   
  }
  /* USER CODE END 3 */
}

// main timer task
void main_timer_task(void) {
  //HAL_GPIO_TogglePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin);
}

/*
void init_LCD()
{
  I2C_Start(); 
  I2C_out(0x7C); 
  I2C_out(0x00); 
  I2C_out(0x38); 
  delay(10); 
  I2C_out(0x39); 
  delay(10); 
  I2C_out(0x14); 
  I2C_out(0x78); 
  I2C_out(0x5E); 
  I2C_out(0x6D); 
  I2C_out(0x0C); 
  I2C_out(0x01); 
  I2C_out(0x06); 
  delay(10); 
  I2C_stop(); 
}
*/

/** @brief similar to Arduino's map() function
 * This take a value, and two ranges; fromLow, fromHigh and toLow, toHigh. The values relative position 
 * between the 'from' values is used to find the corresponding relative function in the 'to' range.
 * @param value: The value being mapped
 * @param fromLow: The lowest integer that 'value' could be
 * @param fromHigh: The highest integer that 'value' could be
 * @param toLow: The lowest integer that 'value' can map to
 * @param toHigh: The highest integer that 'value' can map to
 * @retval int
*/
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    // Calculate the percentage of 'value' within the original range
    int percentage = ((value - fromLow) * 100) / (fromHigh - fromLow);
 
    // Map the percentage to the new range
    int result = toLow + (percentage * (toHigh - toLow) / 100);
 
    return result;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0; // Throttle Starboard Pot, channel number from reference manual for this pin
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1; // Regen Pot, , channel number from reference manual for this pin
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channe 13 to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13; // Throttle Port Pot, channel number from reference manual for this pin
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}



/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}



/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, INT_PRIO_ADC, 0); // For the ADC 
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_0_Pin|DEBUG_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|FOOT_SW_OUT_Pin|SPEED_HIGH_SW_OUT_Pin|SPEED_LOW_SW_OUT_Pin, GPIO_PIN_RESET);

  /* TODO: Remove - Testing PC6, PC7 and PC8 (LCD1 Backlight Pins) */
  GPIO_InitStruct.Pin   = LCD1_BL_RED_Pin | LCD1_BL_GREEN_Pin | LCD1_BL_BLUE_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_0_Pin DEBUG_1_Pin M10KW_THROTTLE_DAC_A0_Pin M10KW_REGEN_DAC_A0_Pin*/
  GPIO_InitStruct.Pin = DEBUG_0_Pin|DEBUG_1_Pin|M10KW_THROTTLE_DAC_A0_Pin|M10KW_REGEN_DAC_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC4
                           PC5 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin FOOT_SW_OUT_Pin SPEED_HIGH_SW_OUT_Pin SPEED_LOW_SW_OUT_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|FOOT_SW_OUT_Pin|SPEED_HIGH_SW_OUT_Pin|SPEED_LOW_SW_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MAIN_SW_IN_Pin | THROTTLE_STOP_SW_IN_Pin SPEED_HIGH_SW_IN_Pin SPEED_LOW_SW_IN_Pin */
  GPIO_InitStruct.Pin = MAIN_SW_IN_Pin|THROTTLE_STOP_SW_IN_Pin|SPEED_HIGH_SW_IN_Pin|SPEED_LOW_SW_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M5KW_THROTTLE_DAC_A0_Pin M5KW_REGEN_DAC_A0_Pin */
  GPIO_InitStruct.Pin = M5KW_THROTTLE_DAC_A0_Pin|M5KW_REGEN_DAC_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB4 PB12 PB15 PB3*/
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_3;;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Enable the LCD1 Prev/Next and throttle select switch pins */
  GPIO_InitStruct.Pin = LCD1_SELECT_PREV_Pin|LCD1_SELECT_NEXT_Pin|MOTOR_5KW_SELECT_Pin
                        |THROTTLE_PORT_SELECT_Pin|THROTTLE_STBD_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
      main_timer_task();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}




/* USER CODE BEGIN 4 */
	
/********************************************************************
 set lcd cursor
********************************************************************/
void lcd_cursor(unsigned int x)
{
	//tx_packet[0] = 0xFE;
	//tx_packet[1] = 0x45;
	//tx_packet[2] = x;
	//send_packet(3);	
	//delay_ms(10);
}

/********************************************************************
 clear one line of display
********************************************************************/
void clear_line(unsigned int x)
{
	unsigned int ij;
	
	for (ij = 0; ij < x; ij++) {
		//tx_packet[ij] = 0x20;
	}
	//send_packet(x);
}

/********************************************************************
 lcd clear
********************************************************************/
void lcd_clear(void)
{
	//tx_packet[0] = 0xFE;
	//tx_packet[1] = 0x51;
	//send_packet(2);
	//delay_ms(10);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#endif /* Include guard */