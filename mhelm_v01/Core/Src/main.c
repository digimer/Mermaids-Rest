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
/* Includes ------------------------------------------------------------------*/
#include "ioctl.h"
#include "main.h"
#include "config.h"
//#include "nd_lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

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

I2C_HandleTypeDef hi2c_dac;  // rename to 'hi2c_dac' DAC
I2C_HandleTypeDef hi2c_lcd;  // rename to 'hi2c_lcd' LCD

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  //MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //uint8_t count = 0;
  uint8_t mainSwitchValue         = 0;
  uint8_t speedHighSwitchValue    = 0; // 0 = Off, 1 = Selected (inverted at GPIO read)
  uint8_t speedLowSwitchValue     = 0; // 0 = Off, 1 = Selected (inverted at GPIO read)
  uint8_t lcd1SelPrevSwitchValue  = 0; // 0 = Off, 1 = Selected (inverted at GPIO read)
  uint8_t lcd1SelNextSwitchValue  = 0; // 0 = Off, 1 = Selected (inverted at GPIO read)
  uint8_t throttleStopSwitchValue = 0; // 0 = Run, 1 = Stop
  uint8_t speedSelect             = 1; // 0 = Low, 1 = Med, 2 = High, 3 = E-Stop

  // ADC values
  uint16_t adcThrottleValue = 2047; // Mid-point is "neutral"
  uint16_t adcRegenValue    = 0;    // Disable regen

  // LCD1 Page Display
  uint8_t lcd1Pages    = 2; // Max page
  uint8_t lcd1ShowPage = 0; // Integer changes the data shown on LCD1 (for now, serial)
   
  //uint8_t Test[] = "Mermaid's Rest - mhelm v0.1\r\n"; //Data to send
  uint8_t Test[128];
  uint8_t Speed[16];
  uint8_t Pots[128];

  /* USER CODE END 2 */
  ioctl_init();

  // start timers
  HAL_TIM_Base_Start_IT(&htim1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Ground the red pin, set high the green and blue pins
  HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, 0);
  HAL_GPIO_WritePin(LCD1_BL_GREEN_GPIO_Port, LCD1_BL_GREEN_Pin, 1);
  HAL_GPIO_WritePin(LCD1_BL_BLUE_GPIO_Port, LCD1_BL_BLUE_Pin, 1);
  while (1)
  {
    // Read the switch states
    speedSelect             = 1; // Medium speed is default
    mainSwitchValue         = HAL_GPIO_ReadPin(MAIN_SW_IN_GPIO_Port, MAIN_SW_IN_Pin);
    speedHighSwitchValue    = 1 - HAL_GPIO_ReadPin(SPEED_HIGH_SW_IN_GPIO_Port, SPEED_HIGH_SW_IN_Pin);
    speedLowSwitchValue     = 1 - HAL_GPIO_ReadPin(SPEED_LOW_SW_IN_GPIO_Port, SPEED_LOW_SW_IN_Pin);
    lcd1SelPrevSwitchValue  = 1 - HAL_GPIO_ReadPin(LCD1_SELECT_PREV_GPIO_Port, LCD1_SELECT_PREV_Pin);
    lcd1SelNextSwitchValue  = 1 - HAL_GPIO_ReadPin(LCD1_SELECT_NEXT_GPIO_Port, LCD1_SELECT_NEXT_Pin);
    throttleStopSwitchValue = HAL_GPIO_ReadPin(THROTTLE_STOP_SW_IN_GPIO_Port, THROTTLE_STOP_SW_IN_Pin);

    if (mainSwitchValue == 0) {
      //HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, 0);
    } else {
      //HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, 1);
    }
    
    // Emergency Stop logic
    if (throttleStopSwitchValue == 1){
      // Shut everything down
      speedSelect         = 3;    // E-Stop
      speedLowSwitchValue = 1;    // Set slow speed
      adcThrottleValue    = 2048; // Set the throttle to neutral
      adcRegenValue       = 4095; // Set the regen to max
    } else {
      // Do normal logic

      // Read in the potentiometers
      adcThrottleValue = ioctl_get_pot(IOCTL_THROTTLE_POT); // get the adc value for the throttle 
      adcRegenValue    = ioctl_get_pot(IOCTL_REGEN_POT);    // get the adc value for the regen

      // What speed?
      if (speedLowSwitchValue == 1) {
        speedSelect = 0;
      } else if (speedHighSwitchValue) {
        speedSelect = 2;
      }
    }
    
    // Handle LCD paging
    if (lcd1SelPrevSwitchValue) {
      if (lcd1ShowPage < lcd1Pages) { 
        lcd1ShowPage ++;
      } else {
        lcd1ShowPage = 0;
      }
    } else if (lcd1SelNextSwitchValue) {
      if (lcd1ShowPage == 0) { 
        lcd1ShowPage = lcd1Pages;
      } else {
        lcd1ShowPage --;
      }
    }
    
    // Display data
    if (lcd1ShowPage == 0) {
      // Page 1 - Switch positions
      sprintf(Test, "Sw; Mn: [%d], Th: [%d], SH: [%d], SL: [%d], L1P: [%d], L1N: [%d]\n\r", mainSwitchValue, throttleStopSwitchValue, speedHighSwitchValue, speedLowSwitchValue, speedSelect, lcd1SelPrevSwitchValue, lcd1SelNextSwitchValue);
      HAL_UART_Transmit(&huart2,Test,strlen(Test),1000);  // Sending in normal mode
    } else if (lcd1ShowPage == 1) {
      // Page 2 - Speed 
      if (speedSelect == 2) {
        sprintf(Speed, "Turbo speed!\n\r");
      } else if (speedSelect == 0) {
        sprintf(Speed, "Turtle speed.\n\r");
      } else if (speedSelect == 0) {
        sprintf(Speed, "Emegergency Stop!\n\r");
      } else {
        sprintf(Speed, "Normal speed\n\r");
      }
      HAL_UART_Transmit(&huart2,Speed,strlen(Speed),1000);  // Sending in normal mode
    } else if (lcd1ShowPage == 2) {
      // Page 3 - ADC and DAC
      sprintf(Pots, "ADC; Throttle: [%d], Regen: [%d]\n\r", adcThrottleValue, adcRegenValue);
      HAL_UART_Transmit(&huart2,Pots,strlen(Pots),1000);  // Sending in normal mode
    }
    
    HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// main timer task
void main_timer_task(void) {
  HAL_GPIO_TogglePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin);
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
  sConfig.Channel = ADC_CHANNEL_0; // Throttle Pot
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1; // Regen Pot
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c_dac.Instance = I2C1;
  hi2c_dac.Init.Timing = 0x2010091A;
  hi2c_dac.Init.OwnAddress1 = 0;
  hi2c_dac.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c_dac.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c_dac.Init.OwnAddress2 = 0;
  hi2c_dac.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c_dac.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c_dac.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c_dac) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c_dac, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c_dac, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c_lcd.Instance = I2C2;
  hi2c_lcd.Init.Timing = 0x2010091A;
  hi2c_lcd.Init.OwnAddress1 = 0;
  hi2c_lcd.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c_lcd.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c_lcd.Init.OwnAddress2 = 0;
  hi2c_lcd.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c_lcd.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c_lcd.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c_lcd) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c_lcd, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c_lcd, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  #define LCD1_SELECT_PREV_Pin GPIO_PIN_1
  #define LCD1_SELECT_PREV_GPIO_Port GPIOB
  #define LCD1_SELECT_NEXT_Pin GPIO_PIN_2
  #define LCD1_SELECT_NEXT_GPIO_Port GPIOB
  #define LCD1_BL_RED_Pin GPIO_PIN_6
  #define LCD1_BL_RED_GPIO_Port GPIOC
  #define LCD1_BL_GREEN_Pin GPIO_PIN_7
  #define LCD1_BL_GREEN_GPIO_Port GPIOC
  #define LCD1_BL_BLUE_Pin GPIO_PIN_8
  #define LCD1_BL_BLUE_GPIO_Port GPIOC

  /* TODO: Remove - Testing PB6, PB7 and PB8 (LCD1 Backlight Pins) */
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

  /*Configure GPIO pins : DEBUG_0_Pin DEBUG_1_Pin */
  GPIO_InitStruct.Pin = DEBUG_0_Pin|DEBUG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC4 PC5
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
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
  GPIO_InitStruct.Pin = MAIN_SW_IN_Pin | THROTTLE_STOP_SW_IN_Pin|SPEED_HIGH_SW_IN_Pin|SPEED_LOW_SW_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD1_SELECT_PREV_Pin LCD1_SELECT_NEXT_Pin */
  GPIO_InitStruct.Pin = LCD1_SELECT_PREV_Pin|LCD1_SELECT_NEXT_Pin;
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
