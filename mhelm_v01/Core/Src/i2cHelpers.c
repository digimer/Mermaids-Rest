#include "stm32f0xx_hal.h"

#include "i2cHelpers.h"

/** This file contains functions to simplify the usage of
 * the I2c connections to the DACs and the LCD
 * This is mainly init and sending data
*/



// Imported from main.c
void Error_Handler(void);


#define MCP4725_COMMON_I2CADDR     ((0x62) << 1) // This is set before the main loop by asserting all A0 pins to 1
#define MCP4725_ACTIVE_I2CADDR     ((0x63) << 1) // This is set by asserting 0 to the target DAC's A0 pin prior to write

// These will also be used by the IRQ handler
I2C_HandleTypeDef hi2c_dac;  // I2C for the DACs
I2C_HandleTypeDef hi2c_lcd;  // I2C for the LCDs


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
  hi2c_lcd.Init.Timing = 0x20303EFD;
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

void I2cInit() {
    MX_I2C2_Init();
    MX_I2C1_Init();
}

/**
 * This function writes a command to the DAC specified by (port,pin)
 * We expect 3 bytes in pData in the format [COMMAND,PARAM1,PARAM2]
*/
HAL_StatusTypeDef I2cSendtoDAC(const GPIO_TypeDef *port, const uint16_t pin, uint8_t *pData, const uint32_t timeout) {
  HAL_StatusTypeDef result = HAL_OK;

  // Set Address of target DAC to 62
  I2C_ACTIVATE_DAC(port, pin);
  /* Don't think this is required */
  /*HAL_Delay(10); */

  result = HAL_I2C_Master_Transmit(&hi2c_dac, MCP4725_ACTIVE_I2CADDR, pData, 3, timeout);

  /* Transmit is defined as blocking,
     so we can assume that the sending process is done on return from the call
     and there is no reason to put a delay here
  */
  /*HAL_Delay(10);*/

  // Reset Address to the inactive one
  I2C_DEACTIVATE_DAC(port, pin);

  return result;
}