#ifndef __I2CHELPERS_H__
#define __I2CHELPERS_H__

#include "stm32f0xx_hal.h"

// MCP4725 definitions
#define MCP4725_SET_ADDRESS_ACTIVE   GPIO_PIN_SET
#define MCP4725_SET_ADDRESS_COMMON   GPIO_PIN_RESET

#define I2C_ACTIVATE_DAC(PORT,PIN)  HAL_GPIO_WritePin((GPIO_TypeDef*)PORT, PIN, MCP4725_SET_ADDRESS_ACTIVE)
#define I2C_DEACTIVATE_DAC(PORT,PIN) HAL_GPIO_WritePin((GPIO_TypeDef*)PORT, PIN, MCP4725_SET_ADDRESS_COMMON)

#define MCP4725_CMD_WRITEDAC       (0x40) // Writes data to the DAC
#define MCP4725_CMD_WRITEDACEEPROM (0x60) // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)


// Helpers for calculating the command parameter
#define HIPAR(VAL) ((VAL) >>4)
#define LOPAR(VAL) (((VAL) &0x0F) << 4)


/// @brief Initializes the I2C communication.
/// Sets up I2C1 and I2C2 
void I2cInit();

/// @brief This function writes a command to the DAC specified by (port,pin).
/// @param port The poort A0 of the DAC is connected to.
/// @param pin The poort A0 of the DAC is connected to.
/// @param pData Data to be sent. Has to be of size 3 bytes in the format [command,HIBYTE,LOWBYTE]
/// @param timeout Timeout for the send function
/// @return \c HAL_OK on success or an error describing the problem
HAL_StatusTypeDef I2cSendtoDAC(const GPIO_TypeDef *port, const uint16_t pin, uint8_t *pData, const uint32_t timeout);

#endif