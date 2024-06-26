#ifndef __LCD_H__
#define __LCD_H__

#include <inttypes.h>

#define LCD1_BL_RED_Pin GPIO_PIN_6
#define LCD1_BL_RED_GPIO_Port GPIOC
#define LCD1_BL_GREEN_Pin GPIO_PIN_7
#define LCD1_BL_GREEN_GPIO_Port GPIOC
#define LCD1_BL_BLUE_Pin GPIO_PIN_8
#define LCD1_BL_BLUE_GPIO_Port GPIOC

// LCD1 "previous" and "next" page select
#define LCD1_SELECT_PREV_Pin GPIO_PIN_1
#define LCD1_SELECT_PREV_GPIO_Port GPIOB
#define LCD1_SELECT_NEXT_Pin GPIO_PIN_2
#define LCD1_SELECT_NEXT_GPIO_Port GPIOB


/**
  * @brief  Stores the switch settings of the system for display.
  * @param  main: main switch (LCD display)
  * @param  throttleStop: throttle stop switch
  * @param speedHigh High speed switch for motor
  * @param speedLow Low speed switch for motor
  * @retval None
  */
void lcdSetGlobalSwitches(uint8_t main, uint8_t throttleStop, uint8_t speedHigh, uint8_t speedLow);

/**
  * @brief  Stores the motor selection switch for display.
  * @param  aSwitch: The value of the switch  0==5kw, 1==10kw
  * @retval None
  * @warning The sematics are inverted with respect to the original code
  */
void lcdSetMotorSwitch(uint8_t aSwitch);

/**
  * @brief  Stores the throttle selection switches for display.
  * @param  portSwitch: The value of the port switc (1==selected)
  * @param  stbdSwitch: The value of the starboard switch (1==selected)
  * @retval None
  */
void lcdSetThrottleSwitches(uint8_t portSwitch, uint8_t stbdSwitch);

/**
  * @brief  Stores the throttle raw value as read from the ADC for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetThrottleRaw(uint16_t value);

/**
  * @brief  Stores the calculated position (in per mille) for the throttle for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetThrottlePerMille(uint16_t value);

/**
  * @brief  Stores the smoothed per mille value for the throttle for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetThrottlePerMilleSmooth(uint16_t value);

/**
  * @brief  Stores the trottle value to be sent to the motor DAC for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetThrottleDac(uint16_t value);

/**
  * @brief  Stores the regen raw value as read from the ADC for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetRegenRaw(uint16_t value);

/**
  * @brief  Stores the calculated position (in per mille) for the regen for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetRegenPerMille(uint16_t value);

/**
  * @brief  Stores the smoothed position (in per mille) for the regen for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetRegenPerMilleSmooth(uint16_t value);

/**
  * @brief  Stores the trottle value to be sent to the motor regen DAC for display.
  * @param  value: The value to store
  * @retval None  
  */
void lcdSetRegenDac(uint16_t value);

/**
  * @brief  Initialize LCD page, buttons and storage.
  * @retval None  
*/
void lcdInit();

/**
  * @brief  Handles button presses to switch between LCD pages.
  * @retval None  
  * @note This function does nothing if main switch is off.
  */
void lcdHandlePageButtons();

/**
  * @brief  Prints the data for the currently active page
  * @retval None  
  * @note This function does nothing if main switch is off.
  */
void lcdPrint();

#endif
