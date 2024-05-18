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

void lcdSetGlobalSwitches(uint8_t main, uint8_t throttleStop, uint8_t speedHigh, uint8_t speedLow);
void lcdSetMotorSwitch(uint8_t aSwitch);

void lcdSetThrottleSwitches(uint8_t portSwitch, uint8_t stdbSwitch);
void lcdSetThrottleRaw(uint16_t value);
void lcdSetThrottleSmooth(uint16_t value);
void lcdSetThrottlePerMille(uint16_t value);
void lcdSetThrottleDac(uint16_t value);


void lcdSetRegenRaw(uint16_t value);
void lcdSetRegenSmooth(uint16_t value);
void lcdSetRegenPerMille(uint16_t value);
void lcdSetRegenDac(uint16_t value);

void lcdInit();
void lcdHandlePageButtons();
void lcdPrint();

#endif
