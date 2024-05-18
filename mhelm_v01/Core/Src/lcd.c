#include <stdio.h>
#include <string.h>

#include "main.h"
#include "lcd.h"

// This struct contains all the data that is required for the output
// If something of that is DEBUG only I'd mask it fopr the release version to safe some memory
// It's packed to keep its zize small.
#pragma pack (push,1)
typedef struct {
    // Switches
    int mainSwitch:1;
    int throttleStopSwitch:1;
    int speedHighSwitch:1;
    int speedLowSwitch:1;
    int motorSelect:1;

    // Throttle
    int throttlePortSelect:1;
    int throttleStarboardSelect:1;

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

#pragma pack (pop)

static UART_HandleTypeDef huart2;
static StatusData_t status;

 // LCD1 Page Display
    /* Pages
    0. Switch Positions
    1. Motor speed (Turltle, Normal, or Turbo)
    2. Throttle Pot and DAC Values - Numeric
    3. Regen Pot and DAC Values - Numeric
    4. Throttle and Regen, Percentages
    */
  static const uint8_t lcd1Pages    = 4; // Max page
  uint8_t lcd1ShowPage = 3; // Integer changes the data shown on LCD1 (for now, serial)

void lcdSetGlobalSwitches(uint8_t main, uint8_t throttleStop, uint8_t speedHigh, uint8_t speedLow) {
    status.mainSwitch = main;
    status.throttleStopSwitch = throttleStop;
    status.speedHighSwitch = speedHigh;
    status.speedLowSwitch = speedLow;
}

void lcdSetMotorSwitch(uint8_t aSwitch) {
    status.motorSelect = aSwitch;
}

void lcdSetThrottleSwitches(uint8_t portSwitch, uint8_t stdbSwitch) {
    status.throttlePortSelect = portSwitch;
    status.throttleStarboardSelect = stdbSwitch;
}

void lcdSetThrottleRaw(uint16_t value) {
    status.throttleValueRaw = value;
}

void lcdSetThrottlePerMilleSmooth(uint16_t value) {
    status.throttleValuePerMilleSmooth = value;
}

void lcdSetThrottlePerMille(uint16_t value) {
    status.throttleValuePerMille = value;
}

void lcdSetThrottleDac(uint16_t value) {
    status.throttleValueDac = value;
}

void lcdSetRegenRaw(uint16_t value) {
    status.regenValueRaw = value;
}

void lcdSetRegenPerMilleSmooth(uint16_t value) {
    status.regenValuePerMilleSmooth = value;
}

void lcdSetRegenPerMille(uint16_t value) {
    status.regenValuePerMille = value;
}

void lcdSetRegenDac(uint16_t value) {
    status.regenValueDac = value;
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

void lcdInit() {
    // Ground the red pin, set high the green and blue pins
  HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD1_BL_GREEN_GPIO_Port, LCD1_BL_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD1_BL_BLUE_GPIO_Port, LCD1_BL_BLUE_Pin, GPIO_PIN_SET);
  memset(&status,0,sizeof(status));

  MX_USART2_UART_Init();
}

void lcdHandlePageButtons() {

// The main switch controls the dashboard lighting, not throttle
    if (status.mainSwitch == 0) {
        // Turn off the helm lights and screens
        // And return. No need to handle switches, display is OFF
        HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, GPIO_PIN_RESET);
        return;
    }
    
    // Turn on the helm lights and creens
     HAL_GPIO_WritePin(LCD1_BL_RED_GPIO_Port, LCD1_BL_RED_Pin, GPIO_PIN_SET);
    
    // Handle LCD paging
    // Depending on how frequently this is called, a de-bouncing might be required.
    if (HAL_GPIO_ReadPin(LCD1_SELECT_NEXT_GPIO_Port, LCD1_SELECT_NEXT_Pin) == GPIO_PIN_RESET) {
        // NEXT switch pressed
        lcd1ShowPage = (lcd1ShowPage + 1) % lcd1Pages;
    } else if (HAL_GPIO_ReadPin(LCD1_SELECT_PREV_GPIO_Port, LCD1_SELECT_PREV_Pin) == GPIO_PIN_RESET) {
        // PREV switch pressed
        if (lcd1ShowPage == 0) { 
            lcd1ShowPage = lcd1Pages;
        } else {
            lcd1ShowPage--;
        }
    }
}

void lcdPrint() {
    
    #define MOTOR_SELECTED status.motorSelect?10:5

    static uint16_t counter = 0;
    char buffer[256];

    uint16_t throttlePosition;
    uint16_t regenPosition;

    if (status.mainSwitch == 0) {
        return;
    }
// Display data    
    switch(lcd1ShowPage) {
    case 0: 
      // Page 1 - Switch positions
        sprintf(buffer, "%d - Switches: Main: [%d], E-Stop: [%d], Speed High/Low: [%d/%d], Throttle Stbd/Port: [%d/%d], Select 5kw Motor: [%d]\n\r", 
                        counter, status.mainSwitch, status.throttleStopSwitch, status.speedHighSwitch, status.speedLowSwitch, 
                        status.throttleStarboardSelect, status.throttlePortSelect, status.motorSelect);
        
    break; 
    
    case 1:    
      // Page 2 - Speed 
      if (status.throttleStopSwitch == 1) {
        sprintf(buffer, "%d - Emegergency Stop!\n\r", counter);
      } else if (status.speedLowSwitch) {
        sprintf(buffer, "%d - Turtle speed on: [%dkw] Motor.\n\r", counter, MOTOR_SELECTED);
      } else if (status.speedHighSwitch) {
        sprintf(buffer, "%d - Turbo speed on: [%dkw] Motor!\n\r", counter, MOTOR_SELECTED);
      } else {
        sprintf(buffer, "%d - Normal speed on: [%dkw] Motor\n\r", counter, MOTOR_SELECTED);
      }
    break;

    case 2:
      // Page 3 - Throttle
      if (status.throttleStopSwitch == 1) {
        sprintf(buffer, "%d - Emergency Stop! Speed set to neutral! Raw throttle position on selected throttle: [%d]\n\r", counter, status.throttleValueRaw);
      } else {
        if (status.throttlePortSelect) {
          sprintf(buffer, "%d - Port (selected) throttle (raw/per mille/smooth): [%d/%d/%d], out: [%d] to: [%dkw] motor.\n\r", counter, 
                          status.throttleValueRaw, status.throttleValuePerMille,status.throttleValuePerMilleSmooth, status.throttleValueDac, MOTOR_SELECTED);
        } else if (status.throttleStarboardSelect) {
          sprintf(buffer, "%d - Starboad (selected) throttle (raw/per mille/smooth): [%d/%d/%d], out: [%d] to: [%dkw] motor.\n\r", counter, 
                          status.throttleValueRaw, status.throttleValuePerMille,status.throttleValuePerMilleSmooth, status.throttleValueDac, MOTOR_SELECTED);
        } else {
          sprintf(buffer, "%d - Starboard (default) Throttle (raw/per mille/smooth): [%d/%d/%d], out: [%d] to: [%dkw] motor.\n\r", counter, 
                           status.throttleValueRaw, status.throttleValuePerMille,status.throttleValuePerMilleSmooth, status.throttleValueDac, MOTOR_SELECTED);
        }
      }
      break;

    case 3:
      // Page 4 - Regen
      if (status.throttleStopSwitch == 1) {
        sprintf(buffer, "%d - Emergency Stop! Max regen set! Raw/per mille/Smooth: [%d/%d/%d], out: [%d] to: [%dkw] motor.\n\r", counter, 
                         status.regenValueRaw, status.regenValuePerMille, status.regenValuePerMilleSmooth, status.regenValueDac, MOTOR_SELECTED);
      } else {
        sprintf(buffer, "%d - Regen (Raw/smooth): [%d/%d/%d], out: [%d] to: [%dkw] motor.\n\r", counter, 
                         status.regenValueRaw, status.regenValuePerMille, status.regenValuePerMilleSmooth, status.regenValueDac, MOTOR_SELECTED);
      }
      break;

    case 4:
      // Page 4 - Throttle and Regen Percentage
      throttlePosition = status.throttleValuePerMille / 10;
      regenPosition = status.regenValuePerMille / 10;
      if (status.throttleStopSwitch == 1) {
        sprintf(buffer, "%d - Emergency Stop! Throttle in Neutral, Regen Max!.\n\r", counter);
      } else {
        // throttleDirection: 0 = neutral, 1 = forward, 2 = reverse
        if (status.throttleValuePerMille == 500) {
          // In neutral
          sprintf(buffer, "%d - Throttle in Neutral, Regen: [%d%%].\n\r", counter, status.regenValuePerMille / 10);
        } else if (status.throttleValuePerMille > 500) {
          // Going forward          
          if (status.throttlePortSelect == 1) {
            // Port throttle active
            sprintf(buffer, "%d - Forward: [%d%%], Regen: [%d%%] (Port Throttle to: [%dkw] motor).\n\r", counter, 
                             throttlePosition,  regenPosition, MOTOR_SELECTED);
          } else if (status.throttleStarboardSelect) {
            // Starboard throttle active
            sprintf(buffer, "%d - Forward: [%d%%], Regen: [%d%%] (Starboard Throttle to: [%dkw] motor).\n\r", counter, throttlePosition, regenPosition, MOTOR_SELECTED);
          } else {
            // Remote (eventually, Starboard now) throttle active
            sprintf(buffer, "%d - Forward: [%d%%], Regen: [%d%%] (Remote Throttle to: [%dkw] motor).\n\r", counter, throttlePosition, regenPosition, MOTOR_SELECTED);            
          }
        } else {
          // Going in reverse
          // throttleActive; 0 = port, 1 = remote, 2 = starboard
          if (status.throttlePortSelect == 1) {
            // Port throttle active
            sprintf(buffer, "%d - Reverse: [%d%%], Regen: [%d%%] (Port Throttle to: [%dkw] motor).\n\r", counter, throttlePosition, regenPosition, MOTOR_SELECTED);
          } else if (status.throttleStarboardSelect == 1) {
            // Starboard throttle active
            sprintf(buffer, "%d - Reverse: [%d%%], Regen: [%d%%] (Starboard Throttle to: [%dkw] motor).\n\r", counter, throttlePosition, regenPosition, MOTOR_SELECTED);
          } else {
             // Remote (eventually, Starboard now) throttle active
            sprintf(buffer, "%d - Reverse: [%d%%], Regen: [%d%%] (Remote Throttle to: [%dkw] motor).\n\r", counter, throttlePosition, regenPosition, MOTOR_SELECTED);
            
          }
        }
      }
      break;

      default: 
        sprintf(buffer,"Invalid LCD page %d!\n\r",lcd1ShowPage);
      break;
    }

    HAL_UART_Transmit(&huart2,(uint8_t*)buffer,strlen(buffer),1000);  // Sending in normal mode
    counter++;
}

