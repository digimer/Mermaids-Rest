
#include <inttypes.h>

#include "i2cHelpers.h"
#include "lcd.h"
#include "motor.h"

const uint16_t MOTOR_MAX_REVERSE[NUM_MOTORS]             = { THROTTLE_DAC_M5KW_REVERSE_FASTEST, THROTTLE_DAC_M10KW_REVERSE_FASTEST };
const uint16_t MOTOR_MIN_REVERSE[NUM_MOTORS]             = { THROTTLE_DAC_M5KW_REVERSE_SLOWEST, THROTTLE_DAC_M10KW_REVERSE_SLOWEST };
const uint16_t MOTOR_NEUTRAL[NUM_MOTORS]                 = { THROTTLE_DAC_M5KW_NEUTRAL, THROTTLE_DAC_M10KW_NEUTRAL };
const uint16_t MOTOR_MIN_FORWARD[NUM_MOTORS]             = { THROTTLE_DAC_M5KW_FORWARD_SLOWEST, THROTTLE_DAC_M10KW_FORWARD_SLOWEST };
const uint16_t MOTOR_MAX_FORWARD[NUM_MOTORS]             = { THROTTLE_DAC_M5KW_FORWARD_FASTEST, THROTTLE_DAC_M10KW_FORWARD_FASTEST };
const uint16_t MOTOR_MIN_REGEN[NUM_MOTORS]               = { REGEN_DAC_M5KW_MINIMUM, REGEN_DAC_M10KW_MINIMUM };
const uint16_t MOTOR_MAX_REGEN[NUM_MOTORS]               = { REGEN_DAC_M5KW_MAXIMUM, REGEN_DAC_M10KW_MAXIMUM };
const GPIO_TypeDef *MOTOR_THROTTLE_DAC_PORTS[NUM_MOTORS] = { M5KW_THROTTLE_DAC_A0_GPIO_Port, M10KW_THROTTLE_DAC_A0_GPIO_Port };
const uint16_t MOTOR_THROTTLE_DAC_PINS[NUM_MOTORS]       = { M5KW_THROTTLE_DAC_A0_Pin, M10KW_THROTTLE_DAC_A0_Pin };
const GPIO_TypeDef *MOTOR_REGEN_DAC_PORTS[NUM_MOTORS]    = { M5KW_REGEN_DAC_A0_GPIO_Port, M10KW_REGEN_DAC_A0_GPIO_Port };
const uint16_t MOTOR_REGEN_DAC_PINS[NUM_MOTORS]          = { M5KW_REGEN_DAC_A0_Pin, M10KW_REGEN_DAC_A0_Pin};

static Motor_t motorActive = MOTOR_5KW;

void initMotor() {
  // Write to eeprom the safe defaults, so on reboot the throttles default
  // to 2.5v and the regen DACs to 0. 05v. First, set the 10KW throttle to
  // neutral.
  uint8_t throttle_data[3];
  uint8_t regen_data[3];
  uint16_t i;

  // Initializte all the DACS to inactive
  //  Set the address pins for the DACs to '1' giving them all the address 0x63
  I2C_DEACTIVATE_DAC(M10KW_THROTTLE_DAC_A0_GPIO_Port, M10KW_THROTTLE_DAC_A0_Pin);
  I2C_DEACTIVATE_DAC(M10KW_REGEN_DAC_A0_GPIO_Port, M10KW_REGEN_DAC_A0_Pin);
  I2C_DEACTIVATE_DAC(M5KW_THROTTLE_DAC_A0_GPIO_Port, M5KW_THROTTLE_DAC_A0_Pin);
  I2C_DEACTIVATE_DAC(M5KW_REGEN_DAC_A0_GPIO_Port, M5KW_REGEN_DAC_A0_Pin);

  throttle_data[0] = MCP4725_CMD_WRITEDACEEPROM;
  regen_data[0]    = MCP4725_CMD_WRITEDACEEPROM;

  for (i = 0; i < NUM_MOTORS; ++i) {
    // Neutral for the motor
    throttle_data[1] = HIPAR(MOTOR_NEUTRAL[i]);
    throttle_data[2] = LOPAR(MOTOR_NEUTRAL[i]);

    // And minimum for the regen
    regen_data[1] = HIPAR(MOTOR_MIN_REGEN[i]);
    regen_data[2] = LOPAR(MOTOR_MIN_REGEN[i]);
    // And now send them out.
    I2cSendtoDAC(MOTOR_THROTTLE_DAC_PORTS[i], MOTOR_THROTTLE_DAC_PINS[i], throttle_data, 1000);
    I2cSendtoDAC(MOTOR_REGEN_DAC_PORTS[i], MOTOR_REGEN_DAC_PINS[i], regen_data, 1000);
  }
}

/// Faster version of map(), since we have the position already.
/// Maps the permille value into [MINVAL,MAXVAL]
/// @note this has top be calculated in 32bit, to avoid overruns. Therefore the
/// cast.
#define MAP_TO_VALUE(PERMILLEVAL, MINVAL, MAXVAL)                              \
  ((uint32_t)(PERMILLEVAL)) * (uint32_t)((MAXVAL) - (MINVAL)) / 1000 + (MINVAL)

void handleMotor(uint16_t throttlePerMille, uint16_t regenPerMille) {

  GPIO_PinState motorSwitch;
  uint16_t motorThrottleValue;
  uint16_t motorRegenValue;
  uint8_t throttle_data[3];
  uint8_t regen_data[3];
  Motor_t newMotor;
  uint8_t i;

  // Read position of motor switch
  motorSwitch = HAL_GPIO_ReadPin(MOTOR_5KW_SELECT_GPIO_Port, MOTOR_5KW_SELECT_Pin);
  // Store it for output
  lcdSetMotorSwitch(motorSwitch);

  // Select the motor
  if (motorSwitch == GPIO_PIN_RESET) {
    // 0 means 5k
    newMotor = MOTOR_5KW;
  } else {
    // 1 means 10k
    newMotor = MOTOR_10KW;
  }

  // Motor changed?
  if (newMotor != motorActive) {
    // TODO: When switching motors, add a ramp to ramp down the old motor and
    // ramp up the new motor Think about what happens if we switch back during
    // ramping....
    motorActive = newMotor;
  }

  if (throttlePerMille == 500) {
    // Mid point, use the neutral value
    motorThrottleValue = MOTOR_NEUTRAL[motorActive];
  } else {
    // Just map the per mille values to the DAC values for the active motor
    motorThrottleValue = MAP_TO_VALUE(throttlePerMille, MOTOR_MAX_REVERSE[motorActive], MOTOR_MAX_FORWARD[motorActive]);
  }
  motorRegenValue = MAP_TO_VALUE(regenPerMille, MOTOR_MIN_REGEN[motorActive], MOTOR_MAX_REGEN[motorActive]);

  // Store DAC values for output
  lcdSetThrottleDac(motorThrottleValue);
  lcdSetRegenDac(motorRegenValue);

  throttle_data[0] = MCP4725_CMD_WRITEDAC;
  regen_data[0]    = MCP4725_CMD_WRITEDAC;

  // Send the settings to the motors.
  //  It's a little overkill for only 2 motors, but if we can have more than 2,
  //  who knows. Maybe the mermaid will once live on a superyard ;-)
  for (i = 0; i < NUM_MOTORS; ++i) {
    if (i == motorActive) {
      // To the active motor send the throttle values
      throttle_data[1] = HIPAR(motorThrottleValue);
      throttle_data[2] = LOPAR(motorThrottleValue);

      // Or the regen values respectively
      regen_data[1] = HIPAR(motorRegenValue);
      regen_data[2] = LOPAR(motorRegenValue);
    } else {
      // To the inactive ones send the matching neutral value
      throttle_data[1] = HIPAR(MOTOR_NEUTRAL[i]);
      throttle_data[2] = LOPAR(MOTOR_NEUTRAL[i]);

      // And minimum for the regen
      regen_data[1] = HIPAR(MOTOR_MIN_REGEN[i]);
      regen_data[2] = LOPAR(MOTOR_MIN_REGEN[i]);
    }

    // And now send them out.
    I2cSendtoDAC(MOTOR_THROTTLE_DAC_PORTS[i], MOTOR_THROTTLE_DAC_PINS[i], throttle_data, 1000);
    I2cSendtoDAC(MOTOR_REGEN_DAC_PORTS[i], MOTOR_REGEN_DAC_PINS[i], regen_data, 1000);
  }
}