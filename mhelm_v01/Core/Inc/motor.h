
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <inttypes.h>

/// @brief Lebels for the active motor
typedef enum {
    MOTOR_5KW = 0,
    MOTOR_10KW = 1,
    NUM_MOTORS
} Motor_t;


// Motor select switch
#define MOTOR_5KW_SELECT_Pin GPIO_PIN_5
#define MOTOR_5KW_SELECT_GPIO_Port GPIOB

// DAC Address Pins
#define M10KW_THROTTLE_DAC_A0_Pin GPIO_PIN_10
#define M10KW_THROTTLE_DAC_A0_GPIO_Port GPIOC
#define M5KW_THROTTLE_DAC_A0_Pin GPIO_PIN_8
#define M5KW_THROTTLE_DAC_A0_GPIO_Port GPIOB


// Calibrated Values - These need to be per-sensor, and ideally read from a user-editable field
// DAC values (common regardless of the throttle in use, but needs to be tested/adjusted per DAC)
// 10kw motor DACs
#define THROTTLE_DAC_M10KW_FORWARD_SLOWEST 2130 // Slowest forward, 1965 on this DAC is (2.4?)v
#define THROTTLE_DAC_M10KW_FORWARD_FASTEST 3975 // Fastest forward, TPS Dead Low is set to 0.05vDC, 6 on this DAC is 5.73mvDC
#define THROTTLE_DAC_M10KW_NEUTRAL         2020 // The value that sets the motor to neutral, 2.5v
#define THROTTLE_DAC_M10KW_REVERSE_SLOWEST 1965 // Slowest reverse, 2130 on this DAC is (2.6?)v
#define THROTTLE_DAC_M10KW_REVERSE_FASTEST 75   // Fastest reverse, TPS Dead High is set to 4.95vDC, 4090 on this DAC is 4993.4mvDC
// 5kw motor DACs
#define THROTTLE_DAC_M5KW_FORWARD_SLOWEST 2130 // Slowest forward, 1965 on this DAC is (2.4?)v
#define THROTTLE_DAC_M5KW_FORWARD_FASTEST 3950 // Fastest forward, TPS Dead Low is set to 0.05vDC, 6 on this DAC is 5.73mvDC
#define THROTTLE_DAC_M5KW_NEUTRAL         2010 // The value that sets the motor to neutral, 2.5v
#define THROTTLE_DAC_M5KW_REVERSE_SLOWEST 1965 // Slowest reverse, 2130 on this DAC is (2.6?)v
#define THROTTLE_DAC_M5KW_REVERSE_FASTEST 75   // Fastest reverse, TPS Dead High is set to 4.95vDC, 4090 on this DAC is 4993.4mvDC

// ANd the regen values
#define REGEN_DAC_M10KW_MINIMUM 45          // The TPS Dead Low is set to 0.05vDC, 6  5.73 mvDC
#define REGEN_DAC_M10KW_MAXIMUM 3975        // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC
#define REGEN_DAC_M5KW_MINIMUM  45          // The TPS Dead Low is set to 0.05vDC, 6  5.73 mvDC
#define REGEN_DAC_M5KW_MAXIMUM  3950        // The TPS Dead High is set to 4.95vDC, 4090  4993.4vDC

// Regen DACS
#define M10KW_REGEN_DAC_A0_Pin GPIO_PIN_12
#define M10KW_REGEN_DAC_A0_GPIO_Port GPIOC
#define M5KW_REGEN_DAC_A0_Pin GPIO_PIN_9
#define M5KW_REGEN_DAC_A0_GPIO_Port GPIOB


/// @brief Initializes the motor.
/// Sets the EEPROM of the motor DACS to neutlral and
/// Initializes the DACs to the inactive address
/// @note Should be called before anything is sent to the DACs.
void initMotor();


/**
 * 
 * @brief Send throttle and regen positions to the motor.
 * This function handles sending throttle and regen values to the motors
 * It get as input the per mille values for regen and throttle, maps them to the 
 * dac values for the active motors and sends the data out.
 * @param throttlePerMille The position of the throttle poti in per mille (500 is neutral)
 * @param regenPerMille The position of the regen poti in per mille
*/
void handleMotor(uint16_t throttlePerMille, uint16_t regenPerMille);

#endif