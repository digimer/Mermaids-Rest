#ifndef __THROTTLE_H__
#define __THROTTLE_H

#define THROTTLE_AVERAGE_OVER 8        // Average the reading over this many reads. Must be proportional to delayTime - Max 255

// Throttle - Starboard
#define THROTTLE_STBD_REVERSE_MAX  1140     // The throttle pot value equalling the fastest speed in reverse
#define THROTTLE_STBD_REVERSE_MIN  1850     // The throttle pot value equalling the slowest speed in reverse
#define THROTTLE_STBD_FORWARD_MIN  2040     // The throttle pot value equalling the slowest speed forward
#define THROTTLE_STBD_FORWARD_MAX  2740     // The throttle pot value equalling the fastest speed forward
#define REVERSE_STBD_SENSOR_STEPS  (THROTTLE_STBD_REVERSE_MIN - THROTTLE_STBD_REVERSE_MAX)
#define FORWARD_STBD_SENSOR_STEPS  (THROTTLE_STBD_FORWARD_MAX - THROTTLE_STBD_FORWARD_MIN)
#define NEUTRAL_STBD_MID_POINT     (((THROTTLE_STBD_FORWARD_MIN - THROTTLE_STBD_REVERSE_MIN) / 2) + THROTTLE_STBD_REVERSE_MIN)
#define REVERSE_STBD_PERCENT_STEPS 100 / REVERSE_STBD_SENSOR_STEPS
#define FORWARD_STBD_PERCENT_STEPS 100 / FORWARD_STBD_SENSOR_STEPS

// Throttle - Port
#define THROTTLE_PORT_REVERSE_MAX  1450     // The throttle pot value equalling the fastest speed in reverse
#define THROTTLE_PORT_REVERSE_MIN  1920     // The throttle pot value equalling the slowest speed in reverse
#define THROTTLE_PORT_FORWARD_MIN  2100     // The throttle pot value equalling the slowest speed forward
#define THROTTLE_PORT_FORWARD_MAX  2520     // The throttle pot value equalling the fastest speed forward
#define REVERSE_PORT_SENSOR_STEPS  (THROTTLE_PORT_REVERSE_MIN - THROTTLE_PORT_REVERSE_MAX)
#define FORWARD_PORT_SENSOR_STEPS  (THROTTLE_PORT_FORWARD_MAX - THROTTLE_PORT_FORWARD_MIN)
#define NEUTRAL_PORT_MID_POINT     (((THROTTLE_PORT_FORWARD_MIN - THROTTLE_PORT_REVERSE_MIN) / 2) + THROTTLE_PORT_REVERSE_MIN)
#define REVERSE_PORT_PERCENT_STEPS 100 / REVERSE_PORT_SENSOR_STEPS
#define FORWARD_PORT_PERCENT_STEPS 100 / FORWARD_PORT_SENSOR_STEPS

// Regen 
#define REGEN_POT_MINIMUM       0
#define REGEN_POT_MAXIMUM       4030        // By math, this should be 4095, but is less in practice and needs to be tested per pot

// Static Values 
// Starboard Throttle pot
#define THROTTLE_STBD_POT_Pin GPIO_PIN_0
#define THROTTLE_STBD_POT_GPIO_Port GPIOA

// Port Throttle pot
#define THROTTLE_PORT_POT_Pin GPIO_PIN_3
#define THROTTLE_PORT_POT_GPIO_Port GPIOC

// Throttle Select
#define THROTTLE_PORT_SELECT_Pin GPIO_PIN_14
#define THROTTLE_PORT_SELECT_GPIO_Port GPIOB
#define THROTTLE_STBD_SELECT_Pin GPIO_PIN_13
#define THROTTLE_STBD_SELECT_GPIO_Port GPIOB

// Used to describe which throttle is active
typedef enum
{
    THROTTLE_PORT = 0,
    THROTTLE_REMOTE = 1,
    THROTTLE_STARBOARD = 2,
    NUM_THROTTLES
} Throttles_t;

void initThrottle();
uint16_t handleThrottle(uint8_t stopSwitchEngaged);
uint16_t handleRegen(uint8_t stopSwitchEngaged);

#endif
