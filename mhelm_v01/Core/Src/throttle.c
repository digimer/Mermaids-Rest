
/**
 * Thid file contains all functions to handle the throttles and the regen.
 * The main functions here are initThrottle() and handleThrottle().
 * The latter will be called on every iteration to read the throttle position,
 * calculates a position in per mille.
*/

#include <inttypes.h>

#include "ioctl.h"
#include "i2cHelpers.h"
#include "throttle.h"

// Used for smoothing values
typedef struct {
    uint16_t size;         // Numer of entries in values
    uint16_t oldestIndex;  // Oldest array index, the one to be replaced next
    uint32_t total;        // Sum of all values in the array
    uint16_t values[0]     // Array with values (will be allocated dynamically)
} Smoothing_t;


// These will be used for smooting throttle and regen values
Smoothing_t *smoothingThrottle;
Smoothing_t *smoothingRegen;


// This tells us which throttle is currently active
static Throttles_t throttleActive = THROTTLE_STARBOARD; // 0 = port, 1 = remote, 2 = starboard

// Instead of usinf ifs to check which throttle is active,
// we simply use arrays to store the throttle specific values and use an 
// index for the active throttle to access the specific data.
// That allows to use the same code for all throttles without havind to distinguish in the code
// NOTE: we have to make sure that the enum Throttles_t matches the entries in the array

// Store the mid points as array so that we can access them via "throttleActive"
// Not clear currently what the value for the remote throttle will be, so I just set per mille values, with 500 being neutral
static const uint16_t MID_POINTS[NUM_THROTTLES] = {NEUTRAL_PORT_MID_POINT, 500, NEUTRAL_STBD_MID_POINT};

// Forward values for the throttles
static const uint16_t MIN_POINTS_FORWARD[NUM_THROTTLES] = {THROTTLE_PORT_FORWARD_MIN, 501, THROTTLE_STBD_FORWARD_MIN};
static const uint16_t MAX_POINTS_FORWARD[NUM_THROTTLES] = {THROTTLE_PORT_FORWARD_MAX, 1000, THROTTLE_STBD_FORWARD_MAX};

// NAd reverse values
static const uint16_t MIN_POINTS_REVERSE[NUM_THROTTLES] = {THROTTLE_PORT_REVERSE_MIN, 0, THROTTLE_STBD_REVERSE_MIN};
static const uint16_t MAX_POINTS_REVERSE[NUM_THROTTLES] = {THROTTLE_PORT_REVERSE_MAX, 499, THROTTLE_STBD_REVERSE_MAX};

// The ports for the throttles
static const uint8_t  IOCTL_PORTS[NUM_THROTTLES] = {IOCTL_THROTTLE_PORT_POT,255,IOCTL_THROTTLE_STBD_POT};


// Allocate memory for a new smoothing struct and initializes it.
// Might return NULL if memory allocation failed
static Smoothing_t *createSmoothing(uint16_t size, uint16_t initialValue) {
    Smoothing_t *result;
    
    result = (Smoothing_t*)malloc(sizeof(Smoothing_t)+size*sizeof(uint16_t));
    if(!result) {
        return NULL;
    }

    result->size = size;
    result->oldestIndex = 0;
    result->total = size * initialValue;

    //Initialize values
    for (uint16_t i = 0; i < size; ++i)
    {
        result->values[i] = initialValue;
    }
}

/**
 * This function uses an array of values to calc a glidig average over all calls to this function
 *  handle has to be allocated using thge function createSmoothing
*/
static uint16_t smoothValue(Smoothing_t *handle, uint16_t value) {
    
    // We know the current sum of all values.
    // Therefore, the new sum is the current sum minus oldest value plus new value
    handle->total = handle->total - handle->values[handle->oldestIndex] + value;
    handle->oldestIndex = (handle->oldestIndex + 1) % handle->size; // Modulo handles turnaround

    // TODO: Make sure this is rounded properly
    return handle->total / handle->size;
}

/** Code to initialize the throttle
 *   Especially setting safe start values into the EEProm
 * and setting up the smoothing
 */
void initThrottle()
{
    // Create setup for smoothing the throttle values
    // We will not smooth the raw values but the resulting per-mille values
    // That removes the necessity to handle different mid points
    smoothingThrottle = createSmoothing(THROTTLE_AVERAGE_OVER,500);
    smoothingRegen = createSmoothing(THROTTLE_AVERAGE_OVER,0);    
}


// that's a simpler version of the map() function that basically only imlements the first half
// Since we use factor 1000 here we ned 32 bit values for calculation
// The result fits into an uint16 again
uint16_t calcPerMille(uint32_t value, uint32_t low, uint32_t high) {
    return (uint16_t) ((value - low) * 1000 / (high - low));
}


// This function reads the necessary values and calculates a throttle position in per mille
// where 0 is reverse fastest, 500 is neutral and 1000 is forward fastest
// Therefore, 0-499 is reverse and 501-1000 is forward
// We use per mille to get a better resolution 
uint16_t handleThrottle(uint8_t stopSwitchEngaged)
{
    // NOTE: Eventually, if neither are selected, "remote control" will be enabled. For now, neither selected defaults to Stbd
    uint8_t throttleStbdSelect = 0; // 0 = Not selected, 1 = selected
    uint8_t throttlePortSelect = 0; // 0 = Not selected, 1 = selected
    Throttles_t newThrottle = throttleActive;
    uint16_t RegenValue    = 0;    // raw value,       Disable regen
    uint16_t throttleValue = 2047;
    uint16_t perMille;

    // Emergency Stop logic
    if (stopSwitchEngaged) {
    
     // Is smoothing what we want here?
     // It's an emergency, so maybe we should go to neutral right away?
      return smoothValue(smoothingThrottle,500);
    }
    
    // This is not really clean
    // HAL_GPIO_ReadPin returns a GPIO_PIN_STATE, not an int (at least formally)
    // Therefore its not good to do the subtraction here, even if it works
    throttleStbdSelect = 1 - HAL_GPIO_ReadPin(THROTTLE_STBD_SELECT_GPIO_Port, THROTTLE_STBD_SELECT_Pin);
    throttlePortSelect = 1 - HAL_GPIO_ReadPin(THROTTLE_PORT_SELECT_GPIO_Port, THROTTLE_PORT_SELECT_Pin);

    // Check which Throttle to use
    if ((throttleStbdSelect) || (!throttlePortSelect)) {
        newThrottle = THROTTLE_STARBOARD; // 0 = port, 1 = remote, 2 = starboard
    } else {
        newThrottle = THROTTLE_PORT; // 0 = port, 1 = remote, 2 = starboard
    }

    if(newThrottle != throttleActive) {
        // Throttle changed.
        
        // Do whatever has to be done to switch the throttles
        // Maybe nothing?
        throttleActive = newThrottle;
    }

    // This requires a special handling for the  remote throttle.
    // That's not yet implemented
    throttleValue = ioctl_get_pot(IOCTL_PORTS[throttleActive]); // get the adc value for the active throttle
   

    // Check border cases and enforce limits
    if (throttleValue < MAX_POINTS_REVERSE[throttleActive]) {
        // Beyond reverse max, so set to reverse max
        throttleValue = MAX_POINTS_REVERSE[throttleActive];
    } else if (throttleValue > MAX_POINTS_FORWARD[throttleActive]) {
        // Beyond forward max, so set to forward max
        throttleValue = MAX_POINTS_FORWARD[throttleActive];
    } else if ((throttleValue > MIN_POINTS_REVERSE[throttleActive]) && (throttleValue < MIN_POINTS_FORWARD[throttleActive])) {
        // Somewhere in the neutral zone, we simply return a (smoothed) 500
        return smoothValue(smoothingThrottle,500);
    }

    // Now calculate the position in perMille
    perMille = calcPerMille(throttleValue,MAX_POINTS_REVERSE[throttleActive],MAX_POINTS_FORWARD[throttleActive]);

    // And return the smoothed value
    return smoothValue(smoothingThrottle,perMille);
    
}

uint16_t handleRegen(uint8_t stopSwitchEngaged) {
    uint16_t regenValue;
    uint16_t perMille;
        
    // Emergency Stop logic
    if (stopSwitchEngaged) {    
        // Is smoothing what we want here?
        // It's an emergency, so maybe we should go to maximum right away?
        return smoothValue(smoothingRegen,1000);
    }

    regenValue = ioctl_get_pot(IOCTL_REGEN_POT);         // get the adc value for the regen

    // Restrict it to valid values
    if(regenValue < REGEN_POT_MINIMUM) {
        regenValue = REGEN_POT_MINIMUM;
    } else if (regenValue > REGEN_POT_MAXIMUM) {
        regenValue = REGEN_POT_MAXIMUM;
    }

    // Now calculate the position in per mille
    perMille = calcPerMille(regenValue,REGEN_POT_MINIMUM,REGEN_POT_MAXIMUM);

    // And return the smoothed value
    return smoothValue(smoothingRegen,perMille);    
}