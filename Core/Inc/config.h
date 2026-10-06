/*
/*
 * config.h
/*
 * config.h
 *
 * SmartESC STM32 V3 / M365
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "stdint.h"

//------------------------------------------------------------
// Display
//------------------------------------------------------------

#define DISPLAY_TYPE_M365DASHBOARD (1<<1)
#define DISPLAY_TYPE_DEBUG (1<<0)
#define DISPLAY_TYPE DISPLAY_TYPE_M365DASHBOARD

//------------------------------------------------------------
// Throttle / brake
//------------------------------------------------------------

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define SPEEDFILTER 3

#define THROTTLEOFFSET 45
#define THROTTLEMAX 175

#define BRAKEOFFSET 50
#define BRAKEMAX 100

//------------------------------------------------------------
// Calibration
//------------------------------------------------------------

#define CAL_BAT_V 14
#define CAL_V 25
#define CAL_I (38LL<<8)

//------------------------------------------------------------
// Motor parameters
//------------------------------------------------------------

#define SPEC_ANGLE 0
#define KV 77

#define INDUCTANCE 6LL
#define RESISTANCE 40LL
#define FLUX_LINKAGE 1200LL
#define GAMMA 9LL

//------------------------------------------------------------
// Current controller
//------------------------------------------------------------

#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2

#define P_FACTOR_I_D 100
#define I_FACTOR_I_D 10

//------------------------------------------------------------
// Speed calculation
//------------------------------------------------------------

#define WHEEL_CIRCUMFERENCE 2302
#define GEAR_RATIO 45

//------------------------------------------------------------
// Speed limits
//------------------------------------------------------------

#define SPEEDLIMIT_ECO 20
#define SPEEDLIMIT_NORMAL 40
#define SPEEDLIMIT_SPORT 60

//------------------------------------------------------------
// Phase current limits (mA)
//------------------------------------------------------------

#define PH_CURRENT_MAX_ECO 20000
#define PH_CURRENT_MAX_NORMAL 45000
#define PH_CURRENT_MAX_SPORT 65000

//------------------------------------------------------------
// Regen
//------------------------------------------------------------

#define REGEN_CURRENT 5000
#define REGEN_MAX_CURRENT 5000
#define REGEN_CURRENT_MAX 5000

//------------------------------------------------------------
// Field weakening
//------------------------------------------------------------

#define FW_CURRENT_MAX 0
#define FIELD_WEAKENING_CURRENT_MAX 0
#define FIELD_WEAKNING_CURRENT_MAX 0

//------------------------------------------------------------
// Battery current
//------------------------------------------------------------

#define BATTERYCURRENT_MAX 45000
#define BATTERISTRÖM_MAX 45000

//------------------------------------------------------------
// Battery voltage limits
//------------------------------------------------------------

#define BATTERYVOLTAGE_MIN 39000
#define BATTERYVOLTAGE_MAX 54600

//------------------------------------------------------------
// Battery level thresholds
//------------------------------------------------------------

#define BATTERINIVÅ_1 323000
#define BATTERINIVÅ_2 329000
#define BATTERINIVÅ_3 334400
#define BATTERINIVÅ_4 436000
#define BATTERINIVÅ_5 546000

//------------------------------------------------------------
// ADC channels
//------------------------------------------------------------

#define ADC_VOLTAGE 0
#define ADC_THROTTLE 1
#define ADC_TEMP 2

//------------------------------------------------------------
// Motor direction
//------------------------------------------------------------

#define REVERSE 1

//------------------------------------------------------------
// Speed PLL
//------------------------------------------------------------
// SPEED_PLL intentionally not defined.
// motor.c uses #ifdef SPEED_PLL, so defining it as 0
// would still enable the PLL code.

#define P_FACTOR_PLL 9
#define I_FACTOR_PLL 10

#endif /* CONFIG_H_ */
