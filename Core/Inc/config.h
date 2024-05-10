/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_EBiCS (1<<5)                  // ANT+LEV protocol
#define DISPLAY_TYPE_VESC_TOOL (1<<1)							// For ASCII-Output in Debug mode);
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 256
#define CAL_V 25
#define CAL_I 38LL<<8
#define INDUCTANCE 6LL
#define RESISTANCE 40LL
#define FLUX_LINKAGE 1200LL
#define GAMMA 9LL
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000
#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 1
#define I_FACTOR_I_D 1
#define MAX_D_FACTOR 1
#define ADCTHROTTLE
#define THROTTLEOFFSET 900
#define THROTTLEMAX 3500
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 60
#define SPEEDLIMIT 45

#define PH_CURRENT_MAX 600
#define BATTERYCURRENT_MAX 10000
#define SPEC_ANGLE -167026406L //BionX IGH3 -143165476

#define DISPLAY_TYPE DISPLAY_TYPE_VESC_TOOL
#define REVERSE 1

#define VOLTAGE_MIN 300
#define REGEN_CURRENT 200
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC

#define REGEN_CURRENT_MAX 10000

//#define SPEED_PLL
#define P_FACTOR_PLL 10
#define I_FACTOR_PLL 9

#endif /* CONFIG_H_ */
 
