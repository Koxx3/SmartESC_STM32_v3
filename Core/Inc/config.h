/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"

//Dangerzone, do not touch!!
#define DISPLAY_TYPE_M365DASHBOARD (1<<1)
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028

#define SPEEDFILTER 3

//#define ADCTHROTTLE
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC

// choose your display here
#define DISPLAY_TYPE DISPLAY_TYPE_M365DASHBOARD

// calibration factors for voltage and current
#define CAL_BAT_V 14 	// ADC counts * CAL_BAT_V = Battery voltage in mV
#define CAL_I 38		// ADC counts * CAL_I = current in mA

// gains for PI controls
#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 100
#define I_FACTOR_I_D 10

// min and max values of throttle and brake signals in ADC counts
#define THROTTLEOFFSET 45
#define THROTTLEMAX 175
#define BRAKEOFFSET 50
#define BRAKEMAX 190

// parameters for speed calculation
#define WHEEL_CIRCUMFERENCE 550 //690 for original M365 motor
#define GEAR_RATIO 11 //15 for original M365 motor

// speed limits for invividual modes in kph
#define SPEEDLIMIT_ECO 12
#define SPEEDLIMIT_NORMAL 20
#define SPEEDLIMIT_SPORT 50

// motor current limits for invividual modes in mA, see default settings at https://max.cfw.sh/#
#define PH_CURRENT_MAX_ECO 16000
#define PH_CURRENT_MAX_NORMAL 28000
#define PH_CURRENT_MAX_SPORT 55000

// motor current limit for regen in mA
#define REGEN_CURRENT 20000

// maximum current for flux weakening in mA
#define FW_CURRENT_MAX 18000 //max id

// maximum battery currents in mA
#define BATTERYCURRENT_MAX 14500
#define REGEN_CURRENT_MAX 10000

// battery voltage limits in mV
#define BATTERYVOLTAGE_MIN 33000
#define BATTERYVOLTAGE_MAX 42000


// motor spinning direction
#define REVERSE 1 //1 for original M365 motor

// settings for speed PLL (angle estimation)
#define SPEED_PLL 1 //1 for using PLL, 0 for angle extrapolation
#define P_FACTOR_PLL 10 //7 for original M365 motor
#define I_FACTOR_PLL 10 //7 for original M365 motor

#endif /* CONFIG_H_ */
