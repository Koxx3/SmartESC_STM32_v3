#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif


//------------------------------------------------------------
// Motor parameters
//------------------------------------------------------------

#ifndef WHEEL_CIRCUMFERENCE
#define WHEEL_CIRCUMFERENCE 2302
#endif

#ifndef GEAR_RATIO
#define GEAR_RATIO 45
#endif

#ifndef ADC_VOLTAGE
#define ADC_VOLTAGE 0
#endif

#ifndef CAL_BAT_V
#define CAL_BAT_V 14
#endif

#ifndef CAL_I
#define CAL_I (38LL << 8)
#endif

#ifndef BATTERYCURRENT_MAX
#define BATTERYCURRENT_MAX 45000
#endif

#ifndef REGEN_CURRENT_MAX
#define REGEN_CURRENT_MAX 5000
#endif

#ifndef P_FACTOR_I_Q
#define P_FACTOR_I_Q 100
#endif

#ifndef I_FACTOR_I_Q
#define I_FACTOR_I_Q 2
#endif

#ifndef P_FACTOR_I_D
#define P_FACTOR_I_D 100
#endif

#ifndef I_FACTOR_I_D
#define I_FACTOR_I_D 10
#endif

#ifndef MAX_D_FACTOR
#define MAX_D_FACTOR 1
#endif

#ifndef SPEEDFILTER
#define SPEEDFILTER 3
#endif

#ifndef SPEC_ANGLE
#define SPEC_ANGLE 0
#endif

#ifndef REVERSE
#define REVERSE 1
#endif

#ifndef P_FACTOR_PLL
#define P_FACTOR_PLL 9
#endif

#ifndef I_FACTOR_PLL
#define I_FACTOR_PLL 10
#endif

#ifndef SIXSTEPTHRESHOLD
#define SIXSTEPTHRESHOLD 9000
#endif


//------------------------------------------------------------
// ADC phase-current indexes
//
// motor.c stores six ADC values in adcData[].
// Phase A = ADC index 3
// Phase B = ADC index 4
// Phase C = ADC index 5
//------------------------------------------------------------

#ifndef ADC_CHANA
#define ADC_CHANA 3
#endif

#ifndef ADC_CHANB
#define ADC_CHANB 4
#endif

#ifndef ADC_CHANC
#define ADC_CHANC 5
#endif


//------------------------------------------------------------
// Angle estimation
//------------------------------------------------------------

enum angle_estimation
{
    EXTRAPOLATION = 0,
    SPEED_PLL
};


//------------------------------------------------------------
// Motor system states
//------------------------------------------------------------

enum
{
    Stop = 0,
    SixStep,
    Interpolation,
    PLL
};


//------------------------------------------------------------
// Motor error states
//------------------------------------------------------------

enum errors
{
    none = 0,
    hall = 18,
    lowbattery = 24,
    overcurrent = 4,
    brake = 15
};


//------------------------------------------------------------
// M365 operating modes
//------------------------------------------------------------

#ifndef eco
#define eco 0
#endif

#ifndef normal
#define normal 1
#endif

#ifndef sport
#define sport 2
#endif


//------------------------------------------------------------
// Public motor state
//------------------------------------------------------------

typedef struct
{
    q31_t i_q_setpoint_target;

    int16_t phase_current_limit;

    q31_t battery_voltage;

    q31_t battery_voltage_min;

    uint16_t field_weakening_current_max;

    int8_t system_state;

    int8_t mode;

    int8_t error_state;

    int8_t speed_limit;

    uint32_t speed;

    bool brake_active;

    bool field_weakening_enable;

    /*
     * ADC DMA buffer.
     *
     * motor.c uses:
     *
     * adcData[ADC_CHANA]
     * adcData[ADC_CHANB]
     * adcData[ADC_CHANC]
     *
     * and starts DMA with 6 values.
     */
    uint16_t adcData[16];

    uint32_t debug[10];

} MotorStatePublic_t;


//------------------------------------------------------------
// Internal motor state
//------------------------------------------------------------

typedef struct
{
    q31_t i_d;

    q31_t i_q;

    q31_t i_q_setpoint;

    q31_t i_d_setpoint;

    q31_t i_setpoint_abs;

    int32_t i_q_setpoint_temp;

    int32_t i_d_setpoint_temp;

    q31_t u_d;

    q31_t u_q;

    q31_t u_abs;

    q31_t Battery_Current;

    uint8_t char_dyn_adc_state;

    int8_t system_state;

    int8_t error_state;

    int16_t spec_angle;

    uint8_t assist_level;

    uint8_t regen_level;

    int16_t phase_current_limit;

    int8_t speed_limit;

    int8_t mode;

    enum angle_estimation angle_estimation;

    bool hall_angle_detect_flag;

} MotorState_t;


//------------------------------------------------------------
// Global motor state
//------------------------------------------------------------

extern MotorState_t MS;


//------------------------------------------------------------
// Motor functions
//------------------------------------------------------------

void motor_init(
    volatile MotorStatePublic_t* motorStatePublic
);

void motor_autodetect(void);

void motor_slow_loop(
    volatile MotorStatePublic_t* p_MotorStatePublic,
    M365State_t* p_M365State
);

int32_t speed_to_tics(uint8_t speed);

int8_t tics_to_speed(uint32_t tics);

void calculate_tic_limits(int8_t speed_limit);

q31_t speed_PLL(q31_t actual, q31_t target);

void get_standstill_position(void);

void runPIcontrol(void);


#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
