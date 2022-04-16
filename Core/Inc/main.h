#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <arm_math.h>
#include <stdbool.h>
#include "utils.h"
#include "motor.h"

void Error_Handler(void);

extern void UserSysTickHandler(void);

typedef struct {
  q31_t battery_voltage;
  int16_t phase_current_limit;
  int16_t regen_max_current;
	int8_t temperature;
	int8_t mode;
	bool light;
	bool beep;
  int32_t i_q_setpoint_target;
  uint32_t speed;
	bool brake_active;
  uint32_t shutdown;
  int8_t speed_limit;
  int8_t error_state;
} M365State_t;

enum modes {
  eco = 2,
  normal = 0,
  sport = 4
};

void _Error_Handler(char*, int);

#ifdef __cplusplus
}
#endif
