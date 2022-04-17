/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "print.h"
#include "FOC.h"
#include "config.h"
#include "eeprom.h"
#include "button_processing.h"

#if (DISPLAY_TYPE == DISPLAY_TYPE_M365DASHBOARD)
#include "M365_Dashboard.h"
#endif

#include <stdlib.h>
#include <arm_math.h>
/* USER CODE END Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4; not used here but in stm32f1xx_hal_timebase_tim.c:29

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

int c_squared;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
q31_t get_battery_current(q31_t iq,q31_t id,q31_t uq,q31_t ud);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
q31_t raw_inj1;
q31_t raw_inj2;

uint32_t ui32_tim1_counter = 0;
uint32_t ui32_tim3_counter = 0;
uint8_t ui8_hall_state = 0;
uint8_t ui8_hall_state_old = 0;
uint8_t ui8_hall_case = 0;
uint8_t ui8_BC_limit_flag = 0;
uint16_t ui16_tim2_recent = 0;
uint16_t ui16_timertics = 5000; //timertics between two hall events for 60° interpolation
uint16_t ui16_reg_adc_value;
uint32_t ui32_reg_adc_value_filter;
uint16_t ui16_ph1_offset = 0;
uint16_t ui16_ph2_offset = 0;
uint16_t ui16_ph3_offset = 0;


uint16_t ui16_KV_detect_counter = 0; //for getting timing of the KV detect
static int16_t ui32_KV = 0;

volatile int16_t i16_ph1_current = 0;
volatile int16_t i16_ph2_current = 0;
volatile int16_t i16_ph2_current_filter = 0;
int16_t i16_ph3_current = 0;
uint16_t i = 0;
uint16_t j = 0;
uint16_t k = 0;
volatile uint8_t ui8_overflow_flag = 0;
volatile uint8_t ui8_adc_inj_flag = 0;
volatile uint8_t ui8_adc_regular_flag = 0;
int8_t i8_direction = REVERSE;
volatile int8_t i8_reverse_flag = 1; //for temporaribly reverse direction
volatile int8_t i8_slow_loop_flag = 0;
volatile uint8_t ui8_adc_offset_done_flag = 0;
volatile uint8_t ui8_print_flag = 0;
volatile uint8_t ui8_UART_flag = 0;
volatile uint8_t ui8_Push_Assist_flag = 0;
volatile uint8_t ui8_UART_TxCplt_flag = 1;
volatile uint8_t ui8_PAS_flag = 0;
volatile uint8_t ui8_SPEED_flag = 0;
volatile uint8_t ui8_6step_flag = 0;

static q31_t iq_cum=0;
static q31_t id_cum=0;
static q31_t uq_cum=0;
static q31_t ud_cum=0;


uint32_t uint32_PAS_HIGH_counter = 0;
uint32_t uint32_PAS_HIGH_accumulated = 32000;
uint32_t uint32_PAS_fraction = 100;
uint32_t uint32_SPEED_counter = 32000;
uint32_t uint32_PAS = 32000;

uint8_t ui8_UART_Counter = 0;
int8_t i8_recent_rotor_direction = 1;
int16_t i16_hall_order = 1;

uint32_t uint32_torque_cumulated = 0;
uint32_t uint32_PAS_cumulated = 32000;
uint16_t uint16_mapped_throttle = 0;
uint16_t uint16_mapped_PAS = 0;
uint16_t uint16_half_rotation_counter = 0;
uint16_t uint16_full_rotation_counter = 0;
int32_t int32_current_target = 0;
uint32_t uint32_SPEEDx100_cumulated=0;

q31_t q31_t_Battery_Current_accumulated = 0;
q31_t q31_Battery_Voltage = 0;

q31_t q31_rotorposition_absolute;
q31_t q31_rotorposition_hall;


q31_t q31_rotorposition_PLL = 0;
q31_t q31_angle_per_tic = 0;
q31_t q31_PLL_error=0;
q31_t q31_u_d_temp = 0;
q31_t q31_u_q_temp = 0;
int16_t i16_sinus = 0;
int16_t i16_cosinus = 0;
char buffer[256];
char char_dyn_adc_state = 1;
char char_dyn_adc_state_old = 1;
uint8_t assist_factor[10] = { 0, 51, 102, 153, 204, 255, 255, 255, 255, 255 };

uint16_t VirtAddVarTab[NB_OF_VAR] = { 	EEPROM_POS_HALL_ORDER,
										EEPROM_POS_HALL_45,
										EEPROM_POS_HALL_51,
										EEPROM_POS_HALL_13,
										EEPROM_POS_HALL_32,
										EEPROM_POS_HALL_26,
										EEPROM_POS_HALL_64
									};
enum{Stop, SixStep, Interpolation, PLL, IdleRun};
q31_t switchtime[3];
volatile uint16_t adcData[8]; //Buffer for ADC1 Input


//Rotor angle scaled from degree to q31 for arm_math. -180°-->-2^31, 0°-->0, +180°-->+2^31 read in from EEPROM
q31_t Hall_13 = 0;
q31_t Hall_32 = 0;
q31_t Hall_26 = 0;
q31_t Hall_64 = 0;
q31_t Hall_51 = 0;
q31_t Hall_45 = 0;

const q31_t deg_30 = 357913941;


const uint16_t fw_current_max = FW_CURRENT_MAX/CAL_I;

static q31_t tics_lower_limit;
static q31_t tics_higher_limit;
q31_t q31_tics_filtered = 128000;
//variables for display communication

#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))

MotorState_t MS;
MotorParams_t MP;

//structs for PI_control
PI_control_t PI_iq;
PI_control_t PI_id;

int16_t battery_percent_fromcapacity = 50; //Calculation of used watthours not implemented yet
int16_t wheel_time = 1000;//duration of one wheel rotation for speed calculation
int16_t current_display;				//pepared battery current for display

int16_t power;

static void dyn_adc_state(q31_t angle);
static void set_inj_channel(char state);
void get_standstill_position();
void runPIcontrol();
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max);
int32_t speed_to_tics(uint8_t speed);
int8_t tics_to_speed(uint32_t tics);
int16_t internal_tics_to_speedx100 (uint32_t tics);
q31_t speed_PLL (q31_t ist, q31_t soll);

#define JSQR_PHASE_A 0b00011000000000000000 //3
#define JSQR_PHASE_B 0b00100000000000000000 //4
#define JSQR_PHASE_C 0b00101000000000000000 //5

#define ADC_VOLTAGE 0
#define ADC_THROTTLE 1
#define ADC_TEMP 2
#define ADC_CHANA 3
#define ADC_CHANB 4
#define ADC_CHANC 5

void autodetect() {
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
	MS.hall_angle_detect_flag = 0; //set uq to contstant value in FOC.c for open loop control
	q31_rotorposition_absolute = 1 << 31;
	i16_hall_order = 1;//reset hall order
	MS.i_d_setpoint= 200; //set MS.id to appr. 2000mA
	MS.i_q_setpoint= 0;
//	uint8_t zerocrossing = 0;
//	q31_t diffangle = 0;
	HAL_Delay(5);
	for (i = 0; i < 1080; i++) {
		q31_rotorposition_absolute += 11930465; //drive motor in open loop with steps of 1�
		HAL_Delay(5);
		//printf_("%d, %d, %d, %d\n", temp3>>16,temp4>>16,temp5,temp6);

		if (ui8_hall_state_old != ui8_hall_state) {
			printf_("angle: %d, hallstate:  %d, hallcase %d \n",
					(int16_t) (((q31_rotorposition_absolute >> 23) * 180) >> 8),
					ui8_hall_state, ui8_hall_case);

			switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
			{
			//6 cases for forward direction
			case 64:
				Hall_64=q31_rotorposition_absolute;
				break;
			case 45:
				Hall_45=q31_rotorposition_absolute;
				break;
			case 51:
				Hall_51=q31_rotorposition_absolute;
				break;
			case 13:
				Hall_13=q31_rotorposition_absolute;
				break;
			case 32:
				Hall_32=q31_rotorposition_absolute;
				break;
			case 26:
				Hall_26=q31_rotorposition_absolute;
				break;

				//6 cases for reverse direction
			case 46:
				Hall_64=q31_rotorposition_absolute;
				break;
			case 62:
				Hall_26=q31_rotorposition_absolute;
				break;
			case 23:
				Hall_32=q31_rotorposition_absolute;
				break;
			case 31:
				Hall_13=q31_rotorposition_absolute;
				break;
			case 15:
				Hall_51=q31_rotorposition_absolute;
				break;
			case 54:
				Hall_45=q31_rotorposition_absolute;
				break;

			} // end case


			ui8_hall_state_old = ui8_hall_state;
		}
	}

   	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
    TIM1->CCR1 = 1023; //set initial PWM values
    TIM1->CCR2 = 1023;
    TIM1->CCR3 = 1023;
    MS.hall_angle_detect_flag=1;
    MS.i_d = 0;
    MS.i_q = 0;
    MS.u_d=0;
    MS.u_q=0;
    MS.i_d_setpoint= 0;
    q31_tics_filtered=1000000;

	HAL_FLASH_Unlock();

	if (i8_recent_rotor_direction == 1) {
		EE_WriteVariable(EEPROM_POS_HALL_ORDER, 1);
		i16_hall_order = 1;
	} else {
		EE_WriteVariable(EEPROM_POS_HALL_ORDER, -1);
		i16_hall_order = -1;
	}
	EE_WriteVariable(EEPROM_POS_HALL_45, Hall_45 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_51, Hall_51 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_13, Hall_13 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_32, Hall_32 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_26, Hall_26 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_64, Hall_64 >> 16);

	HAL_FLASH_Lock();

	MS.hall_angle_detect_flag = 1;
#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
	printf_("Motor specific angle:  %d, direction %d, zerocrossing %d \n ",
			(int16_t) (((q31_rotorposition_motor_specific >> 23) * 180) >> 8),
			i16_hall_order, zerocrossing);
#endif
	ui16_KV_detect_counter=HAL_GetTick();
	MS.KV_detect_flag=20;
	//HAL_Delay(5);


}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */


int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
 	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();

	//initialize MS struct.
	MS.hall_angle_detect_flag = 1;
	MS.KV_detect_flag=0;
	MS.Speed = 128000;
	MS.assist_level = 1;
	MS.regen_level = 7;
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;
	MS.angle_est=SPEED_PLL;

	MP.speed_limit = SPEEDLIMIT_NORMAL;

	MP.regen_current= REGEN_CURRENT/CAL_I;
	MP.phase_current_limit  =PH_CURRENT_MAX_NORMAL/CAL_I;

	  //init PI structs
	  PI_id.gain_i=I_FACTOR_I_D;
	  PI_id.gain_p=P_FACTOR_I_D;
	  PI_id.setpoint = 0;
	  PI_id.limit_output = _U_MAX;
	  PI_id.max_step=5000;
	  PI_id.shift=10;
	  PI_id.limit_i=1800;

	  PI_iq.gain_i=I_FACTOR_I_Q;
	  PI_iq.gain_p=P_FACTOR_I_Q;
	  PI_iq.setpoint = 0;
	  PI_iq.limit_output = _U_MAX;
	  PI_iq.max_step=5000;
	  PI_iq.shift=10;
	  PI_iq.limit_i=_U_MAX;

	calculate_tic_limits();

	//Virtual EEPROM init
	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

	MX_ADC1_Init();
	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		/* Calibration Error */
		Error_Handler();
	}
	MX_ADC2_Init();
	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
		/* Calibration Error */
		Error_Handler();
	}

	/* USER CODE BEGIN 2 */
	SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);

	//HAL_ADC_Start_IT(&hadc1);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adcData, 6);
	HAL_ADC_Start_IT(&hadc2);
	MX_TIM1_Init(); //Hier die Reihenfolge getauscht!
	MX_TIM2_Init();
	MX_TIM3_Init();

	// Start Timer 1
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
		/* Counter Enable Error */
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

	TIM1->CCR4 = TRIGGER_DEFAULT; //ADC sampling just before timer overflow (just before middle of PWM-Cycle)
//PWM Mode 1: Interrupt at counting down.

	//TIM1->BDTR |= 1L<<15;
	// TIM1->BDTR &= ~(1L<<15); //reset MOE (Main Output Enable) bit to disable PWM output
	// Start Timer 2
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		/* Counter Enable Error */
		Error_Handler();
	}

	// Start Timer 3
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		/* Counter Enable Error */
		Error_Handler();
	}

#if (DISPLAY_TYPE == DISPLAY_TYPE_M365DASHBOARD)
	M365Dashboard_init(huart1);
	PWR_init();

#endif

	TIM1->CCR1 = 1023; //set initial PWM values
	TIM1->CCR2 = 1023;
	TIM1->CCR3 = 1023;

	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM

	HAL_Delay(1000);

	for (i = 0; i < 16; i++) {
		while (!ui8_adc_regular_flag) {
		}
		ui16_ph1_offset += adcData[ADC_CHANA];
		ui16_ph2_offset += adcData[ADC_CHANB];
		ui16_ph3_offset += adcData[ADC_CHANC];
		ui8_adc_regular_flag = 0;

	}
	ui16_ph1_offset = ui16_ph1_offset >> 4;
	ui16_ph2_offset = ui16_ph2_offset >> 4;
	ui16_ph3_offset = ui16_ph3_offset >> 4;

	printf_("phase current offsets:  %d, %d, %d \n ", ui16_ph1_offset,
			ui16_ph2_offset, ui16_ph3_offset);

//while(1){}

        ADC1->JSQR=JSQR_PHASE_A; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
   	ADC1->JOFR1 = ui16_ph1_offset;

   	ui8_adc_offset_done_flag=1;

       // autodetect();

   	EE_ReadVariable(EEPROM_POS_HALL_ORDER, &i16_hall_order);
   	printf_("Hall_Order: %d \n",i16_hall_order);
   	// set varaiables to value from emulated EEPROM only if valid
   	if(i16_hall_order!=0xFFFF) {
   		int16_t temp;

   		EE_ReadVariable(EEPROM_POS_HALL_45, &temp);
   		Hall_45 = temp<<16;
   		printf_("Hall_45: %d \n",	(int16_t) (((Hall_45 >> 23) * 180) >> 8));

   		EE_ReadVariable(EEPROM_POS_HALL_51, &temp);
   		Hall_51 = temp<<16;
   		printf_("Hall_51: %d \n",	(int16_t) (((Hall_51 >> 23) * 180) >> 8));

   		EE_ReadVariable(EEPROM_POS_HALL_13, &temp);
   		Hall_13 = temp<<16;
   		printf_("Hall_13: %d \n",	(int16_t) (((Hall_13 >> 23) * 180) >> 8));

   		EE_ReadVariable(EEPROM_POS_HALL_32, &temp);
   		Hall_32 = temp<<16;
   		printf_("Hall_32: %d \n",	(int16_t) (((Hall_32 >> 23) * 180) >> 8));

   		EE_ReadVariable(EEPROM_POS_HALL_26, &temp);
   		Hall_26 = temp<<16;
   		printf_("Hall_26: %d \n",	(int16_t) (((Hall_26 >> 23) * 180) >> 8));

   		EE_ReadVariable(EEPROM_POS_HALL_64, &temp);
  		Hall_64 = temp<<16;
  		printf_("Hall_64: %d \n",	(int16_t) (((Hall_64 >> 23) * 180) >> 8));

  		EE_ReadVariable(EEPROM_POS_KV, &ui32_KV);
  		if(!ui32_KV)ui32_KV=111;
  		printf_("KV: %d \n",ui32_KV	);

   	}else{
                autodetect();
        }


#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
	printf_("Lishui FOC v0.9 \n ");
#endif

	HAL_Delay(5);
	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM

	get_standstill_position();



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		//display message processing
#if (DISPLAY_TYPE == DISPLAY_TYPE_M365DASHBOARD)
		search_DashboardMessage(&MS, &MP, huart1);
		checkButton(&MP, &MS);
#endif

#if (defined(FAST_LOOP_LOG))
		if(ui8_UART_TxCplt_flag&&ui8_debug_state==3){
	        sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n", e_log[k][0], e_log[k][1], e_log[k][2],e_log[k][3],e_log[k][4],e_log[k][5]); //>>24
			i=0;
			while (buffer[i] != '\0')
			{i++;}
			ui8_UART_TxCplt_flag=0;
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&buffer, i);
			k++;
			if (k>299){
				k=0;
				ui8_debug_state=4;
				//Obs_flag=0;
			}
		}
#endif


#ifdef ADCTHROTTLE

		MS.i_q_setpoint=map(ui16_reg_adc_value,THROTTLEOFFSET,THROTTLEMAX,0,MP.phase_current_limit);
#endif
		// set power to zero at low voltage
		if(MS.Voltage<BATTERYVOLTAGE_MIN){
			MS.i_q_setpoint= 0;
			MS.i_d_setpoint= 0;
			MS.error_state= lowbattery;
		}
		else{ //calculate setpoints for iq and id
			if(MS.error_state == lowbattery)MS.error_state= none;
		if (MS.i_q_setpoint_temp > MP.phase_current_limit)
			MS.i_q_setpoint_temp = MP.phase_current_limit;
		if (MS.i_q_setpoint_temp < -MP.phase_current_limit)
			MS.i_q_setpoint_temp = -MP.phase_current_limit;

		MS.i_q_setpoint_temp = map(q31_tics_filtered >> 3, tics_higher_limit,
				tics_lower_limit, 0, MS.i_q_setpoint_temp); //ramp down current at speed limit

		if((MS.mode&0x07)==sport){//do flux weakaning
					MS.i_d_setpoint_temp=-map(MS.Speed,(ui32_KV*MS.Voltage/100000)-8,(ui32_KV*MS.Voltage/100000)+30,0,fw_current_max);
				}
		else MS.i_d_setpoint_temp=0;

		//Check and limit absolute value of current vector

		arm_sqrt_q31((MS.i_q_setpoint_temp*MS.i_q_setpoint_temp+MS.i_d_setpoint_temp*MS.i_d_setpoint_temp)<<1,&MS.i_setpoint_abs);
		MS.i_setpoint_abs = (MS.i_setpoint_abs>>16)+1;

		if(MS.hall_angle_detect_flag){ //run only, if autodetect is not active
			if (MS.i_setpoint_abs > MP.phase_current_limit) {
				MS.i_q_setpoint = i8_direction* (MS.i_q_setpoint_temp * MP.phase_current_limit) / MS.i_setpoint_abs; //division!
				MS.i_d_setpoint = (MS.i_d_setpoint_temp * MP.phase_current_limit) / MS.i_setpoint_abs; //division!
				MS.i_setpoint_abs = MP.phase_current_limit;
			} else {
				MS.i_q_setpoint= i8_direction*MS.i_q_setpoint_temp;
				MS.i_d_setpoint= MS.i_d_setpoint_temp;
			}
		}



		  if(MS.KV_detect_flag){
			static  int8_t dir=1;
			static  uint16_t KVtemp;
			MS.i_q_setpoint=1;

			MS.angle_est=0;//switch to angle extrapolation
			  if(((MS.Voltage*MS.u_q)>>(21-SPEEDFILTER))){
					  ui32_KV -=ui32_KV>>4;
					  ui32_KV += (uint32_SPEEDx100_cumulated)/((MS.Voltage*MS.u_q)>>(21-SPEEDFILTER)); //unit: kph*100/V
				  }
			  if(ui16_KV_detect_counter>200){
				  MS.KV_detect_flag +=10*dir;
				  printf_("KV_cumulated= %d,%d\n",ui32_KV>>4,MS.u_q);
				  ui16_KV_detect_counter=0;

			  }
			  if(MS.u_q>1900){
				  KVtemp=ui32_KV>>4;
			      dir=-1;
			  }

			  if(MS.KV_detect_flag<20){
				  dir=1;
				  MS.i_q_setpoint=0;
				  ui32_KV=KVtemp;

				  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM
				  MS.angle_est=SPEED_PLL;//switch back to config setting
				  MS.KV_detect_flag=0;

				  printf_("KV detection finished!%d\n",KVtemp);
			  	  HAL_FLASH_Unlock();
			      	  EE_WriteVariable(EEPROM_POS_KV, (int16_t) (KVtemp));
			      HAL_FLASH_Lock();
			  }
			  if(abs(MS.i_q>300)){

				  MS.i_q_setpoint=0;
				  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM
				  MS.KV_detect_flag=0;
				  MS.angle_est=SPEED_PLL;//switch back to config setting
				  printf_("KV_detection aborted due to overcurrent\n");

			  }

		  }//end KV detect
		}// end undervoltage check

			uint32_SPEEDx100_cumulated -=uint32_SPEEDx100_cumulated>>SPEEDFILTER;
			uint32_SPEEDx100_cumulated +=internal_tics_to_speedx100(q31_tics_filtered>>3);

		  //calculate battery current

			iq_cum-=iq_cum>>8;
			iq_cum+=MS.i_q;

			id_cum-=id_cum>>8;
			id_cum+=MS.i_d;

			uq_cum-=uq_cum>>8;
			uq_cum+=MS.u_q;

			ud_cum-=ud_cum>>8;
			ud_cum+=MS.u_d;




			 MS.Battery_Current=get_battery_current(iq_cum>>8,id_cum>>8,uq_cum>>8,ud_cum>>8)*sign(iq_cum) * i8_direction * i8_reverse_flag;

			 // enable PWM if power is wanted and speed is lower than idle speed
		if (MS.i_q_setpoint && !READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)&&(uint32_SPEEDx100_cumulated>>SPEEDFILTER)*1000<(ui32_KV*MS.Voltage)) {

			TIM1->CCR1 = 1023; //set initial PWM values
			TIM1->CCR2 = 1023;
			TIM1->CCR3 = 1023;


			uint16_half_rotation_counter = 0;
			uint16_full_rotation_counter = 0;
			__HAL_TIM_SET_COUNTER(&htim2, 0); //reset tim2 counter
			ui16_timertics = 40000; //set interval between two hallevents to a large value
			i8_recent_rotor_direction = i8_direction * i8_reverse_flag;
			SET_BIT(TIM1->BDTR, TIM_BDTR_MOE); //enable PWM if power is wanted
			ui8_debug_state=0;//Start fast logging
		    if(MS.system_state == Stop)speed_PLL(0,0);//reset integral part
		    else {
		    	PI_iq.integral_part = (((uint32_SPEEDx100_cumulated>>SPEEDFILTER)<<11)*1000/(ui32_KV*MS.Voltage))<<PI_iq.shift;//((((uint32_SPEEDx100_cumulated*_T))/(MS.Voltage*ui32_KV))<<4)
		    	PI_iq.out=PI_iq.integral_part;
		    }

			get_standstill_position();


		} else {
#ifdef KILL_ON_ZERO
                  if(uint16_mapped_throttle==0&&READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
			  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
			  get_standstill_position();
                          printf_("shutdown %d\n", q31_rotorposition_absolute);
                  }
#endif
		}


		if (i8_slow_loop_flag) {
       i8_slow_loop_flag = 0;

      // low pass filter measured battery voltage 
      static q31_t q31_batt_voltage_acc = 0;
      q31_batt_voltage_acc -= (q31_batt_voltage_acc >> 7);
      q31_batt_voltage_acc += adcData[ADC_VOLTAGE];
      q31_Battery_Voltage = (q31_batt_voltage_acc >> 7) * CAL_BAT_V;
		}

		//slow loop procedere @16Hz, for LEV standard every 4th loop run, send page,
		if (ui32_tim3_counter > 500) {
			if(MS.shutdown)MS.shutdown++;

			MS.Temperature = adcData[ADC_TEMP] * 41 >> 8; //0.16 is calibration constant: Analog_in[10mV/°C]/ADC value. Depending on the sensor LM35)
			MS.Voltage = q31_Battery_Voltage;

			printf_("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", MS.system_state,i16_hall_order,i8_recent_rotor_direction,ui8_hall_case,q31_angle_per_tic>>8,uq_cum>>8,ud_cum>>8,iq_cum>>8 , id_cum>>8);

			MS.Speed=tics_to_speed(q31_tics_filtered>>3);

			if (!MS.i_q_setpoint&&(uint16_full_rotation_counter > 7999
					|| uint16_half_rotation_counter > 7999)) {
				if(READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
					CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
					ui8_debug_state=4;//Reset fast loop logging
					get_standstill_position();

				}
				MS.Speed=0;
				MS.system_state=Stop;
			}



#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG && !defined(FAST_LOOP_LOG))
                //Jon Pry uses this crazy string for automated data collection
	  	//	sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, T: %d, Q: %d, D: %d, %d, %d, S: %d, %d, V: %d, C: %d\r\n", int32_current_target, (int16_t) raw_inj1,(int16_t) raw_inj2, (int32_t) MS.char_dyn_adc_state, q31_rotorposition_hall, q31_rotorposition_absolute, (int16_t) (ui16_reg_adc_value),adcData[ADC_CHANA],adcData[ADC_CHANB],adcData[ADC_CHANC],i16_ph1_current,i16_ph2_current, uint16_mapped_throttle, MS.i_q, MS.i_d, MS.u_q, MS.u_d,q31_tics_filtered>>3,tics_higher_limit, adcData[ADC_VOLTAGE], MS.Battery_Current);
	  		sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", MS.i_q_setpoint, MS.i_q,q31_tics_filtered>>3, adcData[ADC_THROTTLE],MS.i_q_setpoint, MS.u_d, MS.u_q , MS.u_abs,  MS.Battery_Current);
	  	//	sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n",(uint16_t)adcData[0],(uint16_t)adcData[1],(uint16_t)adcData[2],(uint16_t)adcData[3],(uint16_t)(adcData[4]),(uint16_t)(adcData[5])) ;

	  	  i=0;
		  while (buffer[i] != '\0')
		  {i++;}
		 HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&buffer, i);
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		 ui8_print_flag = 0;

#endif

			ui32_tim3_counter = 0;
		}	  	// end of slow loop

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	ADC_MultiModeTypeDef multimode;
	ADC_InjectionConfTypeDef sConfigInjected;
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; //Scan muß für getriggerte Wandlung gesetzt sein
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; // Trigger regular ADC with timer 3 ADC_EXTERNALTRIGCONV_T1_CC1;// // ADC_SOFTWARE_START; //
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 6;
	hadc1.Init.NbrOfDiscConversion = 0;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // Hier bin ich nicht sicher ob Trigger out oder direkt CC4
	sConfigInjected.AutoInjectedConv = DISABLE; //muß aus sein
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0; //ui16_ph1_offset;//1900;
	HAL_ADC_Stop(&hadc1); //ADC muß gestoppt sein, damit Triggerquelle gesetzt werden kann.
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; //ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; //ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; //ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = JSQR_PHASE_A >> 15;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; //ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = JSQR_PHASE_B >> 15;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; //ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig.Channel = JSQR_PHASE_C >> 15;
	sConfig.Rank = ADC_REGULAR_RANK_6;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; //ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	ADC_InjectionConfTypeDef sConfigInjected;

	/**Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE; //hier auch Scan enable?!
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0; //ui16_ph2_offset;//	1860;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = _T;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; //TODO: depends on gate driver!
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = _T - 1;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 32;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 128;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 64000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function 8kHz interrupt frequency for regular adc triggering */
static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 7813;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;

	huart3.Init.BaudRate = 115200;

	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	  /*Configure GPIO pin : PWR_BTN_Pin */
	  GPIO_InitStruct.Pin = PWR_BTN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(PWR_BTN_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : TPS_ENA_Pin */
	  GPIO_InitStruct.Pin = TPS_ENA_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(TPS_ENA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

//	HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : UART1Tx_Pin */
	GPIO_InitStruct.Pin = UART1_Tx_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(UART1_Tx_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(BrakeLight_GPIO_Port, BrakeLight_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : BrakeLight_Pin */
	GPIO_InitStruct.Pin = BrakeLight_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BrakeLight_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(HALL_1_GPIO_Port, HALL_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : HALL_1_Pin HALL_2_Pin */
	GPIO_InitStruct.Pin = HALL_1_Pin | HALL_2_Pin | HALL_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if 0
  GPIO_InitStruct.Pin = HALL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HALL_1_GPIO_Port, &GPIO_InitStruct);

  while(1){
    HAL_GPIO_WritePin(HALL_1_GPIO_Port, HALL_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HALL_1_GPIO_Port, HALL_1_Pin, GPIO_PIN_SET);
  }
#endif

	/*Configure peripheral I/O remapping */
	__HAL_AFIO_REMAP_PD01_ENABLE();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {

		if(MS.angle_est){
		//keep q31_rotorposition_PLL updated when PWM is off
		   if(!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))q31_rotorposition_PLL += (q31_angle_per_tic<<1);
		}
		if(MS.KV_detect_flag)ui16_KV_detect_counter++;

		i8_slow_loop_flag = 1;
		if (ui32_tim3_counter < 32000)
			ui32_tim3_counter++;
		if (uint32_SPEED_counter < 128000) {
			uint32_SPEED_counter++;
		}
		if (uint16_full_rotation_counter < 8000)
			uint16_full_rotation_counter++;	//full rotation counter for motor standstill detection
		if (uint16_half_rotation_counter < 8000)
			uint16_half_rotation_counter++;	//half rotation counter for motor standstill detectio
	}
}

// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ui32_reg_adc_value_filter -= ui32_reg_adc_value_filter >> 4;
	ui32_reg_adc_value_filter += adcData[ADC_THROTTLE]; //HAL_ADC_GetValue(hadc);
	ui16_reg_adc_value = ui32_reg_adc_value_filter >> 4;

	ui8_adc_regular_flag = 1;
}

//injected ADC

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//for oszi-check of used time in FOC procedere
	//HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_SET);
	ui32_tim1_counter++;

	/*  else {
	 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	 uint32_SPEED_counter=0;
	 }*/

	if (!ui8_adc_offset_done_flag) {
		i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1,
				ADC_INJECTED_RANK_1);
		i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2,
				ADC_INJECTED_RANK_1);

		ui8_adc_inj_flag = 1;
	} else {

#ifdef DISABLE_DYNAMIC_ADC

		i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);


#else
		switch (MS.char_dyn_adc_state) //read in according to state
		{
		case 1: //Phase C at high dutycycles, read from A+B directly
		{
			raw_inj1 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc1,
					ADC_INJECTED_RANK_1);
			i16_ph1_current = raw_inj1;

			raw_inj2 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc2,
					ADC_INJECTED_RANK_1);
			i16_ph2_current = raw_inj2;
		}
			break;
		case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
		{

			raw_inj2 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc2,
					ADC_INJECTED_RANK_1);
			i16_ph2_current = raw_inj2;

			raw_inj1 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc1,
					ADC_INJECTED_RANK_1);
			i16_ph1_current = -i16_ph2_current - raw_inj1;

		}
			break;
		case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
		{
			raw_inj1 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc1,
					ADC_INJECTED_RANK_1);
			i16_ph1_current = raw_inj1;
			raw_inj2 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc2,
					ADC_INJECTED_RANK_1);
			i16_ph2_current = -i16_ph1_current - raw_inj2;
		}
			break;

		case 0: //timeslot too small for ADC
		{
			//do nothing
		}
			break;

		} // end case
#endif

		__disable_irq(); //ENTER CRITICAL SECTION!!!!!!!!!!!!!

		//extrapolate recent rotor position
		ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event
		if (MS.hall_angle_detect_flag) {
			if(ui16_timertics<SIXSTEPTHRESHOLD && ui16_tim2_recent<200)ui8_6step_flag=0;
			if(ui16_timertics>(SIXSTEPTHRESHOLD*6)>>2)ui8_6step_flag=1;


			if(MS.angle_est){
				q31_rotorposition_PLL += q31_angle_per_tic;
			}
			if (ui16_tim2_recent < ui16_timertics+(ui16_timertics>>2) && !ui8_overflow_flag && !ui8_6step_flag) { //prevent angle running away at standstill
				if(MS.angle_est&&abs(q31_PLL_error)<deg_30){
					q31_rotorposition_absolute=q31_rotorposition_PLL;
					MS.system_state=PLL;
				}
			else{
				q31_rotorposition_absolute = q31_rotorposition_hall
						+ (q31_t) (i8_recent_rotor_direction
								* ((10923 * ui16_tim2_recent) / ui16_timertics)
								<< 16); //interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60�
				MS.system_state=Interpolation;
				}
			} else {
				ui8_overflow_flag = 1;
				if(MS.KV_detect_flag)q31_rotorposition_absolute = q31_rotorposition_hall;
				else q31_rotorposition_absolute = q31_rotorposition_hall+i8_direction*deg_30;//offset of 30 degree to get the middle of the sector
				MS.system_state=SixStep;
					//	}

			}
		} //end if hall angle detect
		//temp2=(((q31_rotorposition_absolute >> 23) * 180) >> 8);
		__enable_irq(); //EXIT CRITICAL SECTION!!!!!!!!!!!!!!

#ifndef DISABLE_DYNAMIC_ADC

		//get the Phase with highest duty cycle for dynamic phase current reading
		dyn_adc_state(q31_rotorposition_absolute);
		//set the according injected channels to read current at Low-Side active time

		if (MS.char_dyn_adc_state != char_dyn_adc_state_old) {
			set_inj_channel(MS.char_dyn_adc_state);
			char_dyn_adc_state_old = MS.char_dyn_adc_state;
		}
#endif

		//int32_current_target=0;
		// call FOC procedure if PWM is enabled

		if (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) {
			FOC_calculation(i16_ph1_current, i16_ph2_current,
					q31_rotorposition_absolute, &MS);
		}
		//temp5=__HAL_TIM_GET_COUNTER(&htim1);
		//set PWM

		TIM1->CCR1 = (uint16_t) switchtime[0];
		TIM1->CCR2 = (uint16_t) switchtime[1];
		TIM1->CCR3 = (uint16_t) switchtime[2];

		//HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_RESET);

	} // end else

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//Hall sensor event processing
	if (GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_5
			|| GPIO_Pin == GPIO_PIN_0) //check for right interrupt source
	{
		ui8_hall_state = ((GPIOB->IDR & 1) << 2) | ((GPIOB->IDR >> 4) & 0b11); //Mask input register with Hall 1 - 3 bits

		if (ui8_hall_state == ui8_hall_state_old)
			return;

		ui8_hall_case = ui8_hall_state_old * 10 + ui8_hall_state;
		if (MS.hall_angle_detect_flag) { //only process, if autodetect procedere is fininshed
			ui8_hall_state_old = ui8_hall_state;
		}

		ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last hall event

		if (ui16_tim2_recent > 100) { //debounce
			ui16_timertics = ui16_tim2_recent; //save timertics since last hall event
			q31_tics_filtered -= q31_tics_filtered >> 3;
			q31_tics_filtered += ui16_timertics;
			__HAL_TIM_SET_COUNTER(&htim2, 0); //reset tim2 counter
			ui8_overflow_flag = 0;

		}
		if(ui16_timertics<(SIXSTEPTHRESHOLD<<2)&&!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) MS.system_state=IdleRun;
		switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
		{
		//6 cases for forward direction
		case 64:
			q31_rotorposition_hall = Hall_64;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;
		case 45:
			q31_rotorposition_hall = Hall_45;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 51:
			q31_rotorposition_hall = Hall_51;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 13:
			q31_rotorposition_hall = Hall_13;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 32:
			q31_rotorposition_hall = Hall_32;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 26:
			q31_rotorposition_hall = Hall_26;

			i8_recent_rotor_direction = -i16_hall_order;
			break;

			//6 cases for reverse direction
		case 46:
			q31_rotorposition_hall = Hall_64;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 62:
			q31_rotorposition_hall = Hall_26;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 23:
			q31_rotorposition_hall = Hall_32;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 31:
			q31_rotorposition_hall = Hall_13;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 15:
			q31_rotorposition_hall = Hall_51;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 54:
			q31_rotorposition_hall = Hall_45;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;

		} // end case

		if(MS.angle_est){
			q31_PLL_error=q31_rotorposition_PLL-q31_rotorposition_hall;
			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall);
		}

	} //end if
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	ui8_UART_TxCplt_flag = 1;

	if(UartHandle==&huart1)	HAL_HalfDuplex_EnableReceiver(&huart1);

}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	printf_("Lishui FOC v0.9 \n ");
}
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {

}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

	// map the input to the output range.
	// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1)
				+ out_min;
	// round down if mapping smaller ranges to bigger ranges
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//assuming, a proper AD conversion takes 350 timer tics, to be confirmed. DT+TR+TS deadtime + noise subsiding + sample time
void dyn_adc_state(q31_t angle) {
	if (switchtime[2] > switchtime[0] && switchtime[2] > switchtime[1]) {
		MS.char_dyn_adc_state = 1; // -90° .. +30°: Phase C at high dutycycles
		if (switchtime[2] > 1500)
			TIM1->CCR4 = switchtime[2] - TRIGGER_OFFSET_ADC;
		else
			TIM1->CCR4 = TRIGGER_DEFAULT;
	}

	if (switchtime[0] > switchtime[1] && switchtime[0] > switchtime[2]) {
		MS.char_dyn_adc_state = 2; // +30° .. 150° Phase A at high dutycycles
		if (switchtime[0] > 1500)
			TIM1->CCR4 = switchtime[0] - TRIGGER_OFFSET_ADC;
		else
			TIM1->CCR4 = TRIGGER_DEFAULT;
	}

	if (switchtime[1] > switchtime[0] && switchtime[1] > switchtime[2]) {
		MS.char_dyn_adc_state = 3; // +150 .. -90° Phase B at high dutycycles
		if (switchtime[1] > 1500)
			TIM1->CCR4 = switchtime[1] - TRIGGER_OFFSET_ADC;
		else
			TIM1->CCR4 = TRIGGER_DEFAULT;
	}
}

static void set_inj_channel(char state) {
	switch (state) {
	case 1: //Phase C at high dutycycles, read current from phase A + B
	{
		ADC1->JSQR = JSQR_PHASE_A; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
		ADC1->JOFR1 = ui16_ph1_offset;
		ADC2->JSQR = JSQR_PHASE_B; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
		ADC2->JOFR1 = ui16_ph2_offset;

	}
		break;
	case 2: //Phase A at high dutycycles, read current from phase C + B
	{
		ADC1->JSQR = JSQR_PHASE_C; //ADC1 injected reads phase C, JSQ4 = 0b00110, decimal 6
		ADC1->JOFR1 = ui16_ph3_offset;
		ADC2->JSQR = JSQR_PHASE_B; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
		ADC2->JOFR1 = ui16_ph2_offset;

	}
		break;

	case 3: //Phase B at high dutycycles, read current from phase A + C
	{
		ADC1->JSQR = JSQR_PHASE_A; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
		ADC1->JOFR1 = ui16_ph1_offset;
		ADC2->JSQR = JSQR_PHASE_C; //ADC2 injected reads phase C, JSQ4 = 0b00110, decimal 6
		ADC2->JOFR1 = ui16_ph3_offset;

	}
		break;

	}

}

void get_standstill_position() {
	HAL_Delay(100);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_4); //read in initial rotor position

	switch (ui8_hall_state) {
	//6 cases for forward direction
	case 2:
		q31_rotorposition_hall = Hall_32;
		break;
	case 6:
		q31_rotorposition_hall = Hall_26;
		break;
	case 4:
		q31_rotorposition_hall = Hall_64;
		break;
	case 5:
		q31_rotorposition_hall = Hall_45;
		break;
	case 1:
		q31_rotorposition_hall = Hall_51;

		break;
	case 3:
		q31_rotorposition_hall = Hall_13;
		break;

	}

	q31_rotorposition_absolute = q31_rotorposition_hall;



}

void runPIcontrol(){
	//PI-control processing
			if (PI_flag) {
				//Check battery current limit
				if (MS.Battery_Current > BATTERYCURRENT_MAX)
					ui8_BC_limit_flag = 1;
				if (MS.Battery_Current < -REGEN_CURRENT_MAX)
					ui8_BC_limit_flag = 1;
				//reset battery current flag with small hysteresis
				if (MS.i_q * i8_direction * i8_reverse_flag > 100) { //motor mode
					if (get_battery_current(MS.i_q_setpoint,MS.i_d_setpoint,uq_cum>>8,ud_cum>>8)
							<  (BATTERYCURRENT_MAX * 7) >> 3)
						ui8_BC_limit_flag = 0;
				} else { //generator mode
					if (get_battery_current(MS.i_q_setpoint,MS.i_d_setpoint,uq_cum>>8,ud_cum>>8)
							> (-REGEN_CURRENT_MAX * 7) >> 3) // Battery current not negative yet!!!!
						ui8_BC_limit_flag = 0;
				}

				//control iq

//				//if
//				if (!ui8_BC_limit_flag) {
//					q31_u_q_temp = PI_control_i_q(MS.i_q,
//							(q31_t) i8_direction * i8_reverse_flag
//									* MS.i_q_setpoint);
//				} else {
//					if (MS.i_q * i8_direction * i8_reverse_flag > 100) { //motor mode
//						q31_u_q_temp = PI_control_i_q(
//								(MS.Battery_Current >> 6) * i8_direction
//										* i8_reverse_flag,
//								(q31_t) (BATTERYCURRENT_MAX >> 6) * i8_direction
//										* i8_reverse_flag);
//					} else { //generator mode
//						q31_u_q_temp = PI_control_i_q(
//								(MS.Battery_Current >> 6) * i8_direction
//										* i8_reverse_flag,
//								(q31_t) (-REGEN_CURRENT_MAX >> 6) * i8_direction
//										* i8_reverse_flag);
//					}
//				}

				  if (!ui8_BC_limit_flag){
					  PI_iq.recent_value = MS.i_q;
					  PI_iq.setpoint = MS.i_q_setpoint;
				  }
				  else{
					  if(!MS.brake_active){
					 // if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
						  PI_iq.recent_value=  (MS.Battery_Current>>6)*i8_direction*i8_reverse_flag;
						  PI_iq.setpoint = (BATTERYCURRENT_MAX>>6)*i8_direction*i8_reverse_flag;
					  	}
					  else{
						  PI_iq.recent_value=  (MS.Battery_Current>>6)*i8_direction*i8_reverse_flag;
						  PI_iq.setpoint = (-REGEN_CURRENT_MAX>>6)*i8_direction*i8_reverse_flag;
					    }
				  }
				  q31_u_q_temp =  PI_control(&PI_iq);

				//Control id
				//q31_u_d_temp = -PI_control_i_d(MS.i_d, MS.i_d_setpoint,	abs(q31_u_q_temp / MAX_D_FACTOR)); //control direct current to recent setpoint
				  PI_id.recent_value = MS.i_d;
				  PI_id.setpoint = MS.i_d_setpoint;
				  q31_u_d_temp = -PI_control(&PI_id); //control direct current to zero


				arm_sqrt_q31((q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp)<<1,&MS.u_abs);
				MS.u_abs = (MS.u_abs>>16)+1;


				if (MS.u_abs > _U_MAX) {
					MS.u_q = (q31_u_q_temp * _U_MAX) / MS.u_abs; //division!
					MS.u_d = (q31_u_d_temp * _U_MAX) / MS.u_abs; //division!
					MS.u_abs = _U_MAX;
				} else {
					MS.u_q = q31_u_q_temp;
					MS.u_d = q31_u_d_temp;
				}
				PI_flag = 0;
				//HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_RESET);
			}
}

int32_t speed_to_tics(uint8_t speed) {
	return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * speed * 10);
}

int8_t tics_to_speed(uint32_t tics) {
	return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * tics * 10);;
}

int16_t internal_tics_to_speedx100 (uint32_t tics){
	return WHEEL_CIRCUMFERENCE*50*3600/(6*GEAR_RATIO*tics);;
}

void calculate_tic_limits(void){
	tics_lower_limit = WHEEL_CIRCUMFERENCE * 5 * 3600
			/ (6 * GEAR_RATIO * MP.speed_limit * 10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
	tics_higher_limit = WHEEL_CIRCUMFERENCE * 5 * 3600
			/ (6 * GEAR_RATIO * (MP.speed_limit + 2) * 10);

}

q31_t speed_PLL(q31_t actual, q31_t target) {
  static q31_t q31_d_i = 0;



  q31_t delta = target - actual;
  q31_t q31_p = (delta >> P_FACTOR_PLL);   	//7 for Shengyi middrive, 10 for BionX IGH3
  q31_d_i += (delta >> I_FACTOR_PLL);				//11 for Shengyi middrive, 10 for BionX IGH3

  if (q31_d_i>((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i = ((deg_30>>18)*500/ui16_timertics)<<16;
  if (q31_d_i<-((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i =- ((deg_30>>18)*500/ui16_timertics)<<16;

  q31_t q31_d_dc = q31_p + q31_d_i;

  if (!actual&&!target)q31_d_i=0;


  return q31_d_dc;
}
q31_t get_battery_current(q31_t iq,q31_t id,q31_t uq,q31_t ud){
	q31_t ibatq;
	q31_t ibatd;
	ibatq=(iq * uq *CAL_I) >> 11;
	ibatd=(id * ud *CAL_I) >> 11;

	return abs(ibatd)+abs(ibatq);
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM
	}
	/* USER CODE END Error_Handler_Debug */
}

void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
