#define THROTTLEOFFSET 45
#define THROTTLEMAX 175
#define BRAKEOFFSET 50
#define BRAKEMAX 100

// speed limits for invividual modes in kph
#define SPEEDLIMIT_ECO 10
#define SPEEDLIMIT_NORMAL 20
#define SPEEDLIMIT_SPORT 50

// motor current limits for invividual modes in mA
// note that hacked firmware allows up to 55amps motor phase current
#define PH_CURRENT_MAX_ECO 10000
#define PH_CURRENT_MAX_NORMAL 15000
#define PH_CURRENT_MAX_SPORT 25000

// motor current limit for regen in mA
#define REGEN_MAX_CURRENT 10000

// maximum current for field weakening in mA
#define FIELD_WEAKNING_CURRENT_MAX 0 //max id

// ADC channels
#define ADC_VOLTAGE 0
#define ADC_THROTTLE 1
#define ADC_TEMP 2

#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOD
#define UART1_Tx_Pin GPIO_PIN_6
#define UART1_Tx_GPIO_Port GPIOB
#define BrakeLight_Pin GPIO_PIN_15
#define BrakeLight_GPIO_Port GPIOA
#define Temp_Pin GPIO_PIN_0
#define Temp_GPIO_Port GPIOA
#define Throttle_Pin GPIO_PIN_1
#define Throttle_GPIO_Port GPIOA
#define Batt_Voltage_Pin GPIO_PIN_2
#define Batt_Voltage_GPIO_Port GPIOA
#define PWR_BTN_Pin GPIO_PIN_14
#define PWR_BTN_GPIO_Port GPIOC
#define TPS_ENA_Pin GPIO_PIN_15
#define TPS_ENA_GPIO_Port GPIOC

