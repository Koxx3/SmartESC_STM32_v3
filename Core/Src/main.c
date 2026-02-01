#include "main.h"
#include "config.h"
#include "print.h"
#include "motor.h"
#include "button_processing.h"
#include "M365_Dashboard.h"
#include "vesc_uart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

M365State_t M365State;
MotorStatePublic_t MSPublic;

volatile uint32_t systick_cnt = 0;

// every 1ms
void UserSysTickHandler(void) {
  static uint32_t c;  
  
  systick_cnt++;

  c++;
  // every 10ms
  if ((c % 10) == 0) {
    motor_slow_loop(&MSPublic);
  }
}

/**
 * Enable DMA controller clock
 */
static void DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

  // DMA channel 3: used for USART3_RX
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  // DMA channel 4: used for USART1_TX
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  // DMA channel 5: used for USART1_RX
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void USART1_UART_Init(void) {

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
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void USART3_UART_Init(void) {

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void GPIO_Init(void) {
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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_SET);

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

	/*Configure GPIO pin : UART1Tx_Pin */
	GPIO_InitStruct.Pin = UART1_Tx_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_WritePin(BrakeLight_GPIO_Port, BrakeLight_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : BrakeLight_Pin */
	GPIO_InitStruct.Pin = BrakeLight_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BrakeLight_GPIO_Port, &GPIO_InitStruct);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {

	if (UartHandle==&huart1) {
    HAL_HalfDuplex_EnableReceiver(&huart1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
    motor_disable_pwm();
  }
}

void _Error_Handler(char *file, int line) {
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
    motor_disable_pwm();
	}
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

int main(void) {
	// Reset of all peripherals, Initializes the Flash interface and the Systick
	HAL_Init();

	// Configure the system clock
  // board do not have any external crystal and so we use internal clock. Final clock is 64MHz
	SystemClock_Config();

  // init GPIOS
	GPIO_Init();

  // init DMA for ADC, USART_1 and USART_3
	DMA_Init();

  // init USART_1 and USART_3
	USART1_UART_Init();
	USART3_UART_Init();

  MSPublic.brake_active = true; // force motor stop until receive real value from brake
  MSPublic.i_q_setpoint_target = 0; // start at 0 until throttle value is readed
  MSPublic.speed = 128000;
	MSPublic.speed_limit = SPEEDLIMIT_NORMAL;
  MSPublic.phase_current_limit = PH_CURRENT_MAX_NORMAL;
  MSPublic.field_weakening_current_max = FIELD_WEAKNING_CURRENT_MAX;
  MSPublic.battery_voltage_min = BATTERYVOLTAGE_MIN;

  motor_init(&MSPublic);

  M365State.phase_current_limit = PH_CURRENT_MAX_NORMAL;
  M365State.speed_limit = SPEEDLIMIT_NORMAL;
  M365State.regen_max_current = REGEN_MAX_CURRENT;

  // init dashboard
	M365Dashboard_init(huart1);
	VESC_UART_Init(&huart3);
	PWR_init();

	while (1) {

    // update M365State vars that are calculated on motor_slow_loop
    M365State.speed = MSPublic.speed;

		// search and process display message
		search_DashboardMessage(&M365State, huart1);
		VESC_UART_Process(&MSPublic);
    // update vars to MSPublic
    MSPublic.phase_current_limit = M365State.phase_current_limit;
    MSPublic.i_q_setpoint_target = M365State.i_q_setpoint_target;
    MSPublic.brake_active = M365State.brake_active;

		//slow loop process, every 20ms
    static uint32_t systick_cnt_old = 0;
		if ((systick_cnt_old != systick_cnt) && // only at a change
        (systick_cnt % 20) == 0) { // every 20ms
      systick_cnt_old = systick_cnt;

      // process buttons
		  checkButton(&M365State);
      // update vars to MSPublic
      MSPublic.mode = M365State.mode;
      MSPublic.speed_limit = M365State.speed_limit;

      // battery voltage
      // low pass filter measured battery voltage 
      static q31_t q31_batt_voltage_acc = 0;
      q31_batt_voltage_acc -= (q31_batt_voltage_acc >> 7);
      q31_batt_voltage_acc += MSPublic.adcData[ADC_VOLTAGE];
      q31_t q31_battery_voltage = (q31_batt_voltage_acc >> 7) * CAL_BAT_V;
      // update vars to MSPublic
			MSPublic.battery_voltage = M365State.battery_voltage = q31_battery_voltage;

      // increase shutdown counter
			if (M365State.shutdown) M365State.shutdown++;

      // temperature
			M365State.temperature = (MSPublic.adcData[ADC_TEMP] * 41) >> 8; //0.16 is calibration constant: Analog_in[10mV/°C]/ADC value. Depending on the sensor LM35)

      // DEBUG
      /*
      static uint8_t debug_cnt = 0;
      if (++debug_cnt > 13) { // every 13 * 20 ms = 260ms
        debug_cnt = 0;
        printf_("%d, %d\n", MSPublic.debug[0], MSPublic.debug[1] * CAL_I);
      }
      */
		}
	}
}
