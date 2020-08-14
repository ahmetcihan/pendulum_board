#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "max11254.h"

ControlState ControlExti_Max_1_RdbyPin 	= ControlState_CHECKED,
			 ControlExti_Max_2_RdbyPin 	= ControlState_CHECKED,
			 ControlExti_Max_3_RdbyPin 	= ControlState_CHECKED,
			 ControlExti_Max_4_RdbyPin 	= ControlState_CHECKED,
			 ControlUsart1_ReceiveData 	= ControlState_CHECKED,
			 ControlUsart1_TransmitData = ControlState_CHECKED,
			 ControlTIM4_10msec 		= ControlState_CHECKED;
void SystemClock_Config				( void );
/**
 * @brief  Usart Interrupt Fonksiyonu
 * @param  huart	: Kesmenin meydana geldigi usart modud�
 * @retval [ void ]
 */
void HAL_UART_RxCpltCallback		( UART_HandleTypeDef *huart ) {		//	Usart Interrupt Fonksiyonu
	if (huart->Instance == USART1) {
		buffer_clear_timer = 200;
		buffer_cleared = 0;
		usartrx[rx_indeks] = casual_rx_data;
		if (usartrx[rx_indeks] == 0x0A && usartrx[rx_indeks - 1] == 0x0D) { // Gelan datalar arasinda 0x0D ve 0x0A
																			// pesin sira geldiyse
			ControlUsart1_ReceiveData = ControlState_CHECKIT;
		}
		rx_indeks++;
		if (rx_indeks > USART1_RX_ARRAY_SIZE) {
			rx_indeks--;
			for (uint8_t i = 0; i < USART1_RX_ARRAY_SIZE - 1; i++)
				usartrx[i] = usartrx[i + 1];
		}
	}
}
/**
 * @brief  Timer Interrupt Fonksiyonu
 * @param  htim	: Kesmenin meydana geldigi timer modul�
 *          This parameter can be one of the following values:
 *          @arg   TIM4 : Board uzerindeki ledi toggle yapmak icin
 * @retval [ void ]
 */
void HAL_TIM_PeriodElapsedCallback	( TIM_HandleTypeDef *htim ) {		//	Timer Interrupt Fonksiyonu
	static uint32_t usn10;

	if (htim->Instance == TIM1) {
		((TIM1->CR1 & TIM_CR1_DIR) == TIM_CR1_DR_CW ) ?
				enc_signal_msb++ : enc_signal_msb--;
	}
	if (htim->Instance == TIM4) {
		usn10++;
		if (buffer_clear_timer > 0)
			buffer_clear_timer--;
		if ((usn10 % 1000) == 0) {  // 10 mili second
			ControlTIM4_10msec = ControlState_CHECKIT;
		}
		if (usn10 == 10000) {  //100 mili second
		}
		if (usn10 == 100000) {  //1 second
			HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );
			usn10 = 0;
		}
	}
}
/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback			( void ) {
	static uint8_t ledcount = 0;
	ledcount++;
	if( ledcount >= 99 ) {
		ledcount = 0;
	}
}
/**
 * @brief  EXTI Interrupt Fonksiyonu
 * @param  GPIO_Pin : Exti interrupt'un meydana geldigi pin
 *          This parameter can be one of the following values:
 *            @arg   ADC_1_RDYB_EXTI_IRQn : Max11254-1 RDYB pini her low'a d�s�ste olusan kesme
 *            @arg   ADC_2_RDYB_EXTI_IRQn : Max11254-2 RDYB pini her low'a d�s�ste olusan kesme
 *            @arg   ADC_3_RDYB_EXTI_IRQn : Max11254-3 RDYB pini her low'a d�s�ste olusan kesme
 *            @arg   ADC_4_RDYB_EXTI_IRQn : Max11254-4 RDYB pini her low'a d�s�ste olusan kesme
 *            @arg   EncoderZF_EXTI_IRQn	 : EncoderZF  pini her low'a d�s�ste olusan kesme
 * @retval [ void ]
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { 						//	EXTI Interrupt Fonksiyonu
	if (GPIO_Pin == ADC_1_RDYB_Pin)
		ControlExti_Max_1_RdbyPin = ControlState_CHECKIT;
	if (GPIO_Pin == ADC_2_RDYB_Pin)
		ControlExti_Max_2_RdbyPin = ControlState_CHECKIT;
	if (GPIO_Pin == ADC_3_RDYB_Pin)
		ControlExti_Max_3_RdbyPin = ControlState_CHECKIT;
	if (GPIO_Pin == ADC_4_RDYB_Pin)
		ControlExti_Max_4_RdbyPin = ControlState_CHECKIT;
	if (GPIO_Pin == EncoderZF_Pin)
		((TIM1->CR1 & TIM_CR1_DIR) == TIM_CR1_DR_CW ) ?
				signal_z_count++ : signal_z_count--;
}

void clear_usart_buffer(void) {
	rx_indeks = 0;
	for (uint8_t i = 0; i < USART1_RX_ARRAY_SIZE ; i++) {
		usartrx[i] = 0;
	}
}
/**	*****	*****	*****	**/
int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	//MX_TIM1_Init();		//	Encoder sinyalinin bagli oldugu timer modulu
	MX_TIM8_Init(); 	// 	Duty Modilation 		-> 0 to 14Bit Duty output   ( Usart Control ) [DacPWM]	 [PC8]
	MX_TIM3_Init();     //  Frequency Modilation 	-> 0.1Hz to 1MHz PWM output.( Usart Control ) [PulseOut][PB4]

	MX_DMA_Init();

	MX_SPI2_Init();

	Max11254_Init();

	MX_USART1_UART_Init();
	MX_UART4_Init();

	MX_TIM4_Init();
	ControlTIM4_10msec = ControlState_CHECKIT;

	while (1) {
		if (buffer_cleared == 0) {
			if (buffer_clear_timer == 0) {
				buffer_cleared = 1;
				clear_usart_buffer();
			}
		}
		if (ControlTIM4_10msec == ControlState_CHECKIT) {
			if (calculate_slopes == 1) {
				calculate_slopes = 0;
				//slope_calculation(active_cal_channel);
			}
			ControlTIM4_10msec = ControlState_CHECKED;
//			for (uint8_t i = 0; i < 4; i++) {
//				channel[i].raw = DeviceChannel[i];
//				channel[i].raw_sign = '+';			//	Unipolar Rate
////				if (channel[i].raw > 0x7FFFFF) {
////					channel[i].raw_sign = '-';
////					channel[i].raw = (0xFFFFFF + 1) - channel[i].raw;
////				} else {
////					channel[i].raw_sign = '+';
////				}
//			}
//			ControlTIM4_10msec = ControlState_CHECKED;
//			if (calculate_slopes == 1) {
//				calculate_slopes = 0;
//				slope_calculation(active_cal_channel);
//			}
		}
		if (ControlExti_Max_1_RdbyPin == ControlState_CHECKIT) {
			OperatingMaxExtiRdbyControl( MAX_1 );
			channel[0].raw = MAX[resultBinding[0]/6].chResult[resultBinding[0]%6];
			channel[0].raw_sign = '+';
			if   ( MAX[0].polarity == POLARITY_BIPOLAR ) {
				if (channel[0].raw > 0x7FFFFF) {
					channel[0].raw_sign = '-';
					channel[0].raw = (0xFFFFFF + 1) - channel[0].raw;
				}
			}
			//evaluate_calibrated_values( 0 );
			ControlExti_Max_1_RdbyPin = ControlState_CHECKED;
		}
		if (ControlExti_Max_2_RdbyPin == ControlState_CHECKIT) {
			OperatingMaxExtiRdbyControl( MAX_2 );
			channel[1].raw = MAX[resultBinding[1]/6].chResult[resultBinding[1]%6];
			channel[1].raw_sign = '+';
			if   ( MAX[1].polarity == POLARITY_BIPOLAR ) {
				if (channel[1].raw > 0x7FFFFF) {
					channel[1].raw_sign = '-';
					channel[1].raw = (0xFFFFFF + 1) - channel[1].raw;
				}
			}
			//evaluate_calibrated_values( 1 );
			ControlExti_Max_2_RdbyPin = ControlState_CHECKED;
		}
		if (ControlExti_Max_3_RdbyPin == ControlState_CHECKIT) {
			OperatingMaxExtiRdbyControl( MAX_3 );
			channel[2].raw = MAX[resultBinding[2]/6].chResult[resultBinding[2]%6];
			channel[2].raw_sign = '+';
			if   ( MAX[2].polarity == POLARITY_BIPOLAR ) {
				if (channel[2].raw > 0x7FFFFF) {
					channel[2].raw_sign = '-';
					channel[2].raw = (0xFFFFFF + 1) - channel[2].raw;
				}
			}
			//evaluate_calibrated_values( 2 );
			ControlExti_Max_3_RdbyPin = ControlState_CHECKED;
		}
		if (ControlExti_Max_4_RdbyPin == ControlState_CHECKIT) {
			OperatingMaxExtiRdbyControl( MAX_4 );
			channel[3].raw = MAX[resultBinding[3]/6].chResult[resultBinding[3]%6];
			channel[3].raw_sign = '+';
			if   ( MAX[3].polarity == POLARITY_BIPOLAR ) {
				if (channel[3].raw > 0x7FFFFF) {
					channel[3].raw_sign = '-';
					channel[3].raw = (0xFFFFFF + 1) - channel[3].raw;
				}
			}
			//evaluate_calibrated_values( 3 );
			ControlExti_Max_4_RdbyPin = ControlState_CHECKED;
		}

		if (ControlUsart1_ReceiveData == ControlState_CHECKIT) {
			UsartReceiveData_SearchCommand();
			ControlUsart1_ReceiveData = ControlState_CHECKED;
		}
		if (ControlUsart1_TransmitData == ControlState_CHECKIT) {
			HAL_UART_Transmit_DMA(&huart1, &usarttx[0], TxAmound);
			ControlUsart1_TransmitData = ControlState_CHECKED;
		}
	}
}
/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

