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
void SystemClock_Config(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1) {
		usart1.buffer_clear_timer = 200;
		usart1.clear_buffer = 1;
		usart1.rx[usart1.rx_indeks] = usart1.instant_data;
		if (usart1.rx[usart1.rx_indeks] == 0x0A && usart1.rx[usart1.rx_indeks - 1] == 0x0D) {
			ControlUsart1_ReceiveData = ControlState_CHECKIT;
		}
		usart1.rx_indeks++;
		if (usart1.rx_indeks > USART_RX_ARRAY_SIZE) {
			usart1.rx_indeks--;
			for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE - 1; i++)
				usart1.rx[i] = usart1.rx[i + 1];
		}
	}
	if (huart->Instance == USART2) {
		usart2.buffer_clear_timer = 200;
		usart2.clear_buffer = 1;
		usart2.rx[usart2.rx_indeks] = usart2.instant_data;

		if (usart2.rx[usart2.rx_indeks] == 0x0A && usart2.rx[usart2.rx_indeks - 1] == 0x0D) {
			usart2.data_received = 1;
		}
		if (usart2.rx_indeks < USART_RX_ARRAY_SIZE) usart2.rx_indeks++;
	    HAL_UART_Receive_IT(&huart2, &usart2.instant_data,1);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		HAL_GPIO_WritePin( GPIOA , Tx2En , GPIO_PIN_RESET );
	}
}
void HAL_TIM_PeriodElapsedCallback	( TIM_HandleTypeDef *htim ) {		//	Timer Interrupt Fonksiyonu
	static uint32_t usn10;

	if (htim->Instance == TIM1) {
		((TIM1->CR1 & TIM_CR1_DIR) == TIM_CR1_DR_CW ) ?
				enc_signal_msb++ : enc_signal_msb--;
	}
	if (htim->Instance == TIM4) {
		usn10++;

		if(usart1.buffer_clear_timer > 0) usart1.buffer_clear_timer--;
		if(usart2.buffer_clear_timer > 0) usart2.buffer_clear_timer--;

		if ((usn10 % 100) == 0) {
			timer_1_msec = 1;
		}
		if ((usn10 % 1000) == 0) {
			ControlTIM4_10msec = ControlState_CHECKIT;
		}
		if ((usn10 % 10000) == 0) {
			timer_100_msec = 1;
		}
		if (usn10 == 100000) {
			HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );
			usn10 = 0;
		}
	}
}
void HAL_SYSTICK_Callback			( void ) {
	static uint8_t ledcount = 0;
	ledcount++;
	if( ledcount >= 99 ) {
		ledcount = 0;
	}
}
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
void read_inputs(void){
	input_status[0] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_1_Pin);
	input_status[1] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_2_Pin);
	input_status[2] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_3_Pin);
	input_status[3] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_4_Pin);
}
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	//MX_TIM1_Init();		//	Encoder sinyalinin bagli oldugu timer modulu
	MX_TIM8_Init(); 	// 	Duty Modilation 		-> 0 to 14Bit Duty output   ( Usart Control ) [DacPWM]	 [PC8]
	MX_TIM3_Init();     //  Frequency Modilation 	-> 0.1Hz to 1MHz PWM output.( Usart Control ) [PulseOut][PB4]
	MX_DMA_Init();
	MX_SPI2_Init();
	Max11254_Init();
	MX_USART1_UART_Init();
	MX_USART2_Init();
	MX_UART4_Init();
	MX_TIM4_Init();
	ControlTIM4_10msec = ControlState_CHECKIT;
	Timer3_AutoConsolidation_SpecialFunc(0);

	step_motor_command = 0;
	step_motor_requested_pos = 0;
	stepper_abs_pos = 0;
	timer_1_msec = 0;
	timer_100_msec = 0;
	send_RS485 = 0;

	while (1) {
		if (usart2.data_received == 1) {
			usart2.data_received = 0;
			usart2_handle();
		}
		if (usart2.clear_buffer == 1) {
			if (usart2.buffer_clear_timer == 0) {
				usart2.clear_buffer = 0;
				usart2.rx_indeks = 0;
				for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE ; i++) {
					usart2.rx[i] = 0;
				}
			}
		}

		if (usart1.clear_buffer == 1) {
			if (usart1.buffer_clear_timer == 0) {
				usart1.clear_buffer = 0;
				usart1.rx_indeks = 0;
				for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE ; i++) {
					usart1.rx[i] = 0;
				}
			}
		}

		if(send_RS485 == 1){
			send_RS485 = 0;
			MASTER_send_RS485_data_to_motor();
			read_inputs();
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
			//HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );
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
			HAL_UART_Transmit_DMA(&huart1, &usart1.tx[0], usart1.tx_amount);
			ControlUsart1_TransmitData = ControlState_CHECKED;
		}
	}
}
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
void _Error_Handler(char * file, int line) {
	while (1) {
	}
}
