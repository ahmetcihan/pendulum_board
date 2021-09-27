#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "max11254.h"

ControlState ControlUsart1_ReceiveData 	= ControlState_CHECKED,
			 ControlUsart1_TransmitData = ControlState_CHECKED;

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
			timer_10_msec = 1;
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
		max1_dataready = 1;
	if (GPIO_Pin == ADC_2_RDYB_Pin)
		max2_dataready = 1;
	if (GPIO_Pin == ADC_3_RDYB_Pin)
		max3_dataready = 1;
	if (GPIO_Pin == ADC_4_RDYB_Pin)
		max4_dataready = 1;
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
void channel_operation(u8 no){
	OperatingMaxExtiRdbyControl(no+1);
	cal[no].unsigned_raw = MAX[resultBinding[no]/6].chResult[resultBinding[no]%6];
	cal[no].signed_raw = cal[no].unsigned_raw;
	if   ( MAX[no].polarity == POLARITY_BIPOLAR ) {
		if (cal[no].unsigned_raw > 0x7FFFFF) {
			cal[no].unsigned_raw = (0xFFFFFF + 1) - cal[no].unsigned_raw;
			cal[no].signed_raw = -cal[no].unsigned_raw;
		}
	}
	cal[no].calibrated = evaluate_calibrated_values(no);
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
	Timer3_AutoConsolidation_SpecialFunc(0);

	step_motor_command = 0;
	step_motor_requested_pos = 0;
	stepper_abs_pos = 0;
	timer_1_msec = 0;
	timer_10_msec = 0;
	timer_100_msec = 0;
	send_RS485 = 0;
	usart_debugger = 0;

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
		if (timer_10_msec == 0) {
			timer_10_msec = 0;
		}
		if (max1_dataready == 1) {
			max1_dataready = 0;
			channel_operation(0);
		}
		if (max2_dataready == 1) {
			max2_dataready = 0;
			channel_operation(1);
		}
		if (max3_dataready == 1) {
			max3_dataready = 0;
			channel_operation(2);
		}
		if (max4_dataready == 1) {
			max4_dataready = 0;
			channel_operation(3);
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
