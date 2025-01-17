#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "max11254.h"


DMA_HandleTypeDef  hdma_usart1_rx,hdma_usart1_tx,hdma_uart4_tx;
UART_HandleTypeDef huart1,huart2,huart4;

union _char_to_f char_to_f;
struct _usart usart1,usart2;

void MX_USART1_UART_Init(void){
	huart1.Instance 			= USART1;
	huart1.Init.BaudRate 		= 115200;
	huart1.Init.WordLength 		= UART_WORDLENGTH_8B;
	huart1.Init.StopBits 		= UART_STOPBITS_1;
	huart1.Init.Parity 			= UART_PARITY_NONE;
	huart1.Init.Mode 			= UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart1.Init.OverSampling 	= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	HAL_UART_Receive_DMA(&huart1 ,&usart1.instant_data , 1);
	usart1.clear_buffer = 1;
	usart1.rx_indeks = 0;
}
void MX_USART2_Init(void){
	huart2.Instance 			= USART2;
	huart2.Init.BaudRate 		= 230400;
	huart2.Init.WordLength 		= UART_WORDLENGTH_8B;
	huart2.Init.StopBits 		= UART_STOPBITS_1;
	huart2.Init.Parity 			= UART_PARITY_NONE;
	huart2.Init.Mode 			= UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
	huart2.Init.OverSampling 	= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
	usart2.clear_buffer = 1;
	usart2.rx_indeks = 0;
    HAL_UART_Receive_IT(&huart2, &usart2.instant_data,1);
}
void MX_UART4_Init(void){
	huart4.Instance 			= UART4;
	huart4.Init.BaudRate 		= 115200;
	huart4.Init.WordLength 		= UART_WORDLENGTH_8B;
	huart4.Init.StopBits 		= UART_STOPBITS_1;
	huart4.Init.Parity 			= UART_PARITY_NONE;
	huart4.Init.Mode 			= UART_MODE_TX;
	huart4.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
	huart4.Init.OverSampling 	= UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	usart1.rx[ 0 ] = 10;
	HAL_UART_Transmit( &huart4 , &usart1.rx[0] , 1 , 10 );
}
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle){
	GPIO_InitTypeDef GPIO_InitStruct;

	if(uartHandle->Instance == UART4){
		__HAL_RCC_UART4_CLK_ENABLE();

		GPIO_InitStruct.Pin 		= GPIO_PIN_0;
		GPIO_InitStruct.Mode 		= GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull 		= GPIO_PULLUP;
		GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate 	= GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else if ( uartHandle->Instance == USART1 ) {
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitStruct.Pin  = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hdma_usart1_rx.Instance = DMA2_Stream2;
	hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	__HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
	else if(uartHandle->Instance == USART2){
		__HAL_RCC_USART2_CLK_ENABLE();

		GPIO_InitStruct.Pin  = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
	}
}
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle){

	if(uartHandle -> Instance == UART4){
		__HAL_RCC_UART4_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
		HAL_DMA_DeInit(uartHandle->hdmatx);
		HAL_NVIC_DisableIRQ(UART4_IRQn);
	}
	else if(uartHandle -> Instance == USART1){
		__HAL_RCC_USART1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
		HAL_DMA_DeInit(uartHandle->hdmarx);
		HAL_DMA_DeInit(uartHandle->hdmatx);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
  else if(uartHandle -> Instance == USART2){
		__HAL_RCC_USART2_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
		HAL_NVIC_DisableIRQ(USART2_IRQn);
	}
}
uint32_t CyclicRedundancyCheck(uint8_t* data, uint8_t length) {
	int j;
	uint32_t reg_crc=0xFFFF;

	while( length-- ) {
		reg_crc^= *data++;
		for (j = 0; j < 8; j++){
			reg_crc = (reg_crc & 0x01) ? ((reg_crc >> 1)^0xA001) : (reg_crc>>1);
		}
	}
	return reg_crc;
}
void MASTER_send_RS485_data_to_motor(void){
	HAL_GPIO_WritePin( GPIOA , Tx2En , GPIO_PIN_SET );
	usart2.tx[0] = 'S';
	usart2.tx[1] = 'T';
	usart2.tx[2] = 'P';
	usart2.tx[3] = 0x00;	//device address

	usart2.tx[4] = step_motor_command;
	usart2.tx[5] = (step_motor_requested_pos/256)%256;
	usart2.tx[6] = (step_motor_requested_pos)%256;

	usart2.tx[7] = step_motor_speed[0];
	usart2.tx[8] = step_motor_speed[1];
	usart2.tx[9] = step_motor_speed[2];

	usart2.tx[10] = 0x0D;
	usart2.tx[11] = 0x0A;

	usart2.tx_amount = 12;
	HAL_UART_Transmit_IT(&huart2, &usart2.tx[0], usart2.tx_amount);
}
void usart2_handle(void){
	if((usart2.rx[0] == 's') && (usart2.rx[1] == 't') && (usart2.rx[2] == 'p')){
		stepper_abs_pos = 65536 * usart2.rx[3] + 256 * usart2.rx[4] + usart2.rx[5];
		HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );
//		usart2.rx_indeks = 0;
//		for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE ; i++) {
//			usart2.rx[i] = 0;
//		}
	}
}
void USART1_receive_operations(void){
	if(usart1.rx[0]=='C' && usart1.rx[1]=='O' && usart1.rx[2]=='N' && usart1.rx[3]=='V' ) {
			PRESS_CONV_CommandOperating( );
	}
	else if(usart1.rx[0]=='C' && usart1.rx[1]=='L' && usart1.rx[2]=='R' && usart1.rx[3]=='E' && usart1.rx[4]=='N' && usart1.rx[5]=='C' ) {
		signal_z_count = 0;
		enc_signal_msb = 0;
		TIM1->CNT = 0;
		encoder_value = 0;
	}
	else if(usart1.rx[0]=='M' && usart1.rx[1]=='I' && usart1.rx[2]=='D' && usart1.rx[3]=='U' && usart1.rx[4]=='P' ) {
		mid_point_up_cmd = 1;
	}
	else if(usart1.rx[0]=='M' && usart1.rx[1]=='I' && usart1.rx[2]=='D' && usart1.rx[3]=='D' && usart1.rx[4]=='N' ) {
		mid_point_down_cmd = 1;
	}
	else if(usart1.rx[0]=='P' && usart1.rx[1]=='R' && usart1.rx[2]=='I' && usart1.rx[3]=='N' && usart1.rx[4]=='T' ) {
		PRESS_PRINT_CommandOperating();
	}
	else if(usart1.rx[0]=='C' && usart1.rx[1]=='A' && usart1.rx[2]=='L' && usart1.rx[3]=='S' && usart1.rx[4]=='E' && usart1.rx[5]=='N' && usart1.rx[6]=='D' )  {
		PRESS_CALSEND_CommandOperating();
	}
	else if(usart1.rx[0]=='P' && usart1.rx[1]=='L' && usart1.rx[2]=='R' && usart1.rx[3]=='T' ) {
		uint8_t chn  = usart1.rx[4] - 0x30;
		uint8_t plrt = usart1.rx[5] - 0x30;

		channel_polarity[chn] = plrt;

		if (channel_polarity[chn] == 0){
			MAX[chn].polarity = POLARITY_UNIPOLAR;
		}
		else{
			MAX[chn].polarity = POLARITY_BIPOLAR;
		}
		Max11254_PolaritySelect( (chn+1) , MAX[chn].polarity );
		Max11254_ConversionCommand( (chn+1) , MAX[chn].RateNumber|COMMAND_MODE_SEQUENCER );
	}
	else if(usart1.rx[0]=='G' && usart1.rx[1]=='P' && usart1.rx[2]=='I' && usart1.rx[3]=='O' ) {
		if (usart1.rx[4] <= 3 )
			Max11254_GPIOSetting( usart1.rx[4]+1 , usart1.rx[5] );
	}
	else if(usart1.rx[0]=='P' && usart1.rx[1]=='R' && usart1.rx[2]=='M' && usart1.rx[3]=='T' ) {
		//MENU PARAMETERS
		char_to_f.u8_val[0] = usart1.rx[4];
		char_to_f.u8_val[1] = usart1.rx[5];
		char_to_f.u8_val[2] = usart1.rx[6];
		char_to_f.u8_val[3] = usart1.rx[7];
		pendulum.headshake_speed = char_to_f.u32_val;
		char_to_f.u8_val[0] = usart1.rx[8];
		char_to_f.u8_val[1] = usart1.rx[9];
		char_to_f.u8_val[2] = usart1.rx[10];
		char_to_f.u8_val[3] = usart1.rx[11];
		pendulum.head_change_timer = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[12];
		char_to_f.u8_val[1] = usart1.rx[13];
		char_to_f.u8_val[2] = usart1.rx[14];
		char_to_f.u8_val[3] = usart1.rx[15];
		pendulum.mid_point = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[16];
		char_to_f.u8_val[1] = usart1.rx[17];
		char_to_f.u8_val[2] = usart1.rx[18];
		char_to_f.u8_val[3] = usart1.rx[19];
		pendulum.top_boundary = char_to_f.float_val;

		//usart1.rx[20];

		//AUTOTUNING
		char_to_f.u8_val[0] = usart1.rx[21];
		char_to_f.u8_val[1] = usart1.rx[22];
		char_to_f.u8_val[2] = usart1.rx[23];
		char_to_f.u8_val[3] = usart1.rx[24];
		//char_to_f.float_val;

		char_to_f.u8_val[0] = usart1.rx[25];
		char_to_f.u8_val[1] = usart1.rx[26];
		char_to_f.u8_val[2] = usart1.rx[27];
		char_to_f.u8_val[3] = usart1.rx[28];
		parameters.step_second_speed = char_to_f.u32_val;
		parameters.step_transition_time = usart1.rx[29];

		//PID
		char_to_f.u8_val[0] = usart1.rx[30];
		char_to_f.u8_val[1] = usart1.rx[31];
		char_to_f.u8_val[2] = usart1.rx[32];
		char_to_f.u8_val[3] = usart1.rx[33];
		pendulum.kp = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[34];
		char_to_f.u8_val[1] = usart1.rx[35];
		char_to_f.u8_val[2] = usart1.rx[36];
		char_to_f.u8_val[3] = usart1.rx[37];
		pendulum.ki = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[38];
		char_to_f.u8_val[1] = usart1.rx[39];
		char_to_f.u8_val[2] = usart1.rx[40];
		char_to_f.u8_val[3] = usart1.rx[41];
		pendulum.kd = char_to_f.float_val;

		char_to_f.u8_val[0] = usart1.rx[42];
		char_to_f.u8_val[1] = usart1.rx[43];
		char_to_f.u8_val[2] = usart1.rx[44];
		char_to_f.u8_val[3] = usart1.rx[45];
		pendulum.kp_down = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[46];
		char_to_f.u8_val[1] = usart1.rx[47];
		char_to_f.u8_val[2] = usart1.rx[48];
		char_to_f.u8_val[3] = usart1.rx[49];
		pendulum.ki_down = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[50];
		char_to_f.u8_val[1] = usart1.rx[51];
		char_to_f.u8_val[2] = usart1.rx[52];
		char_to_f.u8_val[3] = usart1.rx[53];
		pendulum.kd_down = char_to_f.float_val;

		char_to_f.u8_val[0] = usart1.rx[54];
		char_to_f.u8_val[1] = usart1.rx[55];
		char_to_f.u8_val[2] = usart1.rx[56];
		char_to_f.u8_val[3] = usart1.rx[57];
		pendulum.lqr_k1_down = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[58];
		char_to_f.u8_val[1] = usart1.rx[59];
		char_to_f.u8_val[2] = usart1.rx[60];
		char_to_f.u8_val[3] = usart1.rx[61];
		pendulum.lqr_k2_down = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[62];
		char_to_f.u8_val[1] = usart1.rx[63];
		char_to_f.u8_val[2] = usart1.rx[64];
		char_to_f.u8_val[3] = usart1.rx[65];
		pendulum.lqr_k3_down = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[66];
		char_to_f.u8_val[1] = usart1.rx[67];
		char_to_f.u8_val[2] = usart1.rx[68];
		char_to_f.u8_val[3] = usart1.rx[69];
		pendulum.lqr_k4_down = char_to_f.float_val;

		char_to_f.u8_val[0] = usart1.rx[70];
		char_to_f.u8_val[1] = usart1.rx[71];
		char_to_f.u8_val[2] = usart1.rx[72];
		char_to_f.u8_val[3] = usart1.rx[73];
		pendulum.lqr_k1_up = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[74];
		char_to_f.u8_val[1] = usart1.rx[75];
		char_to_f.u8_val[2] = usart1.rx[76];
		char_to_f.u8_val[3] = usart1.rx[77];
		pendulum.lqr_k2_up = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[78];
		char_to_f.u8_val[1] = usart1.rx[79];
		char_to_f.u8_val[2] = usart1.rx[80];
		char_to_f.u8_val[3] = usart1.rx[81];
		pendulum.lqr_k3_up = char_to_f.float_val;
		char_to_f.u8_val[0] = usart1.rx[82];
		char_to_f.u8_val[1] = usart1.rx[83];
		char_to_f.u8_val[2] = usart1.rx[84];
		char_to_f.u8_val[3] = usart1.rx[85];
		pendulum.lqr_k4_up = char_to_f.float_val;

	}
}
void PRESS_CONV_CommandOperating(void){
	//	0x30[Hex] = 48[Dec]	,	0x31[Hex] = 49[Dec]
	if (usart1.rx[4] == 0x30 )	Electromechanic_RELAY_OFF_AutoManual;
	else if(usart1.rx[4] == 0x31 ) 	Electromechanic_RELAY_ON_AutoManual;

	//	0x30[Hex] = 48[Dec]	,	0x31[Hex] = 49[Dec]
	if (usart1.rx[5] == 0x30 ) 	Electromechanic_RELAY_OFF_StartStop;
	else if (usart1.rx[5] == 0x31 ) 	Electromechanic_RELAY_ON_StartStop;

	TMC_command = usart1.rx[12];
	if(TMC_command == TMC_STOP){
		step_motor_command = usart1.rx[6];
		step_motor_requested_pos = (uint32_t)((uint32_t)usart1.rx[7] * 256 + (uint32_t)usart1.rx[8]);
		step_motor_speed[0] = usart1.rx[9];
		step_motor_speed[1] = usart1.rx[10];
		step_motor_speed[2] = usart1.rx[11];
		//send_RS485 = 1;
		autotuning_is_finished = 0;
		PID_in_operation = 0;
	}
	else if(TMC_command == TMC_AUTOTUNING){
		if(autotuning_is_finished == 0){
			if(autotuning_in_operation == 0){
				step_response_first_in = 1;
			}
		}
		PID_in_operation = 0;
	}
	else if(TMC_command == TMC_RUN){
		if(PID_in_operation == 0){
			control_process_tmp = 0;
		}
	}

	PRESS_ANS_Command();
}
void PRESS_GAIN_CommandOperating(void){
	uint8_t usart_gain = usart1.rx[4] - 50;		//	48;
	uint8_t usart_adc  = usart1.rx[5] - 48;
	if(usart_gain <= 8 ){
		MAX[usart_adc].Gain = (GAIN)( usart_gain );
		MAX[usart_adc].chGain[4] = (GAIN)( usart_gain );
	}
	Max11254_PGAGain_Set( (MaxDevice)(usart_adc+1) , MAX[usart_adc].Gain );
	Max11254_ConversionCommand( (MaxDevice)(usart_adc+1) , MAX[usart_adc].RateNumber|COMMAND_MODE_SEQUENCER );
	//Max11254_SequencerMode2_Entry( MAX_1 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[usart_adc].chGain[4] );
} 
void PRESS_TARE_CommandOperating(void){
	u8 ch = usart1.rx[4];

	cal[ch].tare_val = cal[ch].absolute_calibrated;
}
void PRESS_PRINT_CommandOperating(void){
		static uint8_t print_count;
		print_count++;
		if ( print_count == 22 ) {
			usart1.rx[ 37 ] = 0x0A;
			usart1.rx[ 38 ] = 0x1B;
			usart1.rx[ 39 ] = 0x69;
			HAL_UART_Transmit( &huart4 , &usart1.rx[5] , 35 , 15 );
			print_count = 0;
		}
		else
			HAL_UART_Transmit( &huart4 , &usart1.rx[5] , 32 , 10 );
}
void PRESS_CALSEND_CommandOperating(void){
	u8 ch_no = 0;

	ch_no = usart1.rx[7];
	cal[ch_no].point_no = usart1.rx[8];

	for( uint8_t i = 0 ; i < 8 ; i++ ) {
		char_to_f.u8_val[0] = usart1.rx[9 + 4*i];
		char_to_f.u8_val[1] = usart1.rx[10 + 4*i];
		char_to_f.u8_val[2] = usart1.rx[11 + 4*i];
		char_to_f.u8_val[3] = usart1.rx[12 + 4*i];
		cal[ch_no].real_val[i] = char_to_f.s32_val;
	}
	for( uint8_t i = 0 ; i < 8 ; i++ ) {
		char_to_f.u8_val[0] = usart1.rx[41 + 4*i];
		char_to_f.u8_val[1] = usart1.rx[42 + 4*i];
		char_to_f.u8_val[2] = usart1.rx[43 + 4*i];
		char_to_f.u8_val[3] = usart1.rx[44 + 4*i];
		cal[ch_no].assigned_val[i] = char_to_f.float_val;
	}
	slope_calculation(ch_no);
}
void PRESS_ANS_Command(void){
	//	unsigned char *calibrated;
		usart1.tx[0] = 'A';
		usart1.tx[1] = 'N';
		usart1.tx[2] = 'S';
		for ( uint8_t i = 0 ; i < 4 ; i++ ) {
	//		calibrated = (uint8_t *)&channel[i].calibrated;
			//uint8_t gain_force = (uint8_t)( MAX[i].Gain +  2 );
			uint8_t gain_force =  MAX[resultBinding[i]/6].chGain[resultBinding[i]%6] + 2;	//(uint8_t)( MAX[i].Gain +  2 );

			char_to_f.s32_val = cal[i].signed_raw_filtered;
			usart1.tx[5*i+3]= char_to_f.s8_val[0];
			usart1.tx[5*i+4]= char_to_f.s8_val[1];
			usart1.tx[5*i+5]= char_to_f.s8_val[2];
			usart1.tx[5*i+6]= char_to_f.s8_val[3];
			usart1.tx[5*i+7]= gain_force;
		}

		usart1.tx[23] = input_status[0] + 0x30;
		usart1.tx[24] = input_status[1] + 0x30;
		usart1.tx[25] = input_status[2] + 0x30;
		usart1.tx[26] = input_status[3] + 0x30;

		usart1.tx[27] = channel_polarity[0] + 0x30;
		usart1.tx[28] = channel_polarity[1] + 0x30;
		usart1.tx[29] = channel_polarity[2] + 0x30;
		usart1.tx[30] = channel_polarity[3] + 0x30;

		usart1.tx[31] = (uint8_t)(stepper_abs_pos >> 16);
		usart1.tx[32] = (uint8_t)(stepper_abs_pos >> 8);
		usart1.tx[33] = (uint8_t)(stepper_abs_pos);

		for (uint8_t i = 0 ; i < 4 ; i++ ) {
			char_to_f.float_val = cal[i].calibrated;
			usart1.tx[4*i+34]= char_to_f.s8_val[0];
			usart1.tx[4*i+35]= char_to_f.s8_val[1];
			usart1.tx[4*i+36]= char_to_f.s8_val[2];
			usart1.tx[4*i+37]= char_to_f.s8_val[3];
		}
		//last index is 49
		usart1.tx[50] = usart_debugger_u8;

		char_to_f.s32_val = abs_encoder;
		usart1.tx[51]= char_to_f.s8_val[0];
		usart1.tx[52]= char_to_f.s8_val[1];
		usart1.tx[53]= char_to_f.s8_val[2];
		usart1.tx[54]= char_to_f.s8_val[3];

		char_to_f.s32_val = usart_debugger_s32;
		usart1.tx[55]= char_to_f.s8_val[0];
		usart1.tx[56]= char_to_f.s8_val[1];
		usart1.tx[57]= char_to_f.s8_val[2];
		usart1.tx[58]= char_to_f.s8_val[3];

		char_to_f.float_val = usart_debugger_float[0];
		usart1.tx[59]= char_to_f.s8_val[0];
		usart1.tx[60]= char_to_f.s8_val[1];
		usart1.tx[61]= char_to_f.s8_val[2];
		usart1.tx[62]= char_to_f.s8_val[3];
		char_to_f.float_val = usart_debugger_float[1];
		usart1.tx[63]= char_to_f.s8_val[0];
		usart1.tx[64]= char_to_f.s8_val[1];
		usart1.tx[65]= char_to_f.s8_val[2];
		usart1.tx[66]= char_to_f.s8_val[3];
		char_to_f.float_val = usart_debugger_float[2];
		usart1.tx[67]= char_to_f.s8_val[0];
		usart1.tx[68]= char_to_f.s8_val[1];
		usart1.tx[69]= char_to_f.s8_val[2];
		usart1.tx[70]= char_to_f.s8_val[3];

		usart1.tx[71]= autotuning_in_operation;

		char_to_f.u32_val = plot_counter_1_msec;
		usart1.tx[72]= char_to_f.s8_val[0];
		usart1.tx[73]= char_to_f.s8_val[1];
		usart1.tx[74]= char_to_f.s8_val[2];
		usart1.tx[75]= char_to_f.s8_val[3];

		char_to_f.s32_val = signal_z_count;
		usart1.tx[76]= char_to_f.s8_val[0];
		usart1.tx[77]= char_to_f.s8_val[1];
		usart1.tx[78]= char_to_f.s8_val[2];
		usart1.tx[79]= char_to_f.s8_val[3];

		char_to_f.s32_val = encoder_value;
		usart1.tx[80]= char_to_f.s8_val[0];
		usart1.tx[81]= char_to_f.s8_val[1];
		usart1.tx[82]= char_to_f.s8_val[2];
		usart1.tx[83]= char_to_f.s8_val[3];

		uint16_t fcrc;
		fcrc = CyclicRedundancyCheck( &usart1.tx[0] , 84);
		usart1.tx[84] = fcrc%256;
		usart1.tx[85] = fcrc/256;
		usart1.tx_amount = 86;
		usart1_transmit = 1;
}

