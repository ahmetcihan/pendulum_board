#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "max11254.h"

uint8_t u1_ctrl1;
uint8_t u2_ctrl1;
uint8_t u3_ctrl1;
uint8_t u4_ctrl1;

DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;
DMA_HandleTypeDef  hdma_uart4_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

union 	_char_to_f {
	float float_val;
	unsigned char char_val[4];
};
union _char_to_f char_to_f;
struct chan channel[8];
struct _usart usart1,usart2;
struct _cal cal[4];

void MX_UART4_Init			( void ) {
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
void MX_USART2_Init			( void ) {
	huart2.Instance 			= USART2;
	huart2.Init.BaudRate 		= 115200;
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
void MX_USART1_UART_Init 	( void ) {
	huart1.Instance 			= USART1;
	huart1.Init.BaudRate 		= 115200;
	huart1.Init.WordLength 	= UART_WORDLENGTH_8B;
	huart1.Init.StopBits 		= UART_STOPBITS_1;
	huart1.Init.Parity 		= UART_PARITY_NONE;
	huart1.Init.Mode 			= UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
	huart1.Init.OverSampling 	= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	HAL_UART_Receive_DMA ( &huart1 , &usart1.instant_data , 1 );
	calculate_slopes = 0;
	usart1.clear_buffer = 1;
	usart1.rx_indeks = 0;
}
void HAL_UART_MspInit		( UART_HandleTypeDef* uartHandle ) {
	GPIO_InitTypeDef GPIO_InitStruct;

	if			( uartHandle->Instance == UART4  ) {
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
	else if ( uartHandle->Instance == USART2 ) {
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
void HAL_UART_MspDeInit  	( UART_HandleTypeDef* uartHandle ) {

	if(uartHandle -> Instance == UART4  ) {
		__HAL_RCC_UART4_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
		HAL_DMA_DeInit(uartHandle->hdmatx);
		HAL_NVIC_DisableIRQ(UART4_IRQn);
	}
	else if ( uartHandle -> Instance == USART1 ) {
		__HAL_RCC_USART1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
		HAL_DMA_DeInit(uartHandle->hdmarx);
		HAL_DMA_DeInit(uartHandle->hdmatx);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
  else if ( uartHandle -> Instance == USART2 ) {
		__HAL_RCC_USART2_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
		HAL_NVIC_DisableIRQ(USART2_IRQn);
	}
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
	if((usart2.rx[0] == 'A') && (usart2.rx[1] == 'H') && (usart2.rx[2] == 'A')){
		stepper_abs_pos = 65536 * usart2.rx[3] + 256 * usart2.rx[4] + usart2.rx[5];
		usart2.rx_indeks = 0;
		for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE ; i++) {
			usart2.rx[i] = 0;
		}
	}
}
uint32_t CyclicRedundancyCheck 				( uint8_t* data, uint8_t length ) {
	int j;
	uint32_t reg_crc=0xFFFF;

	while( length-- ) {
		reg_crc^= *data++;
		for (j=0; j<8; j++ ) {
			reg_crc = (reg_crc & 0x01) ? ((reg_crc >> 1)^0xA001) : (reg_crc>>1);
		}
	}
	return reg_crc;
}
void 	 UsartReceiveData_SearchCommand ( void ) {
	//float *channel_floats[8];
	if 		( usart1.rx[0]=='C' && usart1.rx[1]=='O' && usart1.rx[2]=='N' && usart1.rx[3]=='V' ) {
			PRESS_CONV_CommandOperating( );
	}
	else if ( usart1.rx[0]=='G' && usart1.rx[1]=='A' && usart1.rx[2]=='I' && usart1.rx[3]=='N' ) {
			PRESS_GAIN_CommandOperating( );
	}
	else if ( usart1.rx[0]=='T' && usart1.rx[1]=='A' && usart1.rx[2]=='R' && usart1.rx[3]=='E' ) {
			PRESS_TARE_CommandOperating();
	}
	else if ( usart1.rx[0]=='P' && usart1.rx[1]=='R' && usart1.rx[2]=='I' && usart1.rx[3]=='N' && usart1.rx[4]=='T' ) {
			PRESS_PRINT_CommandOperating();			
	}
	else if ( usart1.rx[0]=='C' && usart1.rx[1]=='A' && usart1.rx[2]=='L' && usart1.rx[3]=='S' && usart1.rx[4]=='E' && usart1.rx[5]=='N' && usart1.rx[6]=='D' )  {
		PRESS_CALSEND_CommandOperating();
	}
	else if	( usart1.rx[0]=='P' && usart1.rx[1]=='L' && usart1.rx[2]=='R' && usart1.rx[3]=='T' ) {
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
	else if ( usart1.rx[0]=='M' && usart1.rx[1]=='A' && usart1.rx[2]=='X' && usart1.rx[3]=='A' && usart1.rx[4]=='D' && usart1.rx[5]=='C' ) {
		if ( (usart1.rx[6]<=3) && (usart1.rx[7]<=1) && (usart1.rx[8]<=15) ) {
			Max11254_SoftwareReset 			( usart1.rx[6] + 1 );
			MAX[ usart1.rx[6] ].polarity 		= usart1.rx[ 7 ];
			MAX[ usart1.rx[6] ].RateNumber 	= usart1.rx[ 8 ];

			for ( uint8_t ch=0 ; ch<6 ; ch++ )	//	All channel cleer Read bit
				MAX[usart1.rx[6]].chRead[ch] = CH_NotREAD;
			for ( uint8_t ch=0 ; ch<6 ; ch++ ) {
				if ( usart1.rx[ 9+ch*2 ] == 1 ) {
					if ( usart1.rx[ 10+ch*2 ] <= 8 ) {
						MAX[usart1.rx[6]].chRead[5-ch] = CH_READ;
						MAX[usart1.rx[6]].chGain[5-ch] = (GAIN)usart1.rx[10+ch*2];
					}
					else {
						MAX[usart1.rx[6]].chRead[5-ch] = CH_NotREAD;
						MAX[usart1.rx[6]].chGain[5-ch] = GAIN_Disb;
					}
				}
				else {
					MAX[usart1.rx[6]].chRead[5-ch] = CH_NotREAD;
					MAX[usart1.rx[6]].chGain[5-ch] = GAIN_Disb;
			}

			for ( uint8_t i=0 ; i<6 ; i++ ) {
				if ( MAX[usart1.rx[6]].chRead[i] == CH_READ ) {
					MAX[usart1.rx[6]].rank = i;
				}
			}
			uint8_t chs = 	(uint8_t)( MAX[usart1.rx[6]].chRead[5] << 5 ) |
							(uint8_t)( MAX[usart1.rx[6]].chRead[4] << 4 ) |
							(uint8_t)( MAX[usart1.rx[6]].chRead[3] << 3 ) |
							(uint8_t)( MAX[usart1.rx[6]].chRead[2] << 2 ) |
							(uint8_t)( MAX[usart1.rx[6]].chRead[1] << 1 ) |
							(uint8_t)( MAX[usart1.rx[6]].chRead[0] );

			Max11254_SequencerMode2_EntryUart(  usart1.rx[6]+1,
												(ChmapConvChannels)chs ,
												MAX[usart1.rx[6]].chGain[MAX[usart1.rx[6]].rank] );

			Max11254_ConversionCommand( usart1.rx[6]+1 , MAX[usart1.rx[6]].RateNumber|COMMAND_MODE_SEQUENCER );

//			Max11254_ChannelMap_Set( chooseMax , (ChmapConvChannels)x );
//			Max11254_ConversionCommand( chooseMax , MAX[usart1.rx[6]].RateNumber|COMMAND_MODE_SEQUENCER );
//			OperatingMaxExtiRdbyControl( (MaxDevice)chooseMax );
		}
		u1_ctrl1 = Max11254_Read1byte( MAX_1 , MAX11254_REG_CTRL1 );
		u2_ctrl1 = Max11254_Read1byte( MAX_2 , MAX11254_REG_CTRL1 );
		u3_ctrl1 = Max11254_Read1byte( MAX_3 , MAX11254_REG_CTRL1 );
		u4_ctrl1 = Max11254_Read1byte( MAX_4 , MAX11254_REG_CTRL1 );
		}
	}
	else if	( usart1.rx[0]=='G' && usart1.rx[1]=='P' && usart1.rx[2]=='I' && usart1.rx[3]=='O' ) {
		if ( usart1.rx[4] <= 3 )
			Max11254_GPIOSetting( usart1.rx[4]+1 , usart1.rx[5] );
	}
	else if ( usart1.rx[0]=='R' && usart1.rx[1]=='E' && usart1.rx[2]=='S' && usart1.rx[3]=='U' && usart1.rx[4]=='L' && usart1.rx[5]=='T' ) {
		if ( usart1.rx[6] <= 23 )
						resultBinding[ 0 ] = usart1.rx[ 6 ];
		if ( usart1.rx[7] <= 23 )
						resultBinding[ 1 ] = usart1.rx[ 7 ];
		if ( usart1.rx[8] <= 23 )
						resultBinding[ 2 ] = usart1.rx[ 8 ];
		if ( usart1.rx[9] <= 23 )
						resultBinding[ 3 ] = usart1.rx[ 9 ];
	}
}
void 	 PRESS_CONV_CommandOperating	( void ) {
	//	0x30[Hex] = 48[Dec]	,	0x31[Hex] = 49[Dec]
	if (usart1.rx[4] == 0x30 )	Electromechanic_RELAY_OFF_AutoManual;
	else if(usart1.rx[4] == 0x31 ) 	Electromechanic_RELAY_ON_AutoManual;

	//	0x30[Hex] = 48[Dec]	,	0x31[Hex] = 49[Dec]
	if (usart1.rx[5] == 0x30 ) 	Electromechanic_RELAY_OFF_StartStop;
	else if (usart1.rx[5] == 0x31 ) 	Electromechanic_RELAY_ON_StartStop;

	step_motor_command = usart1.rx[6];
	step_motor_requested_pos = (uint32_t)((uint32_t)usart1.rx[7] * 256 + (uint32_t)usart1.rx[8]);
	step_motor_speed[0] = usart1.rx[9];
	step_motor_speed[1] = usart1.rx[10];
	step_motor_speed[2] = usart1.rx[11];

	send_RS485 = 1;

	PRESS_ANS_Command();
}
void	 PRESS_GAIN_CommandOperating	( void ) {
	uint8_t usart_gain = usart1.rx[4] - 50;		//	48;
	uint8_t usart_adc  = usart1.rx[5] - 48;
	if ( usart_gain <= 8 ){
		MAX[usart_adc].Gain = (GAIN)( usart_gain );
		MAX[usart_adc].chGain[4] = (GAIN)( usart_gain );
	}
	Max11254_PGAGain_Set( (MaxDevice)(usart_adc+1) , MAX[usart_adc].Gain );
	Max11254_ConversionCommand( (MaxDevice)(usart_adc+1) , MAX[usart_adc].RateNumber|COMMAND_MODE_SEQUENCER );
	//Max11254_SequencerMode2_Entry( MAX_1 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[usart_adc].chGain[4] );

} 
void 	 PRESS_TARE_CommandOperating	( void ) {
	if(usart1.rx[4] < 4){
		channel[usart1.rx[4]].tare = channel[usart1.rx[4]].raw;
		channel[usart1.rx[4]].tare_sign = channel[usart1.rx[4]].raw_sign;
	}
}
void 	 PRESS_PRINT_CommandOperating	( void ) {
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
void	 PRESS_CALSEND_CommandOperating ( void ) {
	uint16_t fcrc;								//	unsigned int fcrc,i;
	uint8_t  crc_high , crc_low;	//	unsigned char crc_high,crc_low;  
	fcrc = CyclicRedundancyCheck((uint8_t*)usart1.rx,74);
	crc_high = (fcrc)%256;
	crc_low = (fcrc)/256;
	if(((uint8_t)usart1.rx[74] == crc_high)&&((uint8_t)usart1.rx[75] == crc_low)){
			active_cal_channel = usart1.rx[7];
			channel[active_cal_channel].point_number = usart1.rx[8];
			channel[active_cal_channel].cal_zero_sign = usart1.rx[9];
			if(active_cal_channel > 7) active_cal_channel = 7;
			for( uint8_t i = 0 ; i < 8 ; i++ ) {
				char_to_f.char_val[0] = usart1.rx[42+4*i];
				char_to_f.char_val[1] = usart1.rx[43+4*i];
				char_to_f.char_val[2] = usart1.rx[44+4*i];
				char_to_f.char_val[3] = usart1.rx[45+4*i];
				channel[active_cal_channel].cal_point_value[i] = char_to_f.float_val;
			}
			for( uint8_t i = 0 ; i < 8 ; i++ ) {
					channel[active_cal_channel].cal_raw_value[i] =(uint8_t)usart1.rx[10+4*i] *256*256*256 +
									(uint8_t)usart1.rx[11+4*i] *256*256 +
									(uint8_t)usart1.rx[12+4*i] *256 +
									(uint8_t)usart1.rx[13+4*i] ;
			}
			calculate_slopes = 1;
	}
}
void 	 PRESS_ANS_Command 				( void ) {
	//	unsigned char *calibrated;
		usart1.tx[0] = 'A';
		usart1.tx[1] = 'N';
		usart1.tx[2] = 'S';
		for ( uint8_t i = 0 ; i < 4 ; i++ ) {
	//		calibrated = (uint8_t *)&channel[i].calibrated;
			//uint8_t gain_force = (uint8_t)( MAX[i].Gain +  2 );
			uint8_t gain_force =  MAX[resultBinding[i]/6].chGain[resultBinding[i]%6] + 2;	//(uint8_t)( MAX[i].Gain +  2 );

			if	(channel[i].raw_sign == '+' )	/*	ADC'nin datasi (+) ise */
				gain_force = gain_force|0x10;
			usart1.tx[4*i+3] = (uint8_t)( (channel[i].raw&0x00FF0000)>>16);
			usart1.tx[4*i+4] = (uint8_t)( (channel[i].raw&0x0000FF00)>>8 );
			usart1.tx[4*i+5] = (uint8_t)(  channel[i].raw&0x000000FF     );
			usart1.tx[4*i+6]= gain_force;
		}

		usart1.tx[19] = input_status[0] + 0x30;
		usart1.tx[20] = input_status[1] + 0x30;
		usart1.tx[21] = input_status[2] + 0x30;
		usart1.tx[22] = input_status[3] + 0x30;

		usart1.tx[23] = channel_polarity[0] + 0x30;
		usart1.tx[24] = channel_polarity[1] + 0x30;
		usart1.tx[25] = channel_polarity[2] + 0x30;
		usart1.tx[26] = channel_polarity[3] + 0x30;

		usart1.tx[27] = (uint8_t)(stepper_abs_pos >> 16);
		usart1.tx[28] = (uint8_t)(stepper_abs_pos >> 8);
		usart1.tx[29] = (uint8_t)(stepper_abs_pos);

		uint16_t fcrc;
		fcrc = CyclicRedundancyCheck( &usart1.tx[0] , 30 );
		usart1.tx[30] = fcrc%256;
		usart1.tx[31] = fcrc/256;
		usart1.tx_amount = 32;
		ControlUsart1_TransmitData = ControlState_CHECKIT;

		HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );
}

void slope_calculation(uint8_t no){
    u8 validation_1 = 0xFF;
    u8 validation_2 = 0xFF;

    if(cal[no].point_no > 8) cal[no].point_no = 2;
    for(u8 i = 0; i < (cal[no].point_no - 1); i++){
        if(cal[no].real_val[i] < cal[no].real_val[i+1]){
            validation_1 = validation_1 & 0xFF;
        }
        else{
            validation_1 = validation_1 & 0x00;
        }
    }
    if(validation_1 == 0xFF){
        //"calibration is ascending";
    }
    else{
        for(u8 i = 0; i < (cal[no].point_no - 1); i++){
            if(cal[no].real_val[i] > cal[no].real_val[i+1]){
                validation_2 = validation_2 & 0xFF;
            }
            else{
                validation_2 = validation_2 & 0x00;
            }
        }
        if(validation_2 == 0xFF){
            //"calibration is descending";
        }
        else{
            //"calibration is faulty";
        }
    }

    for(u8 i= 0; i < (cal[no].point_no - 1); i++){
        cal[no].slope[i] = ((1.0*(double)(cal[no].assigned_val[i+1]-cal[no].assigned_val[i]))/
                (1.0*(double)(cal[no].real_val[i+1]-cal[no].real_val[i])));
    }
    cal[no].tare_val = 0;

}
void evaluate_calibrated_values	( uint8_t no ) {
    float aux;
    float tared;
    int8_t tared_sign;
    //	*****	*****	*****	*****	*****		TARE	*****	*****	*****	*****	*****		//
    if  ( channel[no].zero_raw_sign == '+' ) {
        if(channel[no].tare_sign == '+'){
            if(channel[no].tare >= channel[no].zero_raw){
                if(channel[no].raw_sign == '+'){
                    if(channel[no].raw >= (channel[no].tare - channel[no].zero_raw)){
                        tared = channel[no].raw - (channel[no].tare - channel[no].zero_raw);
                        tared_sign = '+';
                    }
                    else{
                        tared = (channel[no].tare - channel[no].zero_raw) - channel[no].raw;
                        tared_sign = '-';
                    }
                }
                else{
                    tared = (channel[no].tare - channel[no].zero_raw) + channel[no].raw;
                    tared_sign = '-';
                }
            }
            else{
                if(channel[no].raw_sign == '+'){
                    tared = (channel[no].zero_raw - channel[no].tare) + channel[no].raw;
                    tared_sign = '+';
                }
                else{
                    if(channel[no].raw >= (channel[no].zero_raw - channel[no].tare)){
                        tared = channel[no].raw - (channel[no].zero_raw - channel[no].tare);
                        tared_sign = '-';
                    }
                    else{
                        tared = (channel[no].zero_raw - channel[no].tare) - channel[no].raw;
                        tared_sign = '+';
                    }
                }
            }
        }
        else{
            if(channel[no].raw_sign == '+'){
                tared = (channel[no].tare + channel[no].zero_raw) + channel[no].raw;
                tared_sign = '+';
            }
            else{
                if(channel[no].raw >= (channel[no].zero_raw + channel[no].tare)){
                    tared = channel[no].raw - (channel[no].zero_raw + channel[no].tare);
                    tared_sign = '-';
                }
                else{
                    tared = (channel[no].zero_raw + channel[no].tare) - channel[no].raw;
                    tared_sign = '+';
                }
            }
        }
    }
    else{
        if(channel[no].tare_sign == '+'){
            if(channel[no].raw_sign == '+'){
                if(channel[no].raw >= (channel[no].zero_raw + channel[no].tare)){
                    tared = channel[no].raw - (channel[no].zero_raw + channel[no].tare);
                    tared_sign = '+';
                }
                else{
                    tared = (channel[no].zero_raw + channel[no].tare) - channel[no].raw;
                    tared_sign = '-';
                }
            }
            else{
                tared = channel[no].raw + (channel[no].zero_raw + channel[no].tare);
                tared_sign = '-';
            }
        }
        else{
            if(channel[no].tare >= channel[no].zero_raw){
                if(channel[no].raw_sign == '+'){
                    tared = (channel[no].tare - channel[no].zero_raw) + channel[no].raw;
                    tared_sign = '+';
                }
                else{
                    if(channel[no].raw >= (channel[no].tare - channel[no].zero_raw)){
                        tared = channel[no].raw - (channel[no].tare - channel[no].zero_raw);
                        tared_sign = '-';
                    }
                    else{
                        tared = (channel[no].tare - channel[no].zero_raw) - channel[no].raw;
                        tared_sign = '+';
                    }
                }
            }
            else{
                if(channel[no].raw_sign == '+'){
                    if(channel[no].raw >= (channel[no].zero_raw - channel[no].tare)){
                        tared = channel[no].raw - (channel[no].zero_raw - channel[no].tare);
                        tared_sign = '+';
                    }
                    else{
                        tared = (channel[no].zero_raw - channel[no].tare) - channel[no].raw;
                        tared_sign = '-';
                    }
                }
                else{
                    tared = (channel[no].zero_raw - channel[no].tare) + channel[no].raw;
                    tared_sign = '-';
                }
            }
        }
    }
    //	*****	*****	*****	*****	*****		*****	*****	*****	*****	*****	*****		//
    if	( channel[no].zero_raw_sign == '+' ) {
        if(tared_sign == '+'){         // if raw data is postive
            switch (channel[no].point_number) {
                case 8:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (tared - channel[no].zero_raw);
                        channel[no].calibrated  = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])&&(tared <= channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    else if ((tared > channel[no].cal_raw_value[4])&&(tared <= channel[no].cal_raw_value[5])){
                        aux  = (tared - channel[no].cal_raw_value[4]);
                        channel[no].calibrated  = channel[no].slope[4]*(aux) + channel[no].cal_point_value[4];
                    }
                    else if ((tared > channel[no].cal_raw_value[5])&&(tared <= channel[no].cal_raw_value[6])){
                        aux  = (tared - channel[no].cal_raw_value[5]);
                        channel[no].calibrated  = channel[no].slope[5]*(aux) + channel[no].cal_point_value[5];
                    }
                    else if ((tared > channel[no].cal_raw_value[6])){
                        aux  = (tared - channel[no].cal_raw_value[6]);
                        channel[no].calibrated  = channel[no].slope[6]*(aux) + channel[no].cal_point_value[6];
                    }
                    break;
                case 7:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (tared - channel[no].zero_raw);
                        channel[no].calibrated  = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])&&(tared <= channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    else if ((tared > channel[no].cal_raw_value[4])&&(tared <= channel[no].cal_raw_value[5])){
                        aux  = (tared - channel[no].cal_raw_value[4]);
                        channel[no].calibrated  = channel[no].slope[4]*(aux) + channel[no].cal_point_value[4];
                    }
                    else if ((tared > channel[no].cal_raw_value[5])){
                        aux  = (tared - channel[no].cal_raw_value[5]);
                        channel[no].calibrated  = channel[no].slope[5]*(aux) + channel[no].cal_point_value[5];
                    }
                    break;
                case 6:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (tared - channel[no].zero_raw);
                        channel[no].calibrated  = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])&&(tared <= channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    else if ((tared > channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[4]);
                        channel[no].calibrated  = channel[no].slope[4]*(aux) + channel[no].cal_point_value[4];
                    }
                    break;
                case 5:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (tared - channel[no].zero_raw);
                        channel[no].calibrated  = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    break;
                case 4:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (tared - channel[no].zero_raw);
                        channel[no].calibrated  = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    break;
                case 3:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (tared - channel[no].zero_raw);
                        channel[no].calibrated  = channel[no].slope[0]*(aux);
                    }
                    else {
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    break;
                case 2:
                    aux  = (tared - channel[no].zero_raw);
                    channel[no].calibrated = channel[no].slope[0]*(aux);
                    break;
                default:
                    break;
            }
        }
        else{                                                // if raw data is negative
            aux  = -(channel[no].zero_raw + tared);
            channel[no].calibrated  = channel[no].slope[0]*(aux);
        }
    }
    else{
        if(tared_sign == '+'){         // if raw data is postive
            switch (channel[no].point_number) {
                case 8:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (channel[no].zero_raw + tared);
                        channel[no].calibrated = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])&&(tared <= channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    else if ((tared > channel[no].cal_raw_value[4])&&(tared <= channel[no].cal_raw_value[5])){
                        aux  = (tared - channel[no].cal_raw_value[4]);
                        channel[no].calibrated  = channel[no].slope[4]*(aux) + channel[no].cal_point_value[4];
                    }
                    else if ((tared > channel[no].cal_raw_value[5])&&(tared <= channel[no].cal_raw_value[6])){
                        aux  = (tared - channel[no].cal_raw_value[5]);
                        channel[no].calibrated  = channel[no].slope[5]*(aux) + channel[no].cal_point_value[5];
                    }
                    else if ((tared > channel[no].cal_raw_value[6])){
                        aux  = (tared - channel[no].cal_raw_value[6]);
                        channel[no].calibrated  = channel[no].slope[6]*(aux) + channel[no].cal_point_value[6];
                    }
                    break;
                case 7:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (channel[no].zero_raw + tared);
                        channel[no].calibrated = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])&&(tared <= channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    else if ((tared > channel[no].cal_raw_value[4])&&(tared <= channel[no].cal_raw_value[5])){
                        aux  = (tared - channel[no].cal_raw_value[4]);
                        channel[no].calibrated  = channel[no].slope[4]*(aux) + channel[no].cal_point_value[4];
                    }
                    else if ((tared > channel[no].cal_raw_value[5])){
                        aux  = (tared - channel[no].cal_raw_value[5]);
                        channel[no].calibrated  = channel[no].slope[5]*(aux) + channel[no].cal_point_value[5];
                    }
                    break;
                case 6:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (channel[no].zero_raw + tared);
                        channel[no].calibrated = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])&&(tared <= channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    else if ((tared > channel[no].cal_raw_value[4])){
                        aux  = (tared - channel[no].cal_raw_value[4]);
                        channel[no].calibrated  = channel[no].slope[4]*(aux) + channel[no].cal_point_value[4];
                    }
                    break;
                case 5:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (channel[no].zero_raw + tared);
                        channel[no].calibrated = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])&&(tared <= channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    else if ((tared > channel[no].cal_raw_value[3])){
                        aux  = (tared - channel[no].cal_raw_value[3]);
                        channel[no].calibrated  = channel[no].slope[3]*(aux) + channel[no].cal_point_value[3];
                    }
                    break;
                case 4:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (channel[no].zero_raw + tared);
                        channel[no].calibrated = channel[no].slope[0]*(aux);
                    }
                    else if ((tared > channel[no].cal_raw_value[1])&&(tared <= channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    else if ((tared > channel[no].cal_raw_value[2])){
                        aux  = (tared - channel[no].cal_raw_value[2]);
                        channel[no].calibrated  = channel[no].slope[2]*(aux) + channel[no].cal_point_value[2];
                    }
                    break;
                case 3:
                    if (tared <= channel[no].cal_raw_value[1]){
                        aux  = (channel[no].zero_raw + tared);
                        channel[no].calibrated = channel[no].slope[0]*(aux);
                    }
                    else {
                        aux  = (tared - channel[no].cal_raw_value[1]);
                        channel[no].calibrated  = channel[no].slope[1]*(aux) + channel[no].cal_point_value[1];
                    }
                    break;
                case 2:
                    aux  = (channel[no].zero_raw + tared);
                    channel[no].calibrated = channel[no].slope[0]*(aux);
                    break;
                default:
                    break;
            }
        }
        else{                                                // if raw data is negative
            aux  = (channel[no].zero_raw - tared);
            channel[no].calibrated = channel[no].slope[0]*(aux);
        }
    }
}
