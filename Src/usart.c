#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "max11254.h"

uint8_t u1_ctrl1;
uint8_t u2_ctrl1;
uint8_t u3_ctrl1;
uint8_t u4_ctrl1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;

UART_HandleTypeDef huart2;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef  hdma_uart4_tx;

uint8_t usarttx[USART1_TX_ARRAY_SIZE];
uint8_t usartrx[USART1_RX_ARRAY_SIZE];
uint8_t rx_indeks;
uint8_t casual_rx_data;
uint8_t usart_tx_size;
uint8_t TxAmound;

uint32_t  	buffer_clear_timer;
uint8_t 	buffer_cleared;	

union 	_char_to_f {
	float float_val;
	unsigned char char_val[4];
};
union 	_char_to_f char_to_f;
struct  chan channel[8];
struct _usart usart2;

uint8_t active_cal_channel,calculate_slopes;	//	unsigned char->uint8_t
uint8_t in_calibration; 

/* UART4 init function */
void MX_UART4_Init			( void ) {

  huart4.Instance 			= UART4;
  huart4.Init.BaudRate 		= 115200;
  huart4.Init.WordLength 	= UART_WORDLENGTH_8B;
  huart4.Init.StopBits 		= UART_STOPBITS_1;
  huart4.Init.Parity 		= UART_PARITY_NONE;
  huart4.Init.Mode 			= UART_MODE_TX;
  huart4.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
  huart4.Init.OverSampling 	= UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  usartrx[ 0 ] = 10;
  HAL_UART_Transmit( &huart4 , &usartrx[0] , 1 , 10 );
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

	HAL_UART_Receive_DMA ( &huart1 , &casual_rx_data , 1 );
	calculate_slopes = 0;
	buffer_cleared = 0;	
}
void HAL_UART_MspInit		( UART_HandleTypeDef* uartHandle ) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if			( uartHandle->Instance == UART4  ) {
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();
  
    /**UART4 GPIO Configuration    
    PA0-WKUP     ------> UART4_TX 
    */
    GPIO_InitStruct.Pin 		= GPIO_PIN_0;
    GPIO_InitStruct.Mode 		= GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull 		= GPIO_PULLUP;
    GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate 	= GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_TX Init */
//    hdma_uart4_tx.Instance 					= DMA1_Stream4;
//    hdma_uart4_tx.Init.Channel 				= DMA_CHANNEL_4;
//    hdma_uart4_tx.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
//    hdma_uart4_tx.Init.PeriphInc 				= DMA_PINC_DISABLE;
//    hdma_uart4_tx.Init.MemInc 				= DMA_MINC_ENABLE;
//    hdma_uart4_tx.Init.PeriphDataAlignment	= DMA_PDATAALIGN_BYTE;
//    hdma_uart4_tx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
//    hdma_uart4_tx.Init.Mode 					= DMA_NORMAL;
//    hdma_uart4_tx.Init.Priority 				= DMA_PRIORITY_LOW;
//    hdma_uart4_tx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;
//    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
//      _Error_Handler(__FILE__, __LINE__);

//    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart4_tx);

    /* UART4 interrupt Init */
//    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  else if ( uartHandle->Instance == USART1 ) {
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin  = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
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

    /* USART1_TX Init */
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

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if ( uartHandle->Instance == USART2 ) {
    __HAL_RCC_USART2_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA2     ------> USART1_TX
    PA3     ------> USART1_RX
    */
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

  if			( uartHandle -> Instance == UART4  ) {
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();
  
    /**UART4 GPIO Configuration    
    PA0-WKUP     ------> UART4_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
  else if ( uartHandle -> Instance == USART1 ) {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if ( uartHandle -> Instance == USART2 ) {
		/* Peripheral clock disable */
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

		abs_pos = 65536 * usart2.rx[3] + 256 * usart2.rx[4] + usart2.rx[5];
		usart2.rx_indeks = 0;
		for (uint8_t i = 0; i < USART4_RX_ARRAY_SIZE ; i++) {
			usart2.rx[i] = 0;
		}
	}
}

/**
  * @brief  CRC ==> Cyclic Redundancy Check(D�ng�sel Artiklik Denetimi) fonksiyonu
  * @param  data      : CRC ye g�nderilecek dizi 
  * @param  length		: CRC ye g�nderilen data elemanlarindan ka� tanesinin CRC ye taabii tutulacagi
  * @retval 32 bitlik crc sonucu
  */
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
/**
  * @brief  Gelen usartrx datalari arasinda CONV ile baslayan komutu arar
  * @param  capture_index : gelen usartrx datalarinda komutu geldigini algiladigi indis numarasi   
  * @retval [ void ]
  */
void 	 UsartReceiveData_SearchCommand ( void ) {
	//float *channel_floats[8];
	if 		( usartrx[0]=='C' && usartrx[1]=='O' && usartrx[2]=='N' && usartrx[3]=='V' ) {
			PRESS_CONV_CommandOperating( );
	}
	else if ( usartrx[0]=='G' && usartrx[1]=='A' && usartrx[2]=='I' && usartrx[3]=='N' ) {
			PRESS_GAIN_CommandOperating( );
	}
	else if ( usartrx[0]=='T' && usartrx[1]=='A' && usartrx[2]=='R' && usartrx[3]=='E' ) {
			PRESS_TARE_CommandOperating();
	}
	else if ( usartrx[0]=='P' && usartrx[1]=='R' && usartrx[2]=='I' && usartrx[3]=='N' && usartrx[4]=='T' ) {
			PRESS_PRINT_CommandOperating();			
	}
	else if ( usartrx[0]=='C' && usartrx[1]=='A' && usartrx[2]=='L' && usartrx[3]=='S' && usartrx[4]=='E' && usartrx[5]=='N' && usartrx[6]=='D' )  {
		PRESS_CALSEND_CommandOperating();
	}
	else if	( usartrx[0]=='P' && usartrx[1]=='L' && usartrx[2]=='R' && usartrx[3]=='T' ) {
		uint8_t chn  = usartrx[4] - 0x30;
		uint8_t plrt = usartrx[5] - 0x30;

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
	else if ( usartrx[0]=='M' && usartrx[1]=='A' && usartrx[2]=='X' && usartrx[3]=='A' && usartrx[4]=='D' && usartrx[5]=='C' ) {
		if ( (usartrx[6]<=3) && (usartrx[7]<=1) && (usartrx[8]<=15) ) {
			Max11254_SoftwareReset 			( usartrx[6] + 1 );
			MAX[ usartrx[6] ].polarity 		= usartrx[ 7 ];
			MAX[ usartrx[6] ].RateNumber 	= usartrx[ 8 ];

			for ( uint8_t ch=0 ; ch<6 ; ch++ )	//	All channel cleer Read bit
				MAX[usartrx[6]].chRead[ch] = CH_NotREAD;
			for ( uint8_t ch=0 ; ch<6 ; ch++ ) {
				if ( usartrx[ 9+ch*2 ] == 1 ) {
					if ( usartrx[ 10+ch*2 ] <= 8 ) {
						MAX[usartrx[6]].chRead[5-ch] = CH_READ;
						MAX[usartrx[6]].chGain[5-ch] = (GAIN)usartrx[10+ch*2];
					}
					else {
						MAX[usartrx[6]].chRead[5-ch] = CH_NotREAD;
						MAX[usartrx[6]].chGain[5-ch] = GAIN_Disb;
					}
				}
				else {
					MAX[usartrx[6]].chRead[5-ch] = CH_NotREAD;
					MAX[usartrx[6]].chGain[5-ch] = GAIN_Disb;
			}

			for ( uint8_t i=0 ; i<6 ; i++ ) {
				if ( MAX[usartrx[6]].chRead[i] == CH_READ ) {
					MAX[usartrx[6]].rank = i;
				}
			}
			uint8_t chs = 	(uint8_t)( MAX[usartrx[6]].chRead[5] << 5 ) |
							(uint8_t)( MAX[usartrx[6]].chRead[4] << 4 ) |
							(uint8_t)( MAX[usartrx[6]].chRead[3] << 3 ) |
							(uint8_t)( MAX[usartrx[6]].chRead[2] << 2 ) |
							(uint8_t)( MAX[usartrx[6]].chRead[1] << 1 ) |
							(uint8_t)( MAX[usartrx[6]].chRead[0] );

			Max11254_SequencerMode2_EntryUart(  usartrx[6]+1,
												(ChmapConvChannels)chs ,
												MAX[usartrx[6]].chGain[MAX[usartrx[6]].rank] );

			Max11254_ConversionCommand( usartrx[6]+1 , MAX[usartrx[6]].RateNumber|COMMAND_MODE_SEQUENCER );

//			Max11254_ChannelMap_Set( chooseMax , (ChmapConvChannels)x );
//			Max11254_ConversionCommand( chooseMax , MAX[usartrx[6]].RateNumber|COMMAND_MODE_SEQUENCER );
//			OperatingMaxExtiRdbyControl( (MaxDevice)chooseMax );
		}
		u1_ctrl1 = Max11254_Read1byte( MAX_1 , MAX11254_REG_CTRL1 );
		u2_ctrl1 = Max11254_Read1byte( MAX_2 , MAX11254_REG_CTRL1 );
		u3_ctrl1 = Max11254_Read1byte( MAX_3 , MAX11254_REG_CTRL1 );
		u4_ctrl1 = Max11254_Read1byte( MAX_4 , MAX11254_REG_CTRL1 );
		}
	}
	else if	( usartrx[0]=='G' && usartrx[1]=='P' && usartrx[2]=='I' && usartrx[3]=='O' ) {
		if ( usartrx[4] <= 3 )
			Max11254_GPIOSetting( usartrx[4]+1 , usartrx[5] );
	}
	else if ( usartrx[0]=='R' && usartrx[1]=='E' && usartrx[2]=='S' && usartrx[3]=='U' && usartrx[4]=='L' && usartrx[5]=='T' ) {
		if ( usartrx[6] <= 23 )
						resultBinding[ 0 ] = usartrx[ 6 ];
		if ( usartrx[7] <= 23 )
						resultBinding[ 1 ] = usartrx[ 7 ];
		if ( usartrx[8] <= 23 )
						resultBinding[ 2 ] = usartrx[ 8 ];
		if ( usartrx[9] <= 23 )
						resultBinding[ 3 ] = usartrx[ 9 ];
	}
}
void 	 PRESS_CONV_CommandOperating	( void ) {
	if 			( usartrx[4] == 0x01 ) {
		uint32_t ServoSpeed = (uint32_t)( ( uint32_t )( usartrx[5]*65536 )
										+ ( uint32_t )( usartrx[6]*256 )
										+ ( uint32_t )( usartrx[7]) );
		Timer3_AutoConsolidation_SpecialFunc( ServoSpeed );
	}
	//	0x30[Hex] = 48[Dec]	,	0x31[Hex] = 49[Dec]
	if (usartrx[8] == 0x30 )	Electromechanic_RELAY_OFF_AutoManual;
	else if(usartrx[8] == 0x31 ) 	Electromechanic_RELAY_ON_AutoManual;

	//	0x30[Hex] = 48[Dec]	,	0x31[Hex] = 49[Dec]
	if (usartrx[9] == 0x30 ) 	Electromechanic_RELAY_OFF_StartStop;
	else if (usartrx[9] == 0x31 ) 	Electromechanic_RELAY_ON_StartStop;

	if (usartrx[10] == 0x01 ) {
		Electromechanic_ServoStop;
		TIM3->CCR1= 0;
	}
	if (usartrx[11] == 0x01 ) Electromechanic_ServoStart;
	if (usartrx[12] == 0x01 ) Electromechanic_ServoForward;
	if (usartrx[13] == 0x01 ) Electromechanic_ServoReverse;

	PRESS_ANS_Command();
}
void	 PRESS_GAIN_CommandOperating	( void ) {
	uint8_t usart_gain = usartrx[4] - 50;		//	48;
	uint8_t usart_adc  = usartrx[5] - 48;	
	if ( usart_gain <= 8 ){
		MAX[usart_adc].Gain = (GAIN)( usart_gain );
		MAX[usart_adc].chGain[4] = (GAIN)( usart_gain );
	}
	Max11254_PGAGain_Set( (MaxDevice)(usart_adc+1) , MAX[usart_adc].Gain );
	Max11254_ConversionCommand( (MaxDevice)(usart_adc+1) , MAX[usart_adc].RateNumber|COMMAND_MODE_SEQUENCER );
	//Max11254_SequencerMode2_Entry( MAX_1 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[usart_adc].chGain[4] );

} 
void 	 PRESS_TARE_CommandOperating	( void ) {
	if(usartrx[4] < 4){
			channel[usartrx[4]].tare = channel[usartrx[4]].raw;
			channel[usartrx[4]].tare_sign = channel[usartrx[4]].raw_sign;
	}
}
void 	 PRESS_PRINT_CommandOperating	( void ) {
		static uint8_t print_count;
		print_count++;
		if ( print_count == 22 ) {
			usartrx[ 37 ] = 0x0A;
			usartrx[ 38 ] = 0x1B;
			usartrx[ 39 ] = 0x69;
			HAL_UART_Transmit( &huart4 , &usartrx[5] , 35 , 15 );
			print_count = 0;
		}
		else
			HAL_UART_Transmit( &huart4 , &usartrx[5] , 32 , 10 );
}
void	 PRESS_CALSEND_CommandOperating ( void ) {
//		else if ( usartrx[capture_indeks-77]=='C'&usartrx[capture_indeks-76]=='A'&usartrx[capture_indeks-75]=='L'&usartrx[capture_indeks-74]=='S'
//						&usartrx[capture_indeks-73]=='E'&usartrx[capture_indeks-72]=='N'&usartrx[capture_indeks-71]=='D' )
	uint16_t fcrc;								//	unsigned int fcrc,i;
	uint8_t  crc_high , crc_low;	//	unsigned char crc_high,crc_low;  
	fcrc = CyclicRedundancyCheck((uint8_t*)usartrx,74);
	crc_high = (fcrc)%256;
	crc_low = (fcrc)/256;
	if(((uint8_t)usartrx[74] == crc_high)&&((uint8_t)usartrx[75] == crc_low)){
			active_cal_channel = usartrx[7];
			channel[active_cal_channel].point_number = usartrx[8];
			channel[active_cal_channel].cal_zero_sign = usartrx[9];
			if(active_cal_channel > 7) active_cal_channel = 7;
			for( uint8_t i = 0 ; i < 8 ; i++ ) {
				char_to_f.char_val[0] = usartrx[42+4*i];
				char_to_f.char_val[1] = usartrx[43+4*i];
				char_to_f.char_val[2] = usartrx[44+4*i];
				char_to_f.char_val[3] = usartrx[45+4*i];
				channel[active_cal_channel].cal_point_value[i] = char_to_f.float_val;
			}
			for( uint8_t i = 0 ; i < 8 ; i++ ) {
					channel[active_cal_channel].cal_raw_value[i] =(uint8_t)usartrx[10+4*i] *256*256*256 +
									(uint8_t)usartrx[11+4*i] *256*256 +
									(uint8_t)usartrx[12+4*i] *256 +
									(uint8_t)usartrx[13+4*i] ;
			}
			calculate_slopes = 1;
	}
}
void 	 PRESS_ANS_Command 				( void ) {
	//	unsigned char *calibrated;
		usarttx[0] = 'A';
		usarttx[1] = 'N';
		usarttx[2] = 'S';
		for ( uint8_t i = 0 ; i < 4 ; i++ ) {
	//		calibrated = (uint8_t *)&channel[i].calibrated;
			//uint8_t gain_force = (uint8_t)( MAX[i].Gain +  2 );
			uint8_t gain_force =  MAX[resultBinding[i]/6].chGain[resultBinding[i]%6] + 2;	//(uint8_t)( MAX[i].Gain +  2 );

			if	(channel[i].raw_sign == '+' )	/*	ADC'nin datasi (+) ise */
				gain_force = gain_force|0x10;
			usarttx[4*i+3] = (uint8_t)( (channel[i].raw&0x00FF0000)>>16);
			usarttx[4*i+4] = (uint8_t)( (channel[i].raw&0x0000FF00)>>8 );
			usarttx[4*i+5] = (uint8_t)(  channel[i].raw&0x000000FF     );
			usarttx[4*i+6]= gain_force;
		}
		int32_t encoder_value = Timer1_CalculateEncoderValue();
		if ( encoder_value < 0 ) {
			usarttx[ 22 ] = 0x00;
			encoder_value = encoder_value* -1;
		}
		else
			usarttx[ 22 ] = 0x10;

		usarttx[ 19 ] = (encoder_value/65536)%256;	//
		usarttx[ 20 ] = (encoder_value/256)%256;	//
		usarttx[ 21 ] =  encoder_value%256;			//

		usarttx[23] = channel_polarity[0] + 0x30;
		usarttx[24] = channel_polarity[1] + 0x30;
		usarttx[25] = channel_polarity[2] + 0x30;
		usarttx[26] = channel_polarity[3] + 0x30;

		uint16_t fcrc;
		fcrc = CyclicRedundancyCheck( &usarttx[0] , 27 );
		usarttx[27] = fcrc%256;
		usarttx[28] = fcrc/256;
		TxAmound = 29;
		ControlUsart1_TransmitData = ControlState_CHECKIT;

		HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );

}

/*	A.C.AKINCA eklemeleri	*/
void slope_calculation			( uint8_t i  ) {
    double aux_raw;
    double aux_cal;
    if(channel[i].cal_raw_value[0] != channel[i].cal_raw_value[1]){
        if(channel[i].cal_zero_sign == '+'){
            channel[i].slope[0] = ((1.0*(double)(channel[i].cal_point_value[1]-channel[i].cal_point_value[0]))/(1.0*(double)(channel[i].cal_raw_value[1]-channel[i].cal_raw_value[0])));
        }
        else{
            channel[i].slope[0] = ((1.0*(double)(channel[i].cal_point_value[1]-channel[i].cal_point_value[0]))/(1.0*(double)(channel[i].cal_raw_value[1]+channel[i].cal_raw_value[0])));
        }
    }
    if(channel[i].cal_raw_value[1] != channel[i].cal_raw_value[2]){
        channel[i].slope[1] = ((1.0*(double)(channel[i].cal_point_value[2]-channel[i].cal_point_value[1]))/(1.0*(double)(channel[i].cal_raw_value[2]-channel[i].cal_raw_value[1])));
    }
    if(channel[i].cal_raw_value[2] != channel[i].cal_raw_value[3]){
        channel[i].slope[2] = ((1.0*(double)(channel[i].cal_point_value[3]-channel[i].cal_point_value[2]))/(1.0*(double)(channel[i].cal_raw_value[3]-channel[i].cal_raw_value[2])));
    }
    if(channel[i].cal_raw_value[3] != channel[i].cal_raw_value[4]){
        channel[i].slope[3] = ((1.0*(double)(channel[i].cal_point_value[4]-channel[i].cal_point_value[3]))/(1.0*(double)(channel[i].cal_raw_value[4]-channel[i].cal_raw_value[3])));
    }
    if(channel[i].cal_raw_value[4] != channel[i].cal_raw_value[5]){
        channel[i].slope[4] = ((1.0*(double)(channel[i].cal_point_value[5]-channel[i].cal_point_value[4]))/(1.0*(double)(channel[i].cal_raw_value[5]-channel[i].cal_raw_value[4])));
    }
    if(channel[i].cal_raw_value[5] != channel[i].cal_raw_value[6]){
        channel[i].slope[5] = ((1.0*(double)(channel[i].cal_point_value[6]-channel[i].cal_point_value[5]))/(1.0*(double)(channel[i].cal_raw_value[6]-channel[i].cal_raw_value[5])));
    }
    if(channel[i].cal_raw_value[6] != channel[i].cal_raw_value[7]){
        channel[i].slope[6] = ((1.0*(double)(channel[i].cal_point_value[7]-channel[i].cal_point_value[6]))/(1.0*(double)(channel[i].cal_raw_value[7]-channel[i].cal_raw_value[6])));
    }

    if(channel[i].cal_point_value[0] == 0){
        channel[i].zero_raw = channel[i].cal_raw_value[0];
        channel[i].zero_raw_sign = channel[i].cal_zero_sign;
    }
    else{
        if((double)channel[i].cal_raw_value[0] >= (((double)channel[i].cal_point_value[0])/channel[i].slope[0])){
            aux_raw = (double)channel[i].cal_raw_value[0] - (((double)channel[i].cal_point_value[0])/channel[i].slope[0]);
        }
        else{
            aux_raw = (((double)channel[i].cal_point_value[0])/channel[i].slope[0]) - (double)channel[i].cal_raw_value[0];
        }

        aux_cal = (double)channel[i]. cal_point_value[0] - (((double)channel[i].cal_raw_value[0]) * channel[i].slope[0]);
        if(aux_cal >= 0){
            channel[i].zero_raw_sign = '-';
        }
        else{
            channel[i].zero_raw_sign = '+';
        }
        channel[i].zero_raw = aux_raw;
    }

    channel[i].tare = channel[i].zero_raw;
    channel[i].tare_sign = channel[i].zero_raw_sign;
}
void evaluate_calibrated_values	( uint8_t no ) {
    float aux;
    float tared;
    int8_t tared_sign; 
    in_calibration = 1;
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
    in_calibration = 0;
}
