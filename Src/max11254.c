#include "stm32f4xx_hal.h"
#include "max11254.h"
#include "spi.h"
 
ConnectedADC 	MAX[4];

void		Max11254_GPIOSetting			( MaxDevice ChooseMax , uint8_t fourGpio );
void 		Max11254_PolaritySelect			( MaxDevice ChooseMax , uint8_t polarity );
void 		Max11254_SPSSelect				( MaxDevice ChooseMax , uint8_t spsRate  );
void		Max11254_NextChannelOperations	( MaxDevice ChooseMax );

void 		Max11254_ConversionCommand 		( MaxDevice ChooseMax , uint8_t command ) {
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT , MAX11254_3_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		default:
  	break;	
	}
} 
void 		Max11254_Write1byte 			( MaxDevice ChooseMax , uint8_t Adr , uint8_t  Data ) {
	uint8_t spi_tx[2];
	spi_tx[0] = (0xC0|(Adr<<1)); 
	spi_tx[1] = Data;
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   2   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   2   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   2   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_SET   );
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   2   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		default:
  	break;	
	}
}
void		Max11254_Write2byte 			( MaxDevice ChooseMax , uint8_t Adr , uint16_t Data ) {
	uint8_t spi_tx[3];
	spi_tx[0] = (0xC0|(Adr<<1)); 
	spi_tx[1] = (Data&0xFF00)>>8;		
	spi_tx[2] = (Data&0x00FF);
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   3   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   3   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   3   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_SET   );
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   3   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		default:
  	break;	
	}
}
void 		Max11254_Write3byte 			( MaxDevice ChooseMax , uint8_t Adr , uint32_t Data ) {
	uint8_t spi_tx[4];
	spi_tx[0] = (0xC0|(Adr<<1)); 
	spi_tx[1] = (Data&0x00FF0000) >> 16;
	spi_tx[2] = (Data&0x0000FF00) >> 8;
	spi_tx[3] =  Data&0x000000FF;
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   4   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   4   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   4   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_SET   );
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx[0] ,   4   , TIMEOUT_HAL_SPI_TRANSMIT 	 );
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		default:
  	break;	
	}
}
uint8_t 	Max11254_Read1byte 				( MaxDevice ChooseMax , uint8_t Adr ){
  uint8_t spi_tx = (0xC1|(Adr<<1));
	uint8_t spi_rx = 0;
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT , MAX11254_3_CS_PIN  , GPIO_PIN_SET   );
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx   , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		default:
  	break;	
	}
	return spi_rx;
}
uint16_t 	Max11254_Read2byte 			   	( MaxDevice ChooseMax , uint8_t Adr ) {
	uint8_t spi_tx = (0xC1|(Adr<<1));
	uint8_t spi_rx[2] = { 0 };
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 2 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 2 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 2 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_SET   );
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 2 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		default:
  	break;	
	}
	return ( (spi_rx[0]<<8) | (spi_rx[1]) );
}
uint32_t 	Max11254_Read3byte				( MaxDevice ChooseMax , uint8_t Adr ) {
  uint8_t spi_tx = (0xC1|(Adr<<1));
	uint8_t spi_rx[3] = { 0 };
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT  , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 3 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 3 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT  , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 3 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT  , MAX11254_3_CS_PIN  , GPIO_PIN_SET   );
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &spi_tx    , 1 , TIMEOUT_HAL_SPI_TRANSMIT );
			HAL_SPI_Receive  ( &hspi2 , &spi_rx[0] , 3 , TIMEOUT_HAL_SPI_TRANSMIT ); 
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT  , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	 );
			break;
		}
		default:
  	break;	
	}	
	return ( (spi_rx[0]<<16) | (spi_rx[1]<<8) | (spi_rx[2]) );
}
void 		Max11254_HardwareReset 			( void ) {
		MAX11254s_RSTB_LOW;
		HAL_Delay( 200 );
		MAX11254s_RSTB_HIGH;
}
void 		Max11254_SoftwareReset 			( MaxDevice ChooseMax ) {
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_CONTSC_ENABLE|CTRL1_PDx_RESET );
	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_POWER_DOWN );								
	HAL_Delay ( 100 );
}
void 		Max11254_SelfCalibration 		( MaxDevice ChooseMax ) {
	uint8_t reg_data = 0;
	reg_data = Max11254_Read1byte ( ChooseMax , MAX11254_REG_CTRL1 );
	reg_data &= 0x3F;
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , reg_data );	//	select self-calibration mode	
	reg_data = Max11254_Read1byte ( ChooseMax , MAX11254_REG_SEQ );
	reg_data &= 0x07;
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ , reg_data );		//	select single conversion	
	Max11254_ConversionCommand		(	ChooseMax , COMMAND_MODE_CALIBRATION|COMMAND_RATE_0 );
	HAL_Delay( 250 );    	
}
void 		MAX11254_SystemZeroCalibration  ( MaxDevice ChooseMax , SeqConvChannel Ch ) {
//  Max11254_Write1byte( ChooseMax ,MAX11254_REG_CTRL1 , CTRL1_CAL_OFFSETCAL|CTRL1_SCYCLE ); // 0x42=CTRL1_CAL_OFFSETCAL|CTRL1_SCYCLE
//	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_CALIBRATION|COMMAND_RATE_0 );
	uint8_t reg_data = 0;
	reg_data = Max11254_Read1byte ( ChooseMax , MAX11254_REG_CTRL1 );
	reg_data&=0x3F;
	reg_data+=0x40;
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , reg_data );		//	select system offset calibration mode
	
	reg_data = Max11254_Read1byte ( ChooseMax , MAX11254_REG_SEQ );
	reg_data&=0xE7;
	reg_data|=Ch;
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ , reg_data );			//	select single conversion and input channel
	Max11254_ConversionCommand (	ChooseMax , COMMAND_MODE_CALIBRATION|COMMAND_RATE_0 );
	HAL_Delay(150);
}
void 		MAX11254_SystemFullCalibration	( MaxDevice ChooseMax , SeqConvChannel Ch ) {
//  Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_CAL_FULLSCALE|CTRL1_SCYCLE );
//	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_CALIBRATION|COMMAND_RATE_0 );
	uint8_t reg_data = 0;
	reg_data = Max11254_Read1byte ( ChooseMax , MAX11254_REG_CTRL1 );
	reg_data&=0x3F;
	reg_data+=0x80;
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , reg_data );
	
	reg_data = Max11254_Read1byte ( ChooseMax , MAX11254_REG_SEQ );
	reg_data&=0x07;
	reg_data|=Ch;
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ , reg_data );
	Max11254_ConversionCommand (	ChooseMax , COMMAND_MODE_CALIBRATION|COMMAND_RATE_0 );	
	HAL_Delay(150);
}
void 		Max11254_SystemCalibration 		( MaxDevice ChooseMax , SeqConvChannel Ch ) {
	uint8_t reg_ctrl1 = Max11254_Read1byte ( ChooseMax , MAX11254_REG_CTRL1 );
	uint8_t	reg_seq		= Max11254_Read1byte ( ChooseMax , MAX11254_REG_SEQ		);
	MAX11254_SystemZeroCalibration( ChooseMax , Ch );	//	Max11254_SystemOffsetCalibration( ChooseMax );
	MAX11254_SystemFullCalibration( ChooseMax , Ch );	//	Max11254_SystemFullScaleCalibration( ChooseMax );
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , reg_ctrl1 );
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ		, reg_seq );
}
void Max11254_PGAGain_Set			( MaxDevice ChooseMax , GAIN Gain ) {
	if (Gain == GAIN_Disb || Gain == GAIN_x1 || Gain == GAIN_x2 || Gain == GAIN_x4 || Gain == GAIN_x8
				|| Gain == GAIN_x16 || Gain == GAIN_x32 || Gain == GAIN_x64 || Gain == GAIN_x128) {
		MAX[ChooseMax - 1].Gain = Gain;
		uint8_t Max_Reg_Gain = 0;
		uint8_t CTRL2_Reg_Val = Max11254_Read1byte(ChooseMax, MAX11254_REG_CTRL2);
		CTRL2_Reg_Val &= 0xF0;
		if ( Gain == GAIN_Disb )
			Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2 , CTRL2_PGA_DISABLE|CTRL2_Reg_Val|Max_Reg_Gain );
		else {
			Max_Reg_Gain = (uint8_t)( ((uint8_t)Gain) - 1 );
			Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2 , CTRL2_PGA_ENABLE |CTRL2_Reg_Val|Max_Reg_Gain );
		}
	}
}
void 		Max11254_ChannelMap_Set			( MaxDevice ChooseMax , ChmapConvChannels Chs ) {
	uint8_t OrderCh = 1;
	for( uint8_t ch=0 ; ch<6 ;ch++ ) {
		if ( (Chs&(0x01<<ch))>>ch == 0x01 ) {
			uint32_t reg_val =( (((uint32_t)OrderCh)<<2) | 0x00000002 ); 
		  reg_val = reg_val << ((ch%3)*8);
			OrderCh++;
			switch ( ch ) {
				uint32_t chmap_val;
				case 0 : {			
					chmap_val = Max11254_Read3byte( ChooseMax , MAX11254_REG_CHMAP0 );
					chmap_val&=0x00FFFF00;
					Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP0 , chmap_val|reg_val );
//					MAX[ChooseMax-1].ReadCh[0] = ChannelRead; 
					break;
				}
				case 1 : {
					chmap_val = Max11254_Read3byte( ChooseMax , MAX11254_REG_CHMAP0 );
					chmap_val&=0x00FF00FF;
					Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP0 , chmap_val|reg_val );
//					MAX[ChooseMax-1].ReadCh[1] = ChannelRead;
					break;
				}
				case 2 : { 
					chmap_val = Max11254_Read3byte( ChooseMax , MAX11254_REG_CHMAP0 );
					chmap_val&=0x0000FFFF;
					Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP0 , chmap_val|reg_val );
//					MAX[ChooseMax-1].ReadCh[2] = ChannelRead;
					break;
				}
				case 3 : { 
					chmap_val = Max11254_Read3byte( ChooseMax , MAX11254_REG_CHMAP1 );
					chmap_val&=0x00FFFF00;
					Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP1 , chmap_val|reg_val );
//					MAX[ChooseMax-1].ReadCh[3] = ChannelRead;
					break;
				}
				case 4 : {
					chmap_val = Max11254_Read3byte( ChooseMax , MAX11254_REG_CHMAP1 );
					chmap_val&=0x00FF00FF;
					Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP1 , chmap_val|reg_val );
//					MAX[ChooseMax-1].ReadCh[4] = ChannelRead;
					break;
				}
				case 5 : {
					chmap_val = Max11254_Read3byte( ChooseMax , MAX11254_REG_CHMAP1 );
					chmap_val&=0x0000FFFF;
					Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP1 , chmap_val|reg_val );
//					MAX[ChooseMax-1].ReadCh[5] = ChannelRead;
					break;
				}
				default: 
				break;
			}
		}
	} 
}
void 		Max11254_SequencerMode1_Entry 	( MaxDevice ChooseMax , SeqConvChannel  Ch  , GAIN Gain ) {
	/*	*****	*****	*****		CTRL1		*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1	, CTRL1_CALx_NOT_USE
														 |CTRL1_PDx_NOP
														 |CTRL1_UB_UNIPOLAR
														 |CTRL1_FORMAT_TWOCOMPLEMENT
														 |CTRL1_SCYCLE_DISABLE
														 |CTRL1_CONTSC_ENABLE );	// 	0x81
	/*	*****	*****	*****		CTRL2		*****	*****	*****	*/																													
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2	, CTRL2_EXTCLK_INTERNAL_CLOCK // CTRL2_EXTCLK_EXTERNAL_CLOCK
														 |CTRL2_CSSEN_CURRENTSOURCES_DIS
														 |CTRL2_LDOEN_INTERNAL_LDO_EN
														 |CTRL2_LPMODE_LOWPOWER_DIS
														 |CTRL2_PGA_DISABLE
														 |CTRL2_PGAGx_1 );
	/*	*****	*****	*****		CTRL3		*****	*****	*****	*/																										 
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL3	, CTRL3_GPO_MODE_DISABLE
														 |CTRL3_SYNC_MODE_ENABLE
														 |CTRL3_CALREGSEL_DISABLE
														 |CTRL3_NOSYSG_DISABLE
														 |CTRL3_NOSYSO_DISABLE
														 |CTRL3_NOSCG_ENABLE
														 |CTRL3_NOSCO_ENABLE );
	/*	*****	*****	*****	GPIO_CTRL	*****	*****	*****	*/
	if		( ChooseMax==MAX_1 )
		Max11254_Write1byte( ChooseMax , MAX11254_REG_GPIO_CTRL , GPIO_CTRL_GPIO1_EN_GPIO
															 	 |GPIO_CTRL_DIR1_OUTPUT
																 |GPIO_CTRL_DIO1_OUTPUT_LOW	//	GPIO_CTRL_DIO1_OUTPUT_HIGH

																 |GPIO_CTRL_GPIO0_EN_GPIO
																 |GPIO_CTRL_DIR0_OUTPUT
																 |GPIO_CTRL_DIO0_OUTPUT_LOW );
	else
		Max11254_Write1byte( ChooseMax , MAX11254_REG_GPIO_CTRL , GPIO_CTRL_GPIO1_EN_GPIO
															 	 |GPIO_CTRL_DIR1_OUTPUT
																 |GPIO_CTRL_DIO1_OUTPUT_LOW

																 |GPIO_CTRL_GPIO0_EN_GPIO
																 |GPIO_CTRL_DIR0_OUTPUT
																 |GPIO_CTRL_DIO0_OUTPUT_LOW );
	/*	*****	*****	*****		SEQ		*****	*****	*****	*/																											 
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ , Ch
														|SEQ_MODE_SEQUENCER_MODE_1
														|SEQ_GPODREN_GPIO_DELAY_DISABLE
														|SEQ_MDREN_MUX_DELAY_DISABLE
														|SEQ_RDYBEN_READY_BAR_ENABLE );
	/*	*****	*****	*****		GPO_DIR		*****	*****	*****	*/
	if		( ChooseMax==MAX_1 )
		Max11254_Write1byte( ChooseMax , MAX11254_REG_GPO_DIR , GPO_DIR_GPO0_OUTPUT_LOW
															   |GPO_DIR_GPO1_OUTPUT_LOW );
	else
		Max11254_Write1byte( ChooseMax , MAX11254_REG_GPO_DIR , GPO_DIR_GPO0_OUTPUT_HIGH
															   |GPO_DIR_GP01_OUTPUT_HIGH);


//	MAX[ChooseMax-1].AdcMode = 'S';
//	for( uint8_t i=0 ; i<6 ; i++ )
//		MAX[ChooseMax-1].ReadCh[i] = ChannelNotRead;
//	MAX[ChooseMax-1].ReadCh[((uint8_t)Ch)>>5] = ChannelRead;
	Max11254_PGAGain_Set( ChooseMax , Gain );
	if (  Gain!=0 )
		Max11254_SystemCalibration( ChooseMax , Ch );
}
void 		Max11254_SequencerMode2_Entry 	( MaxDevice ChooseMax , ChmapConvChannels Chs , GAIN Gain ) {
	/*	*****	*****	*****		CTRL1		*****	*****	*****	*/
//	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 	, CTRL1_CALx_NOT_USE
//															 |CTRL1_PDx_NOP	/*CTRL1_PDx_STANDBY*/
//															 |CTRL1_UB_BIPOLAR
//															 |CTRL1_FORMAT_TWOCOMPLEMENT
//															 |CTRL1_SCYCLE_ENABLE
//															 |CTRL1_CONTSC_ENABLE );
	Max11254_PolaritySelect( ChooseMax , MAX[((uint8_t)ChooseMax) - 1 ].polarity );
	/*	*****	*****	*****		CTRL2		*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2 	, CTRL2_EXTCLK_INTERNAL_CLOCK	//	CTRL2_EXTCLK_EXTERNAL_CLOCK
															 |CTRL2_CSSEN_CURRENTSOURCES_DIS
															 |CTRL2_LDOEN_INTERNAL_LDO_EN
															 |CTRL2_LPMODE_LOWPOWER_DIS
															 |CTRL2_PGA_DISABLE
															 |CTRL2_PGAGx_1 );
	/*	*****	*****	*****		CTRL3		*****	*****	*****	*/																										 
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL3		, CTRL3_GPO_MODE_DISABLE
															 |CTRL3_SYNC_MODE_ENABLE
															 |CTRL3_CALREGSEL_DISABLE
															 |CTRL3_NOSYSG_DISABLE
															 |CTRL3_NOSYSO_DISABLE
															 |CTRL3_NOSCG_ENABLE
															 |CTRL3_NOSCO_ENABLE );
	/*	*****	*****	*****	GPIO_CTRL	*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_GPIO_CTRL , GPIO_CTRL_GPIO1_EN_GPIO
															 |GPIO_CTRL_DIR1_OUTPUT
															 |GPIO_CTRL_DIO1_OUTPUT_LOW

															 |GPIO_CTRL_GPIO0_EN_GPIO
															 |GPIO_CTRL_DIR0_OUTPUT
															 |GPIO_CTRL_DIO0_OUTPUT_LOW );

	/*	*****	*****	*****		DELAY	*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_DELAY , 0x0000 );	
	/*	*****	*****			CHMAP1/CHMAP0		*****	*****	*/
	Max11254_ChannelMap_Set					( ChooseMax , Chs ); 
	/*	*****	*****	*****		SEQ		*****	*****	*****	*/
	// SEQ_RDYBEN_READY_BAR_DISABLE : Her Kanalin    Conversiyon islemi tamamlandiginda RDYB Low'a d�ser 
	// SEQ_RDYBEN_READY_BAR_ENABLE  : T�m Kanallarin Conversiyon islemi tamamlandiginda RDYB Low'a d�ser
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ	, SEQ_MUX_DISABLE
														 |SEQ_MODE_SEQUENCER_MODE_2
														 |SEQ_GPODREN_GPIO_DELAY_DISABLE
														 |SEQ_MDREN_MUX_DELAY_DISABLE
														 |SEQ_RDYBEN_READY_BAR_ENABLE );

	/*	*****	*****	*****	GPO_DIR	*****	*****	*****	*/
	Max11254_Write1byte ( ChooseMax , MAX11254_REG_GPO_DIR ,  GPO_DIR_GPO0_OUTPUT_LOW
														 	 |GPO_DIR_GPO1_OUTPUT_LOW );
//	MAX[ChooseMax-1].AdcMode = 'M';
	Max11254_PGAGain_Set( ChooseMax , Gain );
//	if ( ChooseMax == MAX_1 )
//		MAX[0].SendCommandADC = SendCommand_ENABLE;
//	if ( ChooseMax == MAX_2 ) 
//		MAX[1].SendCommandADC = SendCommand_ENABLE;
//	if ( ChooseMax == MAX_3 ) 
//		MAX[2].SendCommandADC = SendCommand_ENABLE;
//	if ( ChooseMax == MAX_4 ) 
//		MAX[3].SendCommandADC = SendCommand_ENABLE;
}
void 		Max11254_SequencerMode2_EntryUart( MaxDevice ChooseMax , ChmapConvChannels Chs , GAIN Gain ) {
	/*	*****	*****	*****		CTRL1		*****	*****	*****	*/
//	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 	, CTRL1_CALx_NOT_USE
//															 |CTRL1_PDx_NOP	/*CTRL1_PDx_STANDBY*/
//															 |CTRL1_UB_BIPOLAR
//															 |CTRL1_FORMAT_TWOCOMPLEMENT
//															 |CTRL1_SCYCLE_ENABLE
//															 |CTRL1_CONTSC_ENABLE );
	Max11254_PolaritySelect( ChooseMax , MAX[((uint8_t)ChooseMax) - 1 ].polarity );
	/*	*****	*****	*****		CTRL2		*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2 	, CTRL2_EXTCLK_INTERNAL_CLOCK	//	CTRL2_EXTCLK_EXTERNAL_CLOCK
															 |CTRL2_CSSEN_CURRENTSOURCES_DIS
															 |CTRL2_LDOEN_INTERNAL_LDO_EN
															 |CTRL2_LPMODE_LOWPOWER_DIS
															 |CTRL2_PGA_DISABLE
															 |CTRL2_PGAGx_1 );
	/*	*****	*****	*****		CTRL3		*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL3		, CTRL3_GPO_MODE_DISABLE
															 |CTRL3_SYNC_MODE_ENABLE
															 |CTRL3_CALREGSEL_DISABLE
															 |CTRL3_NOSYSG_DISABLE
															 |CTRL3_NOSYSO_DISABLE
															 |CTRL3_NOSCG_ENABLE
															 |CTRL3_NOSCO_ENABLE );
//	/*	*****	*****	*****	GPIO_CTRL	*****	*****	*****	*/
//	Max11254_Write1byte( ChooseMax , MAX11254_REG_GPIO_CTRL , GPIO_CTRL_GPIO1_EN_GPIO
//															 |GPIO_CTRL_DIR1_OUTPUT
//															 |GPIO_CTRL_DIO1_OUTPUT_LOW
//
//															 |GPIO_CTRL_GPIO0_EN_GPIO
//															 |GPIO_CTRL_DIR0_OUTPUT
//															 |GPIO_CTRL_DIO0_OUTPUT_LOW );

	/*	*****	*****	*****		DELAY	*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_DELAY , 0x0000 );
	/*	*****	*****			CHMAP1/CHMAP0		*****	*****	*/
	Max11254_ChannelMap_Set					( ChooseMax , Chs );
	/*	*****	*****	*****		SEQ		*****	*****	*****	*/
	// SEQ_RDYBEN_READY_BAR_DISABLE : Her Kanalin    Conversiyon islemi tamamlandiginda RDYB Low'a d�ser
	// SEQ_RDYBEN_READY_BAR_ENABLE  : T�m Kanallarin Conversiyon islemi tamamlandiginda RDYB Low'a d�ser
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ	, SEQ_MUX_DISABLE
														 |SEQ_MODE_SEQUENCER_MODE_2
														 |SEQ_GPODREN_GPIO_DELAY_DISABLE
														 |SEQ_MDREN_MUX_DELAY_DISABLE
														 |SEQ_RDYBEN_READY_BAR_ENABLE );

//	/*	*****	*****	*****	GPO_DIR	*****	*****	*****	*/
//	Max11254_Write1byte ( ChooseMax , MAX11254_REG_GPO_DIR ,  GPO_DIR_GPO0_OUTPUT_LOW
//														 	 |GPO_DIR_GPO1_OUTPUT_LOW );

	Max11254_PGAGain_Set( ChooseMax , Gain );
}
void 		Max11254_SequencerMode3_Entry 	( MaxDevice ChooseMax ) {
//	 MAX11254_REG_SEQ = SEQ_MODE2_SQUENCER3|SEQ_MDREN 					 = 0x12	--->	Her Kanalin    Conversiyon islemi tamamlandiginda RDYB Low'a d�ser	
//	 MAX11254_REG_SEQ = SEQ_MODE2_SQUENCER3|SEQ_MDREN|SEQ_RDYBEN = 0x13 --->  T�m Kanallarin Conversiyon islemi tamamlandiginda RDYB Low'a d�ser 
 		Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ    ,  SEQ_MODE_SEQUENCER_MODE_3|SEQ_RDYBEN_READY_BAR_ENABLE );														//	0x12
 		Max11254_Write2byte( ChooseMax , MAX11254_REG_DELAY  ,  0x0100 );	//	0xF000 -> 292.8Hz															//	0xF000
		Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL3  ,  CTRL3_GPO_MODE_ENABLE|CTRL3_CALREGSEL_ENABLE|CTRL3_NOSCG_ENABLE|CTRL3_NOSYSO_ENABLE);	//	0x5C
		Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP0 ,  0x0B274F );																									//	0x0B274F
		Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP1 ,  0x12161A );																									//	0x0B274F
		Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2  ,  CTRL2_EXTCLK_EXTERNAL_CLOCK|CTRL2_LDOEN_INTERNAL_LDO_EN|CTRL2_LPMODE_LOWPOWER_EN|CTRL2_PGA_ENABLE|CTRL2_PGAGx_128 );	//	0x3F
		Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_SEQUENCER|COMMAND_RATE_6 );                                   	//	0xBE
}
void 		Max11254_SequencerMode1_Exit 	( MaxDevice ChooseMax ) {
//	MAX[ChooseMax-1].AdcMode = '-';
	Max11254_Write1byte ( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_SCYCLE_ENABLE|CTRL1_PDx_STANDBY );		//	CTRL1_SCYCLE , CTRL1_CONTSC
	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_POWER_DOWN );
//	for ( uint8_t i=0 ; i<6 ; i++ )
//		MAX[ChooseMax-1].ReadCh[i] = ChannelNotRead;
}
void 		Max11254_SequencerMode2_Exit	( MaxDevice ChooseMax ) {
//	MAX[ChooseMax-1].AdcMode = '-';
	Max11254_Write1byte ( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_SCYCLE_ENABLE|CTRL1_PDx_SLEEP );		//	CTRL1_SCYCLE , CTRL1_CONTSC
	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_POWER_DOWN );
//	if ( ChooseMax == MAX_1 )
//		MAX[0].SendCommandADC = SendCommand_DISABLE;
//	if ( ChooseMax == MAX_2 ) 
//		MAX[1].SendCommandADC = SendCommand_DISABLE;
//	if ( ChooseMax == MAX_3 )
//		MAX[2].SendCommandADC = SendCommand_DISABLE;
//	if ( ChooseMax == MAX_4 ) 
//		MAX[3].SendCommandADC = SendCommand_DISABLE;
//	for ( uint8_t i=0 ; i<6 ; i++ )
//		MAX[ChooseMax-1].ReadCh[i] = ChannelNotRead;
}
void 		Max11254_Init					( void ) {
//	Max11254_SoftwareReset( );	
	Max11254_HardwareReset( );
	HAL_Delay( 200 );	

	channel_polarity[0] = 0;
	channel_polarity[1] = 0;
	channel_polarity[2] = 0;
	channel_polarity[3] = 0;

	for(uint8_t max = 0 ; max < 4 ; max++ ) {
		for(uint8_t ch = 0 ; ch < 6 ; ch++ ) {
			MAX[max].chResult[ch]	= 0x00000000;
			MAX[max].chRead[ch]		= CH_NotREAD;
			MAX[max].chGain[ch]		= (GAIN)GAIN_Disb;
		}
		MAX[max].polarity 	= POLARITY_BIPOLAR;
		MAX[max].rank		= 0;
	}

	resultBinding[ 0 ] = 4;
	resultBinding[ 1 ] = 10;
	resultBinding[ 2 ] = 16;
	resultBinding[ 3 ] = 22;

	/* MAX11254 - 1 Values Init */
	MAX[0].Gain = GAIN_x128;
	MAX[0].RateNumber = 2;
	MAX[0].chRead[4] = CH_READ;
	MAX[0].chGain[4] = (GAIN)GAIN_x128;
	MAX[0].chRead[5] = CH_NotREAD;
	MAX[0].chGain[5] = (GAIN)GAIN_x2;
	MAX[0].rank	= 4;
	Max11254_SelfCalibration( MAX_1 );
//	Max11254_SequencerMode1_Entry( MAX_1 , SEQ_CH2_ENABLE  , MAX[0].Gain  );
	Max11254_SequencerMode2_Entry( MAX_1 , (ChmapConvChannels)(CHMAP_CH4_ENABLE) , MAX[0].chGain[4] );

	/* MAX11254 - 2 Values Init */
	MAX[1].Gain = GAIN_x128;
	MAX[1].RateNumber = 0;
	MAX[1].chRead[4] = CH_READ;
	MAX[1].chGain[4] = (GAIN)GAIN_x128;
	MAX[1].chRead[5] = CH_NotREAD;
	MAX[1].chGain[5] = (GAIN)GAIN_x2;
	MAX[1].rank	= 4;
	Max11254_SelfCalibration( MAX_2 );
//	Max11254_SequencerMode1_Entry( MAX_2 , SEQ_CH1_ENABLE  , MAX[1].Gain  );
	Max11254_SequencerMode2_Entry( MAX_2 , (ChmapConvChannels)(CHMAP_CH4_ENABLE) , MAX[1].chGain[4] );
	
	/* MAX11254 - 3 Values Init */
	MAX[2].Gain = GAIN_x128;
	MAX[2].RateNumber = 0;
	MAX[2].chRead[4] = CH_READ;
	MAX[2].chGain[4] = (GAIN)GAIN_x128;
	MAX[2].chRead[5] = CH_NotREAD;
	MAX[2].chGain[5] = (GAIN)GAIN_x2;
	MAX[2].rank = 4;
	Max11254_SelfCalibration( MAX_3 );
//	Max11254_SequencerMode1_Entry( MAX_3 , SEQ_CH1_ENABLE  , MAX[2].Gain  );
	Max11254_SequencerMode2_Entry( MAX_3 , (ChmapConvChannels)(CHMAP_CH4_ENABLE) , MAX[2].chGain[4] );

	/* MAX11254 - 4 Values Init */
	MAX[3].Gain = GAIN_x128;
	MAX[3].RateNumber = 0;
	MAX[3].chRead[4] = CH_READ;
	MAX[3].chGain[4] = (GAIN)GAIN_x128;
	MAX[3].chRead[5] = CH_NotREAD;
	MAX[3].chGain[5] = (GAIN)GAIN_x2;
	MAX[3].rank = 4;
	Max11254_SelfCalibration( MAX_4 );
//	Max11254_SequencerMode1_Entry( MAX_4 , SEQ_CH1_ENABLE  , MAX[3].Gain  );
	Max11254_SequencerMode2_Entry( MAX_4 , (ChmapConvChannels)(CHMAP_CH4_ENABLE) , MAX[3].chGain[4] );

	Max11254_ConversionCommand( MAX_1 , MAX[0].RateNumber|COMMAND_MODE_SEQUENCER );
	Max11254_ConversionCommand( MAX_2 , MAX[1].RateNumber|COMMAND_MODE_SEQUENCER );
	Max11254_ConversionCommand( MAX_3 , MAX[2].RateNumber|COMMAND_MODE_SEQUENCER );
	Max11254_ConversionCommand( MAX_4 , MAX[3].RateNumber|COMMAND_MODE_SEQUENCER );
}
void 		OperatingMaxExtiRdbyControl 	( MaxDevice ChooseMax ) {
	uint8_t choose_max = (uint8_t)ChooseMax - 1 ;
	for ( uint8_t ch=0 ; ch<6 ; ch++ ) {
		if ( (MAX[choose_max].chRead[ch]==CH_READ) && (MAX[choose_max].rank==ch) ) {
			MAX[choose_max].chResult[ch] =  Max11254_Read3byte( ChooseMax , MAX11254_REG_DATA0+ch );
		}
	}
	Max11254_NextChannelOperations( ChooseMax );
	Max11254_ConversionCommand( ChooseMax , MAX[choose_max].RateNumber|COMMAND_MODE_SEQUENCER );
}

void		Max11254_GPIOSetting			( MaxDevice ChooseMax , uint8_t fourGpio ) {
	uint8_t regValue = 0x00;
	if ( (fourGpio & 0x08) == 0x08 )	regValue = GPIO_CTRL_DIO1_OUTPUT_HIGH;
	else 								regValue = GPIO_CTRL_DIO1_OUTPUT_LOW;

	if ( (fourGpio & 0x04) == 0x04 )	regValue |= GPIO_CTRL_DIO0_OUTPUT_HIGH;
	else 								regValue |= GPIO_CTRL_DIO0_OUTPUT_LOW;
	/*	*****	*****	*****	GPIO_CTRL	*****	*****	*****	*/
	Max11254_Write1byte( ChooseMax , MAX11254_REG_GPIO_CTRL , GPIO_CTRL_GPIO1_EN_GPIO
															 |GPIO_CTRL_DIR1_OUTPUT
															 |GPIO_CTRL_DIO1_OUTPUT_LOW

															 |GPIO_CTRL_GPIO0_EN_GPIO
															 |GPIO_CTRL_DIR0_OUTPUT
															 |GPIO_CTRL_DIO0_OUTPUT_LOW
															 |regValue);
	regValue = 0x00;
	if ( (fourGpio & 0x02) == 0x02 )	regValue = GPO_DIR_GP01_OUTPUT_HIGH;
	else								regValue = GPO_DIR_GPO1_OUTPUT_LOW;

	if ( (fourGpio & 0x01) == 0x01 )	regValue |=GPO_DIR_GPO0_OUTPUT_HIGH;
	else								regValue |=GPO_DIR_GPO0_OUTPUT_LOW;
	/*	*****	*****	*****	GPO_DIR	*****	*****	*****	*/
	Max11254_Write1byte ( ChooseMax , MAX11254_REG_GPO_DIR ,  GPO_DIR_GPO0_OUTPUT_HIGH
														 	 |GPO_DIR_GP01_OUTPUT_HIGH
															 |regValue );

}
void 		Max11254_PolaritySelect			( MaxDevice ChooseMax , uint8_t polarity ) {
	if 		( polarity == POLARITY_BIPOLAR ) {
		MAX[ ((uint8_t)ChooseMax) - 1 ].polarity = POLARITY_BIPOLAR;
		Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_CALx_NOT_USE
															 |CTRL1_PDx_NOP	/*CTRL1_PDx_STANDBY*/
															 |CTRL1_UB_BIPOLAR
															 |CTRL1_FORMAT_TWOCOMPLEMENT
															 |CTRL1_SCYCLE_ENABLE
															 |CTRL1_CONTSC_ENABLE );
	}
	else if ( polarity == POLARITY_UNIPOLAR ) {
		MAX[ ((uint8_t)ChooseMax) - 1 ].polarity = POLARITY_BIPOLAR;
		Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_CALx_NOT_USE
															 |CTRL1_PDx_NOP	/*CTRL1_PDx_STANDBY*/
															 |CTRL1_UB_UNIPOLAR
															 |CTRL1_FORMAT_TWOCOMPLEMENT
															 |CTRL1_SCYCLE_ENABLE
															 |CTRL1_CONTSC_ENABLE );
		}
	else
		return;
}
void 		Max11254_SPSSelect				( MaxDevice ChooseMax , uint8_t spsRate  ) {
	if ( spsRate <= 15 )
		MAX[ ((uint8_t)ChooseMax ) - 1 ].RateNumber = spsRate;
}
void		Max11254_NextChannelOperations	( MaxDevice ChooseMax ) {
	uint8_t choose_max = (uint8_t)ChooseMax - 1;
	uint8_t totalRankSize = 0;
	uint8_t control = 0;
	for( uint8_t i=0 ; i<6 ; i++ )
		totalRankSize++;

	if ( totalRankSize == 1 )
		return;

	for ( uint8_t i=MAX[choose_max].rank+1 ; i<6 ; i++ ) {
		if ( MAX[choose_max].chRead[i] == 1 ) {
			control = 1;
			MAX[choose_max].rank = i;
		}
	}
	if ( control == 0 ) {
		for ( uint8_t i=0 ; i<MAX[choose_max].rank ; i++ ) {
			if ( MAX[choose_max].chRead[i] == 1 ) {
				MAX[choose_max].rank = i;
			}
		}
	}
	Max11254_PGAGain_Set( ChooseMax , MAX[choose_max].chGain[MAX[choose_max].rank] );
}
