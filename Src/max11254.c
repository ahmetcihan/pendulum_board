/*	*****	*****	*****	*****	*****	*****	*****	*****	*****	*****	*/
/*																					*/
/*	Bu kutuphane 2019 yilinda TMS700(bb02502V22) PCB'si, 							*/
/*							  Press Cihazý icin										*/
/*							  Mehmet KARATAS Tarafindan yazilmistir.				*/
/*																					*/
/*	[ !! ] Press Cihaznýn TMS700 diger cihazlarýndan farklý							*/
/*		   ( - ) 4 kanal ADC vardir.												*/
/*		   ( - ) Birinci ve ikinci kanalda ki  ADClerden Transducer okunacaktir.	*/
/* 					Transducer okuyabilmek icin ADC Unipolar ve REFSEL_x pini High  */
/* 					olarak yapýlandýrmalý											*/
/*																					*/
/*		   ( - ) Üçüncü ve dördüncü kanalda ki  ADClerden LVDT okunacaktir.			*/
/* 					LVDT okuyabilmek icin ADC Unipolar ve REFSEL_x pini Low  		*/
/* 					olarak yapýlandýrmalý											*/
/*																					*/
/*	*****	*****	*****	*****	*****	*****	*****	*****	*****	*****	*/
#include "stm32f4xx_hal.h"
#include "max11254.h"
#include "spi.h"
 
ConnectedADC 	MAX[4];
uint32_t 		DeviceChannel[4];

uint8_t 		resultBinding[4];


void		Max11254_GPIOSetting			( MaxDevice ChooseMax , uint8_t fourGpio );
void 		Max11254_PolaritySelect			( MaxDevice ChooseMax , uint8_t polarity );
void 		Max11254_SPSSelect				( MaxDevice ChooseMax , uint8_t spsRate  );
void		Max11254_NextChannelOperations	( MaxDevice ChooseMax );

/**
  * @brief	Max11254'e komut girer.
  * @param  command   : max11254.h COMMAND_ ile baslayan Rate ve Mode seçimleri & islemi ile girilebi   
  */
void 		Max11254_ConversionCommand 		( MaxDevice ChooseMax , uint8_t command ) {
	switch ( ChooseMax ) {
		case MAX_1 : {
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			HAL_GPIO_WritePin( MAX11254_1_CS_PORT , MAX11254_1_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		case MAX_2 : {
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT , MAX11254_2_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			HAL_GPIO_WritePin( MAX11254_2_CS_PORT , MAX11254_2_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		case MAX_3 : {
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT , MAX11254_3_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			HAL_GPIO_WritePin( MAX11254_3_CS_PORT , MAX11254_3_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		case MAX_4 : {
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT , MAX11254_4_CS_PIN  , GPIO_PIN_RESET );
			HAL_SPI_Transmit ( &hspi2 , &command  ,   1   , TIMEOUT_HAL_SPI_TRANSMIT 	);
			HAL_GPIO_WritePin( MAX11254_4_CS_PORT , MAX11254_4_CS_PIN  , GPIO_PIN_SET 	);
			break;
		}
		default:
  	break;	
	}
} 
/**
  * @brief  MAX11254'ün 'Adr' registir adresine 8 bitlik 'Data' degerini yazar.
  * @param  Adr      	: Max11254 register adres 
  * @param  Data			: 8 bit register value 
  * @retval [ void ]
  */
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
/**
  * @brief  MAX11254'ün 'Adr' registir adresine 16 bitlik 'Data' degerini yazar
  * @param  Adr      	: Max11254 register adres  
  * @param  Data			: 16 bit register value
  * @retval [ void ]
  */
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
/**
  * @brief  MAX11254'ün 'Adr' registir adresine 24 bitlik 'Data' degerini yazar
  * @param  Adr      	: Max11254 register adres
  * @param  Data			: 24 bit register value
  * @retval [ void ]
  */
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
/**
  * @brief  MAX11254'ün 'Adr' registir adresinden 8 bitlik data okur 
  * @retval 8 bit register value 
  */
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
/**
  * @brief  MAX11254'ün 'Adr' registir adresinden 16 bitlik data okur   
  * @retval 16 bit register value 
  */
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
/**
  * @brief MAX11254'ün 'Adr' registir adresinden 24 bitlik data okur  
  * @retval 24 bit register value
  */
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
/**
  * @brief  MAX11254'ün RSTB pinini Low dan High'a çekerek donanimsal resetler 
  * @param  [ void ] 
  * @retval [ void ]
  */
void 		Max11254_HardwareReset 			( void ) {
		MAX11254s_RSTB_LOW;
		HAL_Delay( 200 );
		MAX11254s_RSTB_HIGH;
}
/**
  * @brief  MAX11254'ü CTRL1 registeri üzerinden yazilimsal resetler
  * @param  [ void ] 
  * @retval [ void ]
  */
void 		Max11254_SoftwareReset 			( MaxDevice ChooseMax ) {
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_CONTSC_ENABLE|CTRL1_PDx_RESET );
	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_POWER_DOWN );								
	HAL_Delay ( 100 );
//	uint8_t  ResetState = 1;
//	uint32_t RegData = 0;
//	while ( ResetState ) {
//		RegData = Max11254_Read3byte( MAX11254_REG_STAT );
//		if( RegData == 0x00000098 ) 	
//			ResetState = 0;
//	}
}
/**
  * @brief 	Max11254'ü Self Kalibre yapar.  
  * @param  [ void ] 
  * @retval [ void ]
  */
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
/**
  * @brief 	Max11254'ü System Offset Kalibresi yapar.  
  * @param  [ void ] 
  * @retval [ void ]
  */
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
/**
  * @brief 	Max11254'ü System Full Scacale Kalibresi yapar.  
  * @param  [ void ] 
  * @retval [ void ]
  */
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
/**
  * @brief 	Max11254'ü System Kalibresi yapar.  
  * @param  [ void ] 
  * @retval [ void ]
  */
void 		Max11254_SystemCalibration 		( MaxDevice ChooseMax , SeqConvChannel Ch ) {
	uint8_t reg_ctrl1 = Max11254_Read1byte ( ChooseMax , MAX11254_REG_CTRL1 );
	uint8_t	reg_seq		= Max11254_Read1byte ( ChooseMax , MAX11254_REG_SEQ		);
	MAX11254_SystemZeroCalibration( ChooseMax , Ch );	//	Max11254_SystemOffsetCalibration( ChooseMax );
	MAX11254_SystemFullCalibration( ChooseMax , Ch );	//	Max11254_SystemFullScaleCalibration( ChooseMax );
	Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL1 , reg_ctrl1 );
	Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ		, reg_seq );
}
/**
 * @brief  Max11254'ün anolog input sinyalini girilen deger kadar güclendirir.
 * @param  gain      : Programmable Gain Amplifier (PGA) kat sayisi.
 *          This parameter can be one of the following values:
 *			  @arg  GAIN_Disb	: PGA Devre Disi
 *            @arg  GAIN_x1  	: PGA Etkin. Kazanc = 1
 *            @arg  GAIN_x2  	: PGA Etkin. Kazanc = 2
 *            @arg  GAIN_x4  	: PGA Etkin. Kazanc = 4
 *            @arg  GAIN_x8	 	: PGA Etkin. Kazanc = 8
 *            @arg  GAIN_x16 	: PGA Etkin. Kazanc = 16
 *            @arg  GAIN_x32 	: PGA Etkin. Kazanc = 32
 *            @arg  GAIN_x64 	: PGA Etkin. Kazanc = 64
 *            @arg  GAIN_x128	: PGA Etkin. Kazanc = 128
 * @retval [ void ]
 */
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
/**
  * @brief  Max11254'ün dahili 6 kanalindan conversiyon islemine tabi tutulacak kanali seçer
  * @param  ActivatedCh : ADC'nin aktif edilcek kanali. 0 ile 5 arasi bi deger alir
	*   @arg   0 : Channel 0 Aktif 
	*   @arg   1 : Channel 1 Aktif   
	*   @arg   2 : Channel 2 Aktif   
	*   @arg   3 : Channel 3 Aktif   
	*   @arg   4 : Channel 4 Aktif   
	*   @arg   5 : Channel 5 Aktif   				
  * @param  OrderCh		  : Aktif edilen kanalin Conversion Sirasi. 1 ile 6 arasi bir deger alir
	*   @arg   1 : Birinci  Conversion Edilecek Kanal   
	*   @arg   2 : Ikinci   Conversion Edilecek Kanal 
	*   @arg   3 : Ucuncu   Conversion Edilecek Kanal   
	*   @arg   4 : Dorduncu Conversion Edilecek Kanal   
	*   @arg   5 : Besinci  Conversion Edilecek Kanal
	*   @arg   6 : Altinci  Conversion Edilecek Kanal   
	* @retval [ void ]
  */
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
/**
  * @brief  Max11254'ü Sequencer Mode 1 de yapilandirir.  
  * @param  [ void ] 
  * @retval [ void ]
  */
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
/**
  * @brief  Max11254'ü Sequencer Mode 2 de yapilandirir.  
  * @param  [ void ] 
  * @retval [ void ]
  */
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
	// SEQ_RDYBEN_READY_BAR_DISABLE : Her Kanalin    Conversiyon islemi tamamlandiginda RDYB Low'a düser 
	// SEQ_RDYBEN_READY_BAR_ENABLE  : Tüm Kanallarin Conversiyon islemi tamamlandiginda RDYB Low'a düser
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
	// SEQ_RDYBEN_READY_BAR_DISABLE : Her Kanalin    Conversiyon islemi tamamlandiginda RDYB Low'a düser
	// SEQ_RDYBEN_READY_BAR_ENABLE  : Tüm Kanallarin Conversiyon islemi tamamlandiginda RDYB Low'a düser
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

/**
  * @brief  Max11254'ü Sequencer Mode 3 de yapilandirir.
  * @param  [ void ] 
  * @retval [ void ]
  */
void 		Max11254_SequencerMode3_Entry 	( MaxDevice ChooseMax ) {
//	 MAX11254_REG_SEQ = SEQ_MODE2_SQUENCER3|SEQ_MDREN 					 = 0x12	--->	Her Kanalin    Conversiyon islemi tamamlandiginda RDYB Low'a düser	
//	 MAX11254_REG_SEQ = SEQ_MODE2_SQUENCER3|SEQ_MDREN|SEQ_RDYBEN = 0x13 --->  Tüm Kanallarin Conversiyon islemi tamamlandiginda RDYB Low'a düser 
 		Max11254_Write1byte( ChooseMax , MAX11254_REG_SEQ    ,  SEQ_MODE_SEQUENCER_MODE_3|SEQ_RDYBEN_READY_BAR_ENABLE );														//	0x12
 		Max11254_Write2byte( ChooseMax , MAX11254_REG_DELAY  ,  0x0100 );	//	0xF000 -> 292.8Hz															//	0xF000
		Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL3  ,  CTRL3_GPO_MODE_ENABLE|CTRL3_CALREGSEL_ENABLE|CTRL3_NOSCG_ENABLE|CTRL3_NOSYSO_ENABLE);	//	0x5C
		Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP0 ,  0x0B274F );																									//	0x0B274F
		Max11254_Write3byte( ChooseMax , MAX11254_REG_CHMAP1 ,  0x12161A );																									//	0x0B274F
		Max11254_Write1byte( ChooseMax , MAX11254_REG_CTRL2  ,  CTRL2_EXTCLK_EXTERNAL_CLOCK|CTRL2_LDOEN_INTERNAL_LDO_EN|CTRL2_LPMODE_LOWPOWER_EN|CTRL2_PGA_ENABLE|CTRL2_PGAGx_128 );	//	0x3F
		Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_SEQUENCER|COMMAND_RATE_6 );                                   	//	0xBE
}
/**
  * @brief  Max11254'ü Sequencer Mode 1 de yapilandirmasi iptal edilir.
  * @param  [ void ] 
  * @retval [ void ]
  */
void 		Max11254_SequencerMode1_Exit 	( MaxDevice ChooseMax ) {
//	MAX[ChooseMax-1].AdcMode = '-';
	Max11254_Write1byte ( ChooseMax , MAX11254_REG_CTRL1 , CTRL1_SCYCLE_ENABLE|CTRL1_PDx_STANDBY );		//	CTRL1_SCYCLE , CTRL1_CONTSC
	Max11254_ConversionCommand( ChooseMax , COMMAND_MODE_POWER_DOWN );
//	for ( uint8_t i=0 ; i<6 ; i++ )
//		MAX[ChooseMax-1].ReadCh[i] = ChannelNotRead;
}
/**
  * @brief  Max11254'ü Sequencer Mode 2 de yapilandirmasi iptal edilir.
  * @param  [ void ] 
  * @retval [ void ]
  */ 
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
/**
  * @brief  Max11254'ün baslangic yapilandirilmasi yapar.
  * @param  [ void ] 
  * @retval [ void ]
  */
void 		Max11254_Init					( void ) {
//	Max11254_SoftwareReset( );	
	Max11254_HardwareReset( );
	HAL_Delay( 200 );	

	for( uint8_t max=0 ; max<4 ; max++ ) {
		for( uint8_t ch=0 ; ch<6 ; ch++ ) {
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
	MAX[0].RateNumber = 0;
	MAX[0].chRead[4] = CH_READ;
	MAX[0].chGain[4] = (GAIN)GAIN_x128;
	MAX[0].chRead[5] = CH_READ;
	MAX[0].chGain[5] = (GAIN)GAIN_x2;
	MAX[0].rank	= 4;
	Max11254_SelfCalibration( MAX_1 );
//	Max11254_SequencerMode1_Entry( MAX_1 , SEQ_CH2_ENABLE  , MAX[0].Gain  );
	Max11254_SequencerMode2_Entry( MAX_1 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[0].chGain[4] );

	/* MAX11254 - 2 Values Init */
	MAX[1].Gain = GAIN_x128;
	MAX[1].RateNumber = 0;
	MAX[1].chRead[4] = CH_READ;
	MAX[1].chGain[4] = (GAIN)GAIN_x128;
	MAX[1].chRead[5] = CH_READ;
	MAX[1].chGain[5] = (GAIN)GAIN_x2;
	MAX[1].rank	= 4;
	Max11254_SelfCalibration( MAX_2 );
//	Max11254_SequencerMode1_Entry( MAX_2 , SEQ_CH1_ENABLE  , MAX[1].Gain  );
	Max11254_SequencerMode2_Entry( MAX_2 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[1].chGain[4] );
	
	/* MAX11254 - 3 Values Init */
	MAX[2].Gain = GAIN_x128;
	MAX[2].RateNumber = 0;
	MAX[2].chRead[4] = CH_READ;
	MAX[2].chGain[4] = (GAIN)GAIN_x128;
	MAX[2].chRead[5] = CH_READ;
	MAX[2].chGain[5] = (GAIN)GAIN_x2;
	MAX[2].rank = 4;
	Max11254_SelfCalibration( MAX_3 );
//	Max11254_SequencerMode1_Entry( MAX_3 , SEQ_CH1_ENABLE  , MAX[2].Gain  );
	Max11254_SequencerMode2_Entry( MAX_3 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[2].chGain[4] );

	/* MAX11254 - 4 Values Init */
	MAX[3].Gain = GAIN_x128;
	MAX[3].RateNumber = 0;
	MAX[3].chRead[4] = CH_READ;
	MAX[3].chGain[4] = (GAIN)GAIN_x128;
	MAX[3].chRead[5] = CH_READ;
	MAX[3].chGain[5] = (GAIN)GAIN_x2;
	MAX[3].rank = 4;
	Max11254_SelfCalibration( MAX_4 );
//	Max11254_SequencerMode1_Entry( MAX_4 , SEQ_CH1_ENABLE  , MAX[3].Gain  );
	Max11254_SequencerMode2_Entry( MAX_4 , (ChmapConvChannels)(CHMAP_CH4_ENABLE|CHMAP_CH5_ENABLE) , MAX[3].chGain[4] );

	Max11254_ConversionCommand( MAX_1 , MAX[0].RateNumber|COMMAND_MODE_SEQUENCER );
	Max11254_ConversionCommand( MAX_2 , MAX[1].RateNumber|COMMAND_MODE_SEQUENCER );
	Max11254_ConversionCommand( MAX_3 , MAX[2].RateNumber|COMMAND_MODE_SEQUENCER );
	Max11254_ConversionCommand( MAX_4 , MAX[3].RateNumber|COMMAND_MODE_SEQUENCER );
}

/**
  * @brief  Max11254 e tekrar conversion islemi icin komut atan ve yapilan conversiyon islemleri
	*				sonucunda raw datalari okuyan fonksiyon
  * @param  [ void ]  
  * @retval [ void ]
  */
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
/*	*****		*****			FILTERS FUNCTIONS			*****		*****	*/
/**	Klasik Moving Average Filtresi Fonksiyonlari 
  * @brief  Klasik Moving Average Filtresi Fonksiyonu    
  * @param  raw_signal	  			: MovingAverage Filtresine gönderilecek raw data 
	* @param  filter_coefficient	: Filtre katsayisi
	* @note   static double running_averange dizisi eleman sayisi filter_coefficient degerinden küçük olmamali 
  *          This parameter can be one of the following values:
  * @retval 
  */
/*
uint32_t 	Filter_ADC1_ClassicMovingAverage 		( uint32_t raw_signal ) {
    static uint32_t running_average[CMA_FILTER_COEF_MAX_VALUE] = { 0 };
    uint32_t processed_value;
    running_average[MAX[0].CMA_Coefficient-1] = raw_signal;
    processed_value = raw_signal;
    for ( uint8_t j=0 ; j<(MAX[0].CMA_Coefficient-1) ; j++ ){
        processed_value += running_average[j];
        running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/(MAX[0].CMA_Coefficient);
    return processed_value;
}
uint32_t 	Filter_ADC2_ClassicMovingAverage 		 ( uint32_t raw_signal ) {
    static uint32_t running_average[CMA_FILTER_COEF_MAX_VALUE] = { 0 };
    uint32_t processed_value;
    running_average[MAX[1].CMA_Coefficient-1] = raw_signal;
    processed_value = raw_signal;
    for ( uint8_t j=0 ; j<(MAX[1].CMA_Coefficient-1) ; j++ ){
        processed_value += running_average[j];
        running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/(MAX[1].CMA_Coefficient);
    return processed_value;
}
uint32_t 	Filter_ADC3_ClassicMovingAverage 		 ( uint32_t raw_signal ) {
    static uint32_t running_average[CMA_FILTER_COEF_MAX_VALUE] = { 0 };
    uint32_t processed_value;
    running_average[MAX[2].CMA_Coefficient-1] = raw_signal;
    processed_value = raw_signal;
    for ( uint8_t j=0 ; j<(MAX[0].CMA_Coefficient-1) ; j++ ){
        processed_value += running_average[j];
        running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/(MAX[0].CMA_Coefficient);
    return processed_value;
}
uint32_t 	Filter_ADC4_ClassicMovingAverage 		 ( uint32_t raw_signal ) {
    static uint32_t running_average[CMA_FILTER_COEF_MAX_VALUE] = { 0 };
    uint32_t processed_value;
    running_average[MAX[3].CMA_Coefficient-1] = raw_signal;
    processed_value = raw_signal;
    for ( uint8_t j=0 ; j<(MAX[1].CMA_Coefficient-1) ; j++ ){
        processed_value += running_average[j];
        running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/(MAX[1].CMA_Coefficient);
    return processed_value;
}
*/
/** Agirlastirilmis Moving Average Filtresi Fonksiyonlari
  * @brief  Agirlastirilmis Moving Average Filtresi Fonksiyonu  
  * @param  raw_signal	  			: MovingAverage Filtresine gönderilecek raw data 
	* @param  filter_coefficient	: Filtre katsayisi
  * @retval 
  */
/*
uint32_t 	Filter_ADC1_WeightedMovingAverage 		( uint32_t raw_signal ) {
	 static double weighted_average = 0;
   weighted_average -= weighted_average/MAX[0].WMA_Coefficient;
   weighted_average += raw_signal/MAX[0].WMA_Coefficient;
   return weighted_average;
}
uint32_t 	Filter_ADC2_WeightedMovingAverage 	 ( uint32_t raw_signal ) {
	 static double weighted_average = 0;
   weighted_average -= weighted_average/MAX[1].WMA_Coefficient;
   weighted_average += raw_signal/MAX[1].WMA_Coefficient;
   return weighted_average;
}
uint32_t 	Filter_ADC3_WeightedMovingAverage 	 ( uint32_t raw_signal ) {
	 static double weighted_average = 0;
   weighted_average -= weighted_average/MAX[2].WMA_Coefficient;
   weighted_average += raw_signal/MAX[2].WMA_Coefficient;
   return weighted_average;
}
uint32_t 	Filter_ADC4_WeightedMovingAverage 	 ( uint32_t raw_signal ) {
	 static double weighted_average = 0;
   weighted_average -= weighted_average/MAX[3].WMA_Coefficient;
   weighted_average += raw_signal/MAX[3].WMA_Coefficient;
   return weighted_average;
}
*/
/** Eksponential Moving Average Filtresi Fonksiyonlari
  * @brief  Eksponential Moving Average Filtresi Fonksiyonu  
  * @param  raw_signal	  			: MovingAverage Filtresine gönderilecek raw data 
	* @param  filter_coefficient	: Filtre katsayisi
  * @retval 
  */
/*
uint32_t 	Filter_ADC1_ExponentialMovingAverage 	( uint32_t raw_signal ) {
	static double EMA = 0;
  static double past_EMA = 0;
  double alpha = (double)2/(MAX[0].EMA_Coefficient+1); 
  EMA = (raw_signal)*alpha + past_EMA*(1-alpha);
  past_EMA = EMA;                                  
  return EMA;
}
uint32_t 	Filter_ADC2_ExponentialMovingAverage ( uint32_t raw_signal ) {
	static double EMA = 0;
  static double past_EMA = 0;
  double alpha = (double)2/(MAX[1].EMA_Coefficient+1); 
  EMA = (raw_signal)*alpha + past_EMA*(1-alpha);
  past_EMA = EMA;                                  
  return EMA;
}
uint32_t 	Filter_ADC3_ExponentialMovingAverage ( uint32_t raw_signal ) {
	static double EMA = 0;
  static double past_EMA = 0;
  double alpha = (double)2/(MAX[2].EMA_Coefficient+1); 
  EMA = (raw_signal)*alpha + past_EMA*(1-alpha);
  past_EMA = EMA;                                  
  return EMA;
}
uint32_t 	Filter_ADC4_ExponentialMovingAverage ( uint32_t raw_signal ) {
	static double EMA = 0;
  static double past_EMA = 0;
  double alpha = (double)2/(MAX[3].EMA_Coefficient+1); 
  EMA = (raw_signal)*alpha + past_EMA*(1-alpha);
  past_EMA = EMA;                                  
  return EMA;
}
*/
//	Filter_Bessel_Init icin */
//double F1ax[3] , F2ax[3] , F3ax[3] , F4ax[3];
//double F1by[3] , F2by[3] , F3by[3] , F4by[3] ;
/** Bessel Filtresi Init Fonksiyonulari
  * @brief  Bessel Filtresi Fonksiyonu  
  * @param  Samplerate	: Samplerate 
	* @param  cutoff			: 
  * @retval  [ void ]
  */
/*
void   		Filter_ADC1_Bessel_Init   				( uint16_t samplerate , uint8_t cutoff ) {
	double QcRaw  = (2.0 * M_PI * cutoff) / (1.0*samplerate);
	double QcWarp = 1.0* tan(QcRaw); // Warp cutoff frequency
  double gain		= 1.0 / (1.0 + M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp));
	F1by[2] = (1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
	F1by[1] = (2.0 - 4.0/(QcWarp*QcWarp)) * gain;
	F1by[0] = 1.0;
	F1ax[0] = 1.0 * gain;
	F1ax[1] = 2.0 * gain;
	F1ax[2] = 1.0 * gain;
}
void   		Filter_ADC2_Bessel_Init   					 ( uint16_t samplerate , uint8_t cutoff ) {
	double QcRaw  = (2.0 * M_PI * cutoff) / (1.0*samplerate);
	double QcWarp = 1.0* tan(QcRaw); // Warp cutoff frequency
  double gain		= 1.0 / (1.0 + M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp));
	F2by[2] = (1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
	F2by[1] = (2.0 - 4.0/(QcWarp*QcWarp)) * gain;
	F2by[0] = 1.0;
	F2ax[0] = 1.0 * gain;
	F2ax[1] = 2.0 * gain;
	F2ax[2] = 1.0 * gain;
}
void   		Filter_ADC3_Bessel_Init   					 ( uint16_t samplerate , uint8_t cutoff ) {
	double QcRaw  = (2.0 * M_PI * cutoff) / (1.0*samplerate);
	double QcWarp = 1.0* tan(QcRaw); // Warp cutoff frequency
  double gain		= 1.0 / (1.0 + M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp));
	F3by[2] = (1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
	F3by[1] = (2.0 - 4.0/(QcWarp*QcWarp)) * gain;
	F3by[0] = 1.0;
	F3ax[0] = 1.0 * gain;
	F3ax[1] = 2.0 * gain;
	F3ax[2] = 1.0 * gain;
}
void   		Filter_ADC4_Bessel_Init   					 ( uint16_t samplerate , uint8_t cutoff ) {
	double QcRaw  = (2.0 * M_PI * cutoff) / (1.0*samplerate);
	double QcWarp = 1.0* tan(QcRaw); // Warp cutoff frequency
  double gain		= 1.0 / (1.0 + M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp));
	F4by[2] = (1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
	F4by[1] = (2.0 - 4.0/(QcWarp*QcWarp)) * gain;
	F4by[0] = 1.0;
	F4ax[0] = 1.0 * gain;
	F4ax[1] = 2.0 * gain;
	F4ax[2] = 1.0 * gain;
}
*/
/** Bessel Filtresi Result Fonksiyon
  * @brief  Bessel Filtresi Fonksiyonu  
  * @param  input	: Samplerate 
  * @retval Bessel Filtre sonucu 
  */
/*
uint32_t 	Filter_ADC1_Bessel_Result				( uint32_t input ) {
	static double xv[3] = {0};
  static double yv[3] = {0}; 
	xv[2] = xv[1];
	xv[1] = xv[0];
	xv[0] = input;
	yv[2] = yv[1];
	yv[1] = yv[0];
	yv[0] =   (F1ax[0] * xv[0] + F1ax[1] * xv[1] + F1ax[2] * xv[2]
							- F1by[1] * yv[0]
							- F1by[2] * yv[1]);
	return yv[0];	
}
uint32_t 	Filter_ADC2_Bessel_Result					   ( uint32_t input ) {
	static double xv[3] = {0};
  static double yv[3] = {0}; 
	xv[2] = xv[1];
	xv[1] = xv[0];
	xv[0] = input;
	yv[2] = yv[1];
	yv[1] = yv[0];
	yv[0] =   (F2ax[0] * xv[0] + F2ax[1] * xv[1] + F2ax[2] * xv[2]
							- F2by[1] * yv[0]
							- F2by[2] * yv[1]);
	return yv[0];	
} 
uint32_t 	Filter_ADC3_Bessel_Result					   ( uint32_t input ) {
	static double xv[3] = {0};
  static double yv[3] = {0}; 
	xv[2] = xv[1];
	xv[1] = xv[0];
	xv[0] = input;
	yv[2] = yv[1];
	yv[1] = yv[0];
	yv[0] =   (F3ax[0] * xv[0] + F3ax[1] * xv[1] + F3ax[2] * xv[2]
							- F3by[1] * yv[0]
							- F3by[2] * yv[1]);
	return yv[0];	
}
uint32_t 	Filter_ADC4_Bessel_Result					   ( uint32_t input ) {
	static double xv[3] = {0};
  static double yv[3] = {0}; 
	xv[2] = xv[1];
	xv[1] = xv[0];
	xv[0] = input;
	yv[2] = yv[1];
	yv[1] = yv[0];
	yv[0] =   (F4ax[0] * xv[0] + F4ax[1] * xv[1] + F4ax[2] * xv[2]
							- F4by[1] * yv[0]
							- F4by[2] * yv[1]);
	return yv[0];	
}
*/
