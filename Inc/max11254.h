#ifndef __MAX11254_H
#define __MAX11254_H 

#if !defined  (   MAX11254_1_CS_PIN 	) 
 #define MAX11254_1_CS_PIN  ADC_1_CS_Pin   							
#endif /* MAX11254_1_CS_PIN */
#if !defined  (   MAX11254_1_CS_PORT 	) 
 #define MAX11254_1_CS_PORT  ADC_1_CS_GPIO_Port
#endif /* MAX11254_1_CS_PORT */

#if !defined  (   MAX11254_2_CS_PIN 	) 
 #define MAX11254_2_CS_PIN  ADC_2_CS_Pin   							
#endif /* MAX11254_2_CS_PIN */
#if !defined  (   MAX11254_2_CS_PORT 	) 
 #define MAX11254_2_CS_PORT  ADC_2_CS_GPIO_Port
#endif /* MAX11254_2_CS_PORT */

#if !defined  (   MAX11254_3_CS_PIN 	) 
 #define MAX11254_3_CS_PIN  ADC_3_CS_Pin   							
#endif /* MAX11254_3_CS_PIN */
#if !defined  (   MAX11254_3_CS_PORT 	) 
 #define MAX11254_3_CS_PORT  ADC_3_CS_GPIO_Port
#endif /* MAX11254_3_CS_PORT */

#if !defined  (   MAX11254_4_CS_PIN 	) 
 #define MAX11254_4_CS_PIN  ADC_4_CS_Pin   							
#endif /* MAX11254_4_CS_PIN */
#if !defined  (   MAX11254_4_CS_PORT 	) 
 #define MAX11254_4_CS_PORT  ADC_4_CS_GPIO_Port
#endif /* MAX11254_4_CS_PORT */

#define MAX11254s_RSTB_LOW	    HAL_GPIO_WritePin( ADC_RST_GPIO_Port , ADC_RST_Pin , GPIO_PIN_RESET );
#define MAX11254s_RSTB_HIGH 	HAL_GPIO_WritePin( ADC_RST_GPIO_Port , ADC_RST_Pin , GPIO_PIN_SET   );

#define CH0	0x00
#define CH1 0x01
#define CH2 0x02
#define CH3 0x03
#define CH4 0x04
#define CH5 0x05 

#define	CH_NotREAD	0x00
#define	CH_READ 	0x01

#define POLARITY_BIPOLAR	0x00	//	( -Vref to +Vref )
#define POLARITY_UNIPOLAR	0x01	//	(   0V  to +Vref )

#define MAX11254_REG_STAT					0			//		W:0xC0			R:0xC1
#define MAX11254_REG_CTRL1					1   		//		W:0xC2			R:0xC3
#define MAX11254_REG_CTRL2					2   		//		W:0xC4			R:0xC5
#define MAX11254_REG_CTRL3					3   		//		W:0xC6			R:0xC7
#define MAX11254_REG_GPIO_CTRL				4   		//		W:0xC8			R:0xC9	
#define MAX11254_REG_DELAY      			5   		//		W:0xCA			R:0xCB	
#define MAX11254_REG_CHMAP1     			6   		//		W:0xCC			R:0xCD	
#define MAX11254_REG_CHMAP0     			7	   		//		W:0xCE 			R:0xCF	
#define MAX11254_REG_SEQ        			8   		//		W:0xD0			R:0xD1	
#define MAX11254_REG_GPO_DIR    			9   		//		W:0xD2			R:0xD3	
#define MAX11254_REG_SOC        			10  		//		W:0xD4			R:0xD5	
#define MAX11254_REG_SGC        			11  		//		W:0xD6			R:0xD7	
#define MAX11254_REG_SCOC       			12  		//		W:0xD8			R:0xD9	
#define MAX11254_REG_SCGC       			13  		//		W:0xDA			R:0xDB	
#define MAX11254_REG_DATA0      			14  		//		W:0xDC			R:0xDD	
#define MAX11254_REG_DATA1      			15  		//		W:0xDE			R:0xDF	
#define MAX11254_REG_DATA2      			16  		//		W:0xE0			R:0xE1	
#define MAX11254_REG_DATA3      			17  		//		W:0xE2			R:0xE3	
#define MAX11254_REG_DATA4      			18  		//		W:0xE4			R:0xE5	
#define MAX11254_REG_DATA5      			19  		//		W:0xE6			R:0xE7	

#define COMMAND_MODE_UNUSED					0x80
#define COMMAND_MODE_POWER_DOWN   			0x90
#define COMMAND_MODE_CALIBRATION  			0xA0
#define COMMAND_MODE_SEQUENCER    			0xB0
#define COMMAND_RATE_0						0x80
#define COMMAND_RATE_1			  			0x81
#define COMMAND_RATE_2			  			0x82
#define COMMAND_RATE_3			  			0x83
#define COMMAND_RATE_4						0x84
#define COMMAND_RATE_5			  			0x85
#define COMMAND_RATE_6			  			0x86
#define COMMAND_RATE_7			  			0x87
#define COMMAND_RATE_8			  			0x88
#define COMMAND_RATE_9			  			0x89
#define COMMAND_RATE_10			  			0x8A
#define COMMAND_RATE_11    					0x8B
#define COMMAND_RATE_12    					0x8C
#define COMMAND_RATE_13    					0x8D
#define COMMAND_RATE_14   					0x8E
#define COMMAND_RATE_15						0x8F

#define STAT_PDSTAT_CONVERSION	((uint32_t)0x00000000U)					 					
#define STAT_PDSTAT_SLEEP		((uint32_t)0x00000004U)
#define STAT_PDSTAT_STANDBY     ((uint32_t)0x00000008U)
#define STAT_PDSTAT_RESET       ((uint32_t)0x0000000CU)

#define CTRL1_CALx_SELF_CAL           		0x00
#define CTRL1_CALx_SYSTEMOFFSET_CAL   		0x40
#define CTRL1_CALx_SYSTEMFULL_CAL			0x80
#define CTRL1_CALx_NOT_USE					0xC0
#define CTRL1_PDx_NOP                 		0x00
#define CTRL1_PDx_SLEEP               		0x10
#define CTRL1_PDx_STANDBY             		0x20
#define CTRL1_PDx_RESET               		0x30
#define CTRL1_UB_BIPOLAR              		0x00
#define CTRL1_UB_UNIPOLAR					0x08
#define CTRL1_FORMAT_TWOCOMPLEMENT     		0x00
#define CTRL1_FORMAT_BINARYFORMAT   		0x04 
#define CTRL1_SCYCLE_DISABLE          		0x00
#define CTRL1_SCYCLE_ENABLE           		0x02
#define CTRL1_CONTSC_DISABLE          		0x00
#define CTRL1_CONTSC_ENABLE           		0x01

#define CTRL2_PGA_DISABLE    				0x00
#define CTRL2_PGA_ENABLE    	 			0x08
#define CTRL2_PGAGx_1     					0x00
#define CTRL2_PGAGx_2     					0x01
#define CTRL2_PGAGx_8    					0x03
#define CTRL2_PGAGx_16   	 				0x04
#define CTRL2_PGAGx_32     					0x05
#define CTRL2_PGAGx_64   					0x06
#define CTRL2_PGAGx_128     				0x07
#define CTRL2_LPMODE_LOWPOWER_DIS    		0x00 
#define CTRL2_LPMODE_LOWPOWER_EN    		0x10
#define CTRL2_LDOEN_INTERNAL_LDO_DIS 	 	0x00
#define CTRL2_LDOEN_INTERNAL_LDO_EN   		0x20
#define CTRL2_CSSEN_CURRENTSOURCES_DIS		0x00
#define CTRL2_CSSEN_CURRENTSOURCES_EN 		0x40
#define CTRL2_EXTCLK_INTERNAL_CLOCK  		0x00
#define CTRL2_EXTCLK_EXTERNAL_CLOCK  		0x80

#define CTRL3_GPO_MODE_ENABLE				0x80
#define CTRL3_GPO_MODE_DISABLE          	0x00
#define CTRL3_SYNC_MODE_ENABLE          	0x40
#define CTRL3_SYNC_MODE_DISABLE				0x00
#define CTRL3_CALREGSEL_ENABLE          	0x20
#define CTRL3_CALREGSEL_DISABLE				0x00
#define CTRL3_NOSYSG_ENABLE             	0x00
#define CTRL3_NOSYSG_DISABLE				0x08
#define CTRL3_NOSYSO_ENABLE					0x00
#define CTRL3_NOSYSO_DISABLE            	0x04
#define CTRL3_NOSCG_ENABLE              	0x00
#define CTRL3_NOSCG_DISABLE             	0x02
#define CTRL3_NOSCO_ENABLE              	0x00
#define CTRL3_NOSCO_DISABLE             	0x01

#define SEQ_MUX_DISABLE						0x00
#define SEQ_MUX_SEQUENCER1_CH0				0x00
#define SEQ_MUX_SEQUENCER1_CH1        		0x20
#define SEQ_MUX_SEQUENCER1_CH2       	 	0x40
#define SEQ_MUX_SEQUENCER1_CH3        		0x60
#define SEQ_MUX_SEQUENCER1_CH4        		0x80
#define SEQ_MUX_SEQUENCER1_CH5        		0xA0
#define SEQ_MODE_SEQUENCER_MODE_1     		0x00
#define SEQ_MODE_SEQUENCER_MODE_2     		0x08
#define SEQ_MODE_SEQUENCER_MODE_3     		0x10
#define SEQ_MODE_DO_NOT_USE           		0x18
#define SEQ_GPODREN_GPIO_DELAY_ENABLE		0x00
#define SEQ_GPODREN_GPIO_DELAY_DISABLE		0x04
#define SEQ_MDREN_MUX_DELAY_DISABLE     	0x00
#define SEQ_MDREN_MUX_DELAY_ENABLE      	0x02
#define SEQ_RDYBEN_READY_BAR_DISABLE    	0x00
#define SEQ_RDYBEN_READY_BAR_ENABLE     	0x01

#define GPIO_CTRL_GPIO1_EN_GPIO				0x80
#define GPIO_CTRL_GPIO1_EN_SYNCINPUT		0x00
#define GPIO_CTRL_GPIO0_EN_GPIO       		0x40
#define GPIO_CTRL_GPIO0_EN_EXCLOCK_IN 		0x00
#define GPIO_CTRL_DIR1_OUTPUT         		0x10
#define GPIO_CTRL_DIR1_INPUT          		0x00
#define GPIO_CTRL_DIR0_OUTPUT         		0x08
#define GPIO_CTRL_DIR0_INPUT          		0x00
#define GPIO_CTRL_DIO1_OUTPUT_HIGH			0x02
#define GPIO_CTRL_DIO1_OUTPUT_LOW      		0x00
#define GPIO_CTRL_DIO0_OUTPUT_HIGH      	0x01
#define GPIO_CTRL_DIO0_OUTPUT_LOW       	0x00

#define GPO_DIR_GPO0_OUTPUT_LOW				0x01
#define GPO_DIR_GPO0_OUTPUT_HIGH      		0x00
#define GPO_DIR_GPO1_OUTPUT_LOW				0x02
#define GPO_DIR_GP01_OUTPUT_HIGH      		0x00

typedef enum 	 { /*		  SeqConvChannel		*/
	SEQ_CH0_ENABLE = 0x00,   /*	MAX11254 SingleChannel mod yanlizca CH0 conversion	*/
	SEQ_CH1_ENABLE = 0x20,   /*	MAX11254 SingleChannel mod yanlizca CH1 conversion	*/
	SEQ_CH2_ENABLE = 0x40,   /*	MAX11254 SingleChannel mod yanlizca CH2 conversion 	*/
	SEQ_CH3_ENABLE = 0x60,   /*	MAX11254 SingleChannel mod yanlizca CH3 conversion	*/
	SEQ_CH4_ENABLE = 0x80,   /*	MAX11254 SingleChannel mod yanlizca CH4 conversion	*/
	SEQ_CH5_ENABLE = 0xA0,   /*	MAX11254 SingleChannel mod yanlizca CH5 conversion	*/
}SeqConvChannel;
typedef enum 	 { /*		ChmapConvChannels		*/
	CHMAP_CH0_ENABLE = 0x01,   /*	MAX11254 MultiChannel modda CH0'ida conversion'a dahil et	*/
	CHMAP_CH1_ENABLE = 0x02,   /*	MAX11254 MultiChannel modda CH1'ida conversion'a dahil et	*/
	CHMAP_CH2_ENABLE = 0x04,   /*	MAX11254 MultiChannel modda CH2'ida conversion'a dahil et	*/
	CHMAP_CH3_ENABLE = 0x08,   /*	MAX11254 MultiChannel modda CH3'ida conversion'a dahil et	*/
	CHMAP_CH4_ENABLE = 0x10,   /*	MAX11254 MultiChannel modda CH4'ida conversion'a dahil et	*/
	CHMAP_CH5_ENABLE = 0x20,   /*	MAX11254 MultiChannel modda CH5'ida conversion'a dahil et	*/
}ChmapConvChannels; 
typedef enum	 { /*				MaxDevice		*/
	MAX_1 = 0x01,  /*	First  Connected MAX11254	*/
	MAX_2 = 0x02,  /*	Second Connected MAX11254	*/
	MAX_3 = 0x03,  /*	Third	 Connected MAX11254	*/
	MAX_4 = 0x04   /*	Fourth Connected MAX11254	*/	
}MaxDevice;
typedef enum	 { /*			 SendCommand		*/
	SendCommand_DISABLE  = 0x00,	/*	MAX11254 MultiChannel modda conversion sonrasi komut ATMA!  */
	SendCommand_ENABLE	 = 0x01   	/*	MAX11254 MultiChannel modda conversion sonrasi komut AT !   */
}SendCommand;
typedef enum	 { /* 		 ReadChannel			*/
	ChannelNotRead 	= 0x00,    	/*	Kanali Okuma  	*/
	ChannelRead 	= 0x01   	/*	Kanali Oku   	*/
}ReadChannel;
typedef enum	 { /* 		 FilterState			*/
	FilterStatDisable 	= 0x00,    		/*	Yazilimsal Filtreyi Calistirma */
	FilterStateEnable 	= 0x01   		/*	Yazilimsal Filtreyi Calistir   */
}FilterState;
typedef enum	 { /*  Programmable Gain Amplifier	*/
	GAIN_Disb 	= 0x00,    	/*	MAX11254 --> CTRL2 --> 0x_0 --> PGA DISABLE */
	GAIN_x1 	= 0x01,   	/*	MAX11254 --> CTRL2 --> 0x_8 --> PGA = 1	 	*/
	GAIN_x2 	= 0x02,   	/*	MAX11254 --> CTRL2 --> 0x_9 --> PGA = 2	 	*/
	GAIN_x4 	= 0x03,   	/*	MAX11254 --> CTRL2 --> 0x_A --> PGA = 4	 	*/
	GAIN_x8 	= 0x04,   	/*	MAX11254 --> CTRL2 --> 0x_B --> PGA = 8	 	*/
	GAIN_x16 	= 0x05,   	/*	MAX11254 --> CTRL2 --> 0x_C --> PGA = 16	*/
	GAIN_x32 	= 0x06,   	/*	MAX11254 --> CTRL2 --> 0x_D --> PGA = 32	*/
	GAIN_x64 	= 0x07,   	/*	MAX11254 --> CTRL2 --> 0x_E --> PGA = 64	*/
	GAIN_x128 	= 0x08,   	/*	MAX11254 --> CTRL2 --> 0x_F --> PGA = 128	*/
}GAIN;
typedef struct { /*		  ConnectedADC      */
  GAIN			Gain;				/* MAX'in Gain Degeri				 			*/
  uint8_t  	  	RateNumber;       	/* MAX'in calisma SPS degeri 					*/
  uint32_t		chResult[6];
  uint8_t 		chRead[6];
  GAIN			chGain[6];
  uint8_t 		polarity;
  uint8_t		rank;
}ConnectedADC;

extern ConnectedADC	MAX[4];
									
void 		Max11254_ConversionCommand 		( MaxDevice ChooseMax , uint8_t command );
void 		Max11254_Write1byte 			( MaxDevice ChooseMax , uint8_t Adr , uint8_t  Data );
void		Max11254_Write2byte 			( MaxDevice ChooseMax , uint8_t Adr , uint16_t Data );
void 		Max11254_Write3byte 			( MaxDevice ChooseMax , uint8_t Adr , uint32_t Data );
uint8_t 	Max11254_Read1byte 				( MaxDevice ChooseMax , uint8_t Adr );
uint16_t 	Max11254_Read2byte 			   	( MaxDevice ChooseMax , uint8_t Adr );
uint32_t 	Max11254_Read3byte				( MaxDevice ChooseMax , uint8_t Adr );
void 		Max11254_HardwareReset 			( void );
void 		Max11254_SoftwareReset 			( MaxDevice ChooseMax );
void 		Max11254_SelfCalibration 		( MaxDevice ChooseMax );
void 		Max11254_SystemZeroCalibration  ( MaxDevice ChooseMax , SeqConvChannel Ch );
void 		Max11254_SystemFullCalibration	( MaxDevice ChooseMax , SeqConvChannel Ch );
void 		Max11254_SystemCalibration 		( MaxDevice ChooseMax , SeqConvChannel Ch );
void 		Max11254_PGAGain_Set			( MaxDevice ChooseMax , GAIN Gain );
void 		Max11254_ChannelMap_Set			( MaxDevice ChooseMax , ChmapConvChannels Chs );
void 		Max11254_SequencerMode1_Entry 	( MaxDevice ChooseMax , SeqConvChannel 	  Ch  , GAIN Gain );
void 		Max11254_SequencerMode2_Entry 	( MaxDevice ChooseMax , ChmapConvChannels Chs , GAIN Gain );
void 		Max11254_SequencerMode1_Exit 	( MaxDevice ChooseMax );
void 		Max11254_SequencerMode2_Exit	( MaxDevice ChooseMax );
void 		Max11254_Init					( void );
void 		OperatingMaxExtiRdbyControl 	( MaxDevice ChooseMax );
void		Max11254_GPIOSetting			( MaxDevice ChooseMax , uint8_t fourGpio );
void 		Max11254_PolaritySelect			( MaxDevice ChooseMax , uint8_t polarity );
void 		Max11254_SequencerMode2_EntryUart( MaxDevice ChooseMax , ChmapConvChannels Chs , GAIN Gain );

uint8_t channel_polarity[4];
uint32_t DeviceChannel[4];
uint8_t	resultBinding[4];

#endif /*__MAX11254_H */
