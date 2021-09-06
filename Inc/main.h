#ifndef __MAIN_H
#define __MAIN_H

#define Tx2En 					GPIO_PIN_12

#define Led_Pin 				GPIO_PIN_11
#define Led_GPIO_Port 			GPIOA

#define RELAY_DRV_3_Pin       	GPIO_PIN_3
#define RELAY_DRV_3_GPIO_Port	GPIOC
#define RELAY_DRV_4_Pin       	GPIO_PIN_4
#define RELAY_DRV_4_GPIO_Port	GPIOC

#define ADC_RST_Pin 			GPIO_PIN_7
#define ADC_RST_GPIO_Port 		GPIOC
#define ADC_1_CS_Pin 			GPIO_PIN_12
#define ADC_1_CS_GPIO_Port 		GPIOB
#define ADC_1_RDYB_Pin 			GPIO_PIN_6
#define ADC_1_RDYB_GPIO_Port 	GPIOC
#define ADC_1_RDYB_EXTI_IRQn 	EXTI9_5_IRQn
#define ADC_2_CS_Pin 			GPIO_PIN_10
#define ADC_2_CS_GPIO_Port 		GPIOB  
#define ADC_2_RDYB_Pin 			GPIO_PIN_2
#define ADC_2_RDYB_GPIO_Port 	GPIOB
#define ADC_2_RDYB_EXTI_IRQn 	EXTI2_IRQn  
#define ADC_3_CS_Pin 			GPIO_PIN_4
#define ADC_3_CS_GPIO_Port 		GPIOA
#define ADC_3_RDYB_Pin 			GPIO_PIN_1
#define ADC_3_RDYB_GPIO_Port 	GPIOB
#define ADC_3_RDYB_EXTI_IRQn 	EXTI1_IRQn  
#define ADC_4_CS_Pin 			GPIO_PIN_5
#define ADC_4_CS_GPIO_Port 		GPIOC
#define ADC_4_RDYB_Pin 			GPIO_PIN_0
#define ADC_4_RDYB_GPIO_Port 	GPIOB
#define ADC_4_RDYB_EXTI_IRQn 	EXTI0_IRQn 

#define Dac_Sync_Pin 			GPIO_PIN_10
#define Dac_Sync_GPIO_Port    	GPIOA
#define DacPWM_Pin 				GPIO_PIN_9
#define DacPWM_GPIO_Port		GPIOC

#define PulseOut_Pin 			GPIO_PIN_4   	//	Press Cihazinda Islevi Yok
#define PulseOut_GPIO_Port		GPIOB        	//	Press Cihazinda Islevi Yok
#define PulseOutEn_Pin 			GPIO_PIN_5    	//	Press Cihazinda Islevi Yok
#define PulseOutEn_GPIO_Port  	GPIOB         	//	Press Cihazinda Islevi Yok
#define MotorDir_Pin 			GPIO_PIN_8    	//	Press Cihazinda Islevi Yok
#define MotorDir_GPIO_Port    	GPIOC         	//	Press Cihazinda Islevi Yok

#define RELAY_DRV_1_Pin       	GPIO_PIN_1		//	Press Cihazinda Islevi Yok
#define RELAY_DRV_1_GPIO_Port	GPIOC         	//	Press Cihazinda Islevi Yok
#define RELAY_DRV_2_Pin       	GPIO_PIN_2    	//	Press Cihazinda Islevi Yok
#define RELAY_DRV_2_GPIO_Port	GPIOC        	//	Press Cihazinda Islevi Yok

#define ExtInp_1_Pin 			GPIO_PIN_0    	//	Press Cihazinda Islevi Yok
#define ExtInp_1_GPIO_Port 		GPIOC         	//	Press Cihazinda Islevi Yok
#define ExtInp_2_Pin 			GPIO_PIN_15  	//	Press Cihazinda Islevi Yok
#define ExtInp_2_GPIO_Port 		GPIOC        	//	Press Cihazinda Islevi Yok
#define ExtInp_3_Pin 			GPIO_PIN_14   	//	Press Cihazinda Islevi Yok
#define ExtInp_3_GPIO_Port 		GPIOC         	//	Press Cihazinda Islevi Yok
#define ExtInp_4_Pin 			GPIO_PIN_13   	//	Press Cihazinda Islevi Yok
#define ExtInp_4_GPIO_Port 		GPIOC         	//	Press Cihazinda Islevi Yok

#define EncoderAIn_Pin        	GPIO_PIN_8    	//	Press Cihazinda Islevi Yok
#define EncoderAIn_GPIO_Port  	GPIOA         	//	Press Cihazinda Islevi Yok
#define EncoderBIn_Pin        	GPIO_PIN_9    	//	Press Cihazinda Islevi Yok
#define EncoderBIn_GPIO_Port  	GPIOA         	//	Press Cihazinda Islevi Yok
#define EncoderZF_Pin         	GPIO_PIN_3    	//	Press Cihazinda Islevi Yok
#define EncoderZF_GPIO_Port		GPIOB         	//	Press Cihazinda Islevi Yok
#define EncoderZF_EXTI_IRQn		EXTI3_IRQn    	//	Press Cihazinda Islevi Yok

typedef enum {
  ControlState_CHECKED = 0,		//	 kontrol edildi
  ControlState_CHECKIT			//	 kontrol et
}ControlState; 
extern ControlState ControlUsart1_TransmitData;

void _Error_Handler(char *, int);

unsigned char step_motor_command;
unsigned char step_motor_speed[3];
unsigned int step_motor_requested_pos;
unsigned char timer_1_msec;
unsigned char timer_100_msec;

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif /* __MAIN_H */
