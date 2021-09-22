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

#define PulseOut_Pin 			GPIO_PIN_4
#define PulseOut_GPIO_Port		GPIOB
#define PulseOutEn_Pin 			GPIO_PIN_5
#define PulseOutEn_GPIO_Port  	GPIOB
#define MotorDir_Pin 			GPIO_PIN_8
#define MotorDir_GPIO_Port    	GPIOC

#define RELAY_DRV_1_Pin       	GPIO_PIN_1
#define RELAY_DRV_1_GPIO_Port	GPIOC
#define RELAY_DRV_2_Pin       	GPIO_PIN_2
#define RELAY_DRV_2_GPIO_Port	GPIOC

#define INPUT_1_Pin 			GPIO_PIN_0
#define INPUT_2_Pin 			GPIO_PIN_15
#define INPUT_3_Pin 			GPIO_PIN_14
#define INPUT_4_Pin 			GPIO_PIN_13
#define INPUT_Port		 		GPIOC

#define ExtInp_1_Pin 			GPIO_PIN_0
#define ExtInp_1_GPIO_Port 		GPIOC
#define ExtInp_2_Pin 			GPIO_PIN_15
#define ExtInp_2_GPIO_Port 		GPIOC
#define ExtInp_3_Pin 			GPIO_PIN_14
#define ExtInp_3_GPIO_Port 		GPIOC
#define ExtInp_4_Pin 			GPIO_PIN_13
#define ExtInp_4_GPIO_Port 		GPIOC

#define EncoderAIn_Pin        	GPIO_PIN_8
#define EncoderAIn_GPIO_Port  	GPIOA
#define EncoderBIn_Pin        	GPIO_PIN_9
#define EncoderBIn_GPIO_Port  	GPIOA
#define EncoderZF_Pin         	GPIO_PIN_3
#define EncoderZF_GPIO_Port		GPIOB
#define EncoderZF_EXTI_IRQn		EXTI3_IRQn

typedef enum {
  ControlState_CHECKED = 0,
  ControlState_CHECKIT
}ControlState; 
extern ControlState ControlUsart1_TransmitData;

void _Error_Handler(char *, int);

unsigned char step_motor_command;
unsigned char step_motor_speed[3];
unsigned int step_motor_requested_pos;
unsigned char timer_1_msec;
unsigned char timer_100_msec;
unsigned char input_status[4];
unsigned char send_RS485;

#endif /* __MAIN_H */
