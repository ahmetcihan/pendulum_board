#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f4xx_hal.h"
#include "main.h"

#define TIM_CR1_DR_CW		(uint16_t)0x0000
#define TIM_CR1_DR_CCW   	(uint16_t)0x0010

/****	Timer 1 icin	****/
extern int32_t signal_z_count;		//		Incremental ( Artimli ) Encoder Z Signal
extern int8_t  enc_signal_msb;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8; 

extern void _Error_Handler(char *, int);

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Timer3_OutputFrequency_Update( uint32_t TenTimesFreq );
void 	Timer3_AutoConsolidation_SpecialFunc	( uint32_t value );
int32_t Timer1_CalculateEncoderValue 			( void );

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
