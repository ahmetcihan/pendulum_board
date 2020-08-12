#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler 		( void );
void HardFault_Handler	( void );
void MemManage_Handler 	( void );
void BusFault_Handler	( void );
void UsageFault_Handler	( void );
void SVC_Handler 		( void );
void DebugMon_Handler	( void );
void PendSV_Handler		( void );
void SysTick_Handler	( void );

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

void EXTI0_IRQHandler 			( void );
void EXTI1_IRQHandler 			( void );
void EXTI2_IRQHandler 			( void );
void EXTI3_IRQHandler 			( void );
void EXTI9_5_IRQHandler			( void );
void TIM1_UP_TIM10_IRQHandler	( void );
void TIM4_IRQHandler			( void );
void SPI2_IRQHandler			( void );
void USART1_IRQHandler  		( void );

void DMA1_Stream4_IRQHandler 	( void );
void DMA2_Stream2_IRQHandler 	( void );
void DMA2_Stream7_IRQHandler 	( void );

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
