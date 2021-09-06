#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

extern TIM_HandleTypeDef 	htim1;
extern TIM_HandleTypeDef 	htim4;

extern SPI_HandleTypeDef 	hspi2;

extern UART_HandleTypeDef 	huart1;
extern UART_HandleTypeDef 	huart2;

extern DMA_HandleTypeDef 	hdma_usart1_rx;
extern DMA_HandleTypeDef 	hdma_usart1_tx;

extern UART_HandleTypeDef 	huart4;
extern DMA_HandleTypeDef 	hdma_uart4_tx;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/  
/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler 			( void ) {
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
	
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */ 
  /* USER CODE END NonMaskableInt_IRQn 1 */
}
/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler		( void ) {
  while ( 1 ) {
  }
}
/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler 		( void ) {
  while (1)
  {
  }
}
/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler	 	( void ) {
  while (1)
  {
  }
}
/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler		( void ) {
  while (1)
  {
  }
}
/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler 			( void ) {
}
/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler		( void ) {
}
/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler			( void ) {

}
/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler		( void ) {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
/**
* @brief This function handles EXTI line 0 interrupt.
*/
void EXTI0_IRQHandler 	( void ) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
* @brief This function handles EXTI line 1 interrupt.
*/
void EXTI1_IRQHandler 	( void ) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles EXTI line 2 interrupt.
*/
void EXTI2_IRQHandler 	( void ) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
/**
* @brief This function handles EXTI line 3 interrupt.
*/
void EXTI3_IRQHandler 	( void ) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}
/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler ( void ) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}
/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler ( void ) {
  HAL_TIM_IRQHandler(&htim1);
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler		( void ) {
  HAL_TIM_IRQHandler(&htim4);
}

/**
* @brief This function handles SPI2 global interrupt.
*/
void SPI2_IRQHandler		(	void ) {
  HAL_SPI_IRQHandler(&hspi2);
} 

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler  ( void ) {
  HAL_UART_IRQHandler(&huart1);
}
void USART2_IRQHandler  ( void ) {
  HAL_UART_IRQHandler(&huart2);
}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler		( void ) { 
  HAL_UART_IRQHandler(&huart4);
}
/**
* @brief This function handles DMA1 stream4 global interrupt.
*/
void DMA1_Stream4_IRQHandler ( void ) {
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
}
/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler ( void ) {
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
* @brief This function handles DMA2 stream7 global interrupt.
*/
void DMA2_Stream7_IRQHandler ( void ) {
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}
