#include "dma.h"

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init ( void ) {
  
	/* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
 
 /* DMA interrupt init */
	
  /* DMA1_Stream4_IRQn interrupt configuration */
	/* hdma_uart4_tx	*/
//  HAL_NVIC_SetPriority ( DMA1_Stream4_IRQn , 0 , 0 );
//  HAL_NVIC_EnableIRQ	 ( DMA1_Stream4_IRQn );
	
  /* DMA2_Stream2_IRQn interrupt configuration */
  /* hdma_usart1_rx	*/
  HAL_NVIC_SetPriority 	( DMA2_Stream2_IRQn , 0 , 0 );
  HAL_NVIC_EnableIRQ	( DMA2_Stream2_IRQn );
	
  /* DMA2_Stream7_IRQn interrupt configuration */
	/* hdma_usart1_tx	*/
  HAL_NVIC_SetPriority 	( DMA2_Stream7_IRQn , 0 , 0 );
  HAL_NVIC_EnableIRQ	( DMA2_Stream7_IRQn );
}
