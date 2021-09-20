#include "gpio.h"

//GPIO_PinState ExtInp1_PinState 	= GPIO_PIN_RESET,
//				ExtInp2_PinState	= GPIO_PIN_RESET,
//				ExtInp3_PinState 	= GPIO_PIN_RESET,
//				ExtInp4_PinState 	= GPIO_PIN_RESET;
//				RLY1_PinState 		= GPIO_PIN_RESET,
//				RLY2_PinState 		= GPIO_PIN_RESET,
//				RLY3_PinState 	 	= GPIO_PIN_RESET,
//				RLY4_PinState 	 	= GPIO_PIN_RESET;
void MX_GPIO_Init ( void ) {

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */  
  __HAL_RCC_GPIOA_CLK_ENABLE();  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
	
/*	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----		*/	
	
	/*	Configure  LED : GPIO pin Output Level	*/
	HAL_GPIO_WritePin	( Led_GPIO_Port , Led_Pin , GPIO_PIN_SET );
	/*	Configure  Dac_Sync : GPIO pin Output Level	*/
	HAL_GPIO_WritePin	( Dac_Sync_GPIO_Port , Dac_Sync_Pin , GPIO_PIN_RESET );
	/*	Configure  PulseOutEn : GPIO pin Output Level	*/
	HAL_GPIO_WritePin	( PulseOutEn_GPIO_Port , PulseOutEn_Pin , GPIO_PIN_RESET );
	/*	Configure  MotorDir : GPIO pin Output Level	*/
	HAL_GPIO_WritePin	( MotorDir_GPIO_Port , MotorDir_Pin , GPIO_PIN_RESET );
	/*	Configure  ADC_1_CS|ADC_2_CS|ADC_3_CS|ADC_4_CS : GPIO pin Output Level	*/
	HAL_GPIO_WritePin ( ADC_RST_GPIO_Port  , ADC_RST_Pin  , GPIO_PIN_SET );
	HAL_GPIO_WritePin ( ADC_1_CS_GPIO_Port , ADC_1_CS_Pin , GPIO_PIN_SET );
	HAL_GPIO_WritePin ( ADC_2_CS_GPIO_Port , ADC_2_CS_Pin , GPIO_PIN_SET );
	HAL_GPIO_WritePin ( ADC_3_CS_GPIO_Port , ADC_3_CS_Pin , GPIO_PIN_SET );
	HAL_GPIO_WritePin ( ADC_4_CS_GPIO_Port , ADC_4_CS_Pin , GPIO_PIN_SET );
	/*	Configure  RELAY_DRV_1|RELAY_DRV_2|RELAY_DRV_3|RELAY_DRV_4 : GPIO pin Output Level	*/
	HAL_GPIO_WritePin ( RELAY_DRV_1_GPIO_Port , RELAY_DRV_1_Pin , GPIO_PIN_RESET );
 	HAL_GPIO_WritePin ( RELAY_DRV_2_GPIO_Port , RELAY_DRV_2_Pin , GPIO_PIN_RESET );
	HAL_GPIO_WritePin ( RELAY_DRV_3_GPIO_Port , RELAY_DRV_3_Pin , GPIO_PIN_RESET );
	HAL_GPIO_WritePin ( RELAY_DRV_4_GPIO_Port , RELAY_DRV_4_Pin , GPIO_PIN_RESET );
/*	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----		*/

	/*Configure GPIO pin : Led */
	GPIO_InitStruct.Pin   = Led_Pin|Tx2En;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init ( Led_GPIO_Port , &GPIO_InitStruct );
	
//	 /*Configure GPIO pins : ExtInp_4 ExtInp_3 ExtInp_2 ExtInp_1 */
//  GPIO_InitStruct.Pin  = ExtInp_4_Pin | ExtInp_3_Pin | ExtInp_2_Pin | ExtInp_1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin  = INPUT_4_Pin | INPUT_3_Pin | INPUT_2_Pin | INPUT_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pin : RELAY_DRV_1|RELAY_DRV_2|RELAY_DRV_3|RELAY_DRV_4 */
  GPIO_InitStruct.Pin   = RELAY_DRV_1_Pin|RELAY_DRV_2_Pin|RELAY_DRV_3_Pin|RELAY_DRV_4_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init ( GPIOC , &GPIO_InitStruct );
	
  /*Configure GPIO pin : Dac_Sync_ */
  GPIO_InitStruct.Pin   = Dac_Sync_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init ( Dac_Sync_GPIO_Port , &GPIO_InitStruct );	


  /*Configure GPIO pin : ADC_RST */
  GPIO_InitStruct.Pin   = ADC_RST_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( ADC_RST_GPIO_Port , &GPIO_InitStruct );	
  /*Configure GPIO pins : ADC_1_CS */
  GPIO_InitStruct.Pin   = ADC_1_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init ( ADC_1_CS_GPIO_Port , &GPIO_InitStruct );
  /*Configure GPIO pin : ADC_1_RDYB */
  GPIO_InitStruct.Pin  = ADC_1_RDYB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init ( ADC_1_RDYB_GPIO_Port , &GPIO_InitStruct );	
  /*Configure GPIO pins : ADC_2_CS */
  GPIO_InitStruct.Pin   = ADC_2_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : ADC_2_RDYB */
  GPIO_InitStruct.Pin  = ADC_2_RDYB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init( ADC_2_RDYB_GPIO_Port , &GPIO_InitStruct ); 	
  /*Configure GPIO pin : ADC_3_CS */
  GPIO_InitStruct.Pin   = ADC_3_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init( ADC_3_CS_GPIO_Port , &GPIO_InitStruct );
  /*Configure GPIO pins : ADC_3_RDYB */
  GPIO_InitStruct.Pin  = ADC_3_RDYB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init( ADC_3_RDYB_GPIO_Port , &GPIO_InitStruct );
  /*Configure GPIO pin : ADC_4 */
  GPIO_InitStruct.Pin   = ADC_4_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init( ADC_4_CS_GPIO_Port , &GPIO_InitStruct );
  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin  = ADC_4_RDYB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init( ADC_4_RDYB_GPIO_Port , &GPIO_InitStruct);
	
  /*Configure GPIO pin : EncoderZF_Pin */
  GPIO_InitStruct.Pin  = EncoderZF_Pin;								
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 				
  GPIO_InitStruct.Pull = GPIO_NOPULL;          			
  HAL_GPIO_Init( EncoderZF_GPIO_Port , &GPIO_InitStruct );

  /*Configure GPIO pin : PulseOutEn */
  GPIO_InitStruct.Pin   = PulseOutEn_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init ( PulseOutEn_GPIO_Port , &GPIO_InitStruct );

  /*Configure GPIO pin : MotorDir */
  GPIO_InitStruct.Pin   = MotorDir_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init ( MotorDir_GPIO_Port , &GPIO_InitStruct );	

/*	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----	-----		*/

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority 	( ADC_4_RDYB_EXTI_IRQn , 0 , 0 );
  HAL_NVIC_EnableIRQ	( ADC_4_RDYB_EXTI_IRQn ) ;

  HAL_NVIC_SetPriority 	( ADC_3_RDYB_EXTI_IRQn , 0 , 0 );
  HAL_NVIC_EnableIRQ	( ADC_3_RDYB_EXTI_IRQn );

  HAL_NVIC_SetPriority 	( ADC_2_RDYB_EXTI_IRQn , 0 , 0 );
  HAL_NVIC_EnableIRQ	( ADC_2_RDYB_EXTI_IRQn );

  HAL_NVIC_SetPriority 	( ADC_1_RDYB_EXTI_IRQn , 0 , 0 );
  HAL_NVIC_EnableIRQ	( ADC_1_RDYB_EXTI_IRQn );
	
  HAL_NVIC_SetPriority  ( EncoderZF_EXTI_IRQn  , 0 , 0 );
  HAL_NVIC_EnableIRQ	( EncoderZF_EXTI_IRQn  );
}

