#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/****	Timer 1 icin	****/
int32_t  signal_z_count;		//		Incremental ( Artimli ) Encoder Z Signal
int8_t   enc_signal_msb;		//  

/* TIM1 init function */
/* TIM1_CH1 ~~~~~~ ENCODER_N */  
/* TIM1_CH2 ~~~~~~ ENCODER_P */
void MX_TIM1_Init ( void ) {
	TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);
		
	__HAL_TIM_ENABLE_IT	 ( &htim1 , TIM_IT_UPDATE ); 
	HAL_TIM_Encoder_Start( &htim1 , TIM_CHANNEL_ALL );

	signal_z_count = 0;
	enc_signal_msb = 0;
}
/* TIM3 init function */
/* TIM3_CH1[PulseOut][PB4]	----> 0.1 to 1 MHz frequency output	*/
void MX_TIM3_Init ( void ) {
  HAL_GPIO_WritePin( PulseOutEn_GPIO_Port , PulseOutEn_Pin , GPIO_PIN_SET );
	
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  HAL_TIM_MspPostInit(&htim3);
	
	HAL_TIM_Base_Start_IT( &htim3 );
	HAL_TIM_PWM_Start		 ( &htim3 , TIM_CHANNEL_1 );
	Timer3_OutputFrequency_Update( 25000 );
}
/* TIM4 init function */
/* Board uzerinde ki LedGreen[PA11] pinini toggle etmek icin */
void MX_TIM4_Init ( void ) {
  
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

	HAL_TIM_Base_Start_IT ( &htim4 );
}
/* TIM8 init function */	//	!! MotorDir Pinini kontrol et !!!
/* TIM8_CH4[DacPWM][PC9] 		----> PWM duty[ 0 to 14 bit ] modilation	*/
void MX_TIM8_Init ( void ) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 16383;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource( &htim8, &sClockSourceConfig ) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;		//2048
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

  HAL_TIM_MspPostInit(&htim8);
	
	HAL_TIM_Base_Start_IT( &htim8 );
	HAL_TIM_PWM_Start		 ( &htim8 , TIM_CHANNEL_4 );
}
/*	*/
void HAL_TIM_Encoder_MspInit 	( TIM_HandleTypeDef* tim_encoderHandle ) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if ( tim_encoderHandle->Instance == TIM1 ) {
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2 
    */
    GPIO_InitStruct.Pin = EncoderAIn_Pin|EncoderBIn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  }
}
void HAL_TIM_Base_MspInit 	 	( TIM_HandleTypeDef* tim_baseHandle ) 	 {
  if			( tim_baseHandle -> Instance == TIM3 ) {
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
	else if ( tim_baseHandle -> Instance == TIM4 ) {
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority ( TIM4_IRQn , 0 , 0 );
    HAL_NVIC_EnableIRQ	 ( TIM4_IRQn );
  }
	else if ( tim_baseHandle -> Instance == TIM8 ) {
    /* TIM8 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
  }
}
void HAL_TIM_MspPostInit		( TIM_HandleTypeDef* timHandle ) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if			( timHandle -> Instance == TIM3 ) {
    /**TIM3 GPIO Configuration    
    PB4     ------> TIM3_CH1 
    */
    GPIO_InitStruct.Pin = PulseOut_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(PulseOut_GPIO_Port, &GPIO_InitStruct);
  }
  else if ( timHandle -> Instance == TIM8 ) {
    /**TIM8 GPIO Configuration    
    PC8     ------> TIM8_CH3 
    */
    GPIO_InitStruct.Pin = DacPWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(DacPWM_GPIO_Port, &GPIO_InitStruct);
  }

}
/*	*/
void HAL_TIM_Encoder_MspDeInit 	( TIM_HandleTypeDef* tim_encoderHandle ) {
  if( tim_encoderHandle -> Instance == TIM1 ) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();  
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2 
    */
    HAL_GPIO_DeInit(GPIOA, EncoderAIn_Pin|EncoderBIn_Pin);
    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  }
}
void HAL_TIM_Base_MspDeInit 	( TIM_HandleTypeDef* tim_baseHandle ) {
  if			( tim_baseHandle -> Instance == TIM3 ) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }

	else if ( tim_baseHandle -> Instance == TIM4 ) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
    /* TIM4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
	else if	( tim_baseHandle -> Instance == TIM8 ) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
  }
} 
/**
  * @brief  Timer_3 , Channel_1'e bagli PULSE_OUT_Pin pininden 0.1Hz~1MHz arasi frekans üretir.  
  * @param  TenTimesFreq : Girilen TenTimesFreq degerinin onda biri Timer3 Ch3 kanalindan sinyal olusturur.
	*		@example TenTimesFreq = 1 				  ise PULSE_OUT_Pin 0.1Hz output frequency	
	*		@example TenTimesFreq = 10 000  		ise PULSE_OUT_Pin 1 KHz output frequency	
	*		@example TenTimesFreq = 10 000 000  ise PULSE_OUT_Pin 1 MHz output frequency	
  * @retval [ void ]
  */
void Timer3_OutputFrequency_Update( uint32_t TenTimesFreq ) {
		uint32_t psc_crossed_arr;
		uint16_t register_psc, register_arr;
		psc_crossed_arr = 900000000 / TenTimesFreq;
		if   ( psc_crossed_arr <= 65535 ) {
			register_psc = 1;
			register_arr = psc_crossed_arr;
		}
		else									            {
			uint16_t casual_val = psc_crossed_arr/65535;
			register_psc = casual_val+1;
			register_arr = psc_crossed_arr/register_psc;	
		}
		TIM3->PSC = register_psc-1;
		TIM3->ARR = register_arr-1;
		TIM3->CCR1= register_arr/2;
}
