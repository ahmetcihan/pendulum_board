#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

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
void MX_TIM3_Init ( void ) {
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	HAL_GPIO_WritePin( PulseOutEn_GPIO_Port , PulseOutEn_Pin , GPIO_PIN_SET );

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
	
	HAL_TIM_Base_Start_IT(&htim3 );
	HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_1 );
	Timer3_OutputFrequency_Update( 25000 );
}
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
	HAL_TIM_Base_Start_IT(&htim8 );
	HAL_TIM_PWM_Start(&htim8 , TIM_CHANNEL_4 );
}
void HAL_TIM_Encoder_MspInit 	( TIM_HandleTypeDef* tim_encoderHandle ) {
	GPIO_InitTypeDef GPIO_InitStruct;
	if ( tim_encoderHandle->Instance == TIM1 ) {
		__HAL_RCC_TIM1_CLK_ENABLE();
		GPIO_InitStruct.Pin = EncoderAIn_Pin|EncoderBIn_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	}
}
void HAL_TIM_Base_MspInit 	 	( TIM_HandleTypeDef* tim_baseHandle ) 	 {
	if(tim_baseHandle -> Instance == TIM3 ) {
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
	else if ( tim_baseHandle -> Instance == TIM4 ) {
		__HAL_RCC_TIM4_CLK_ENABLE();
		HAL_NVIC_SetPriority ( TIM4_IRQn , 0 , 0 );
		HAL_NVIC_EnableIRQ	 ( TIM4_IRQn );
	}
	else if ( tim_baseHandle -> Instance == TIM8 ) {
		__HAL_RCC_TIM8_CLK_ENABLE();
	}
}
void HAL_TIM_MspPostInit		( TIM_HandleTypeDef* timHandle ) {
	GPIO_InitTypeDef GPIO_InitStruct;
	if( timHandle -> Instance == TIM3 ) {
		GPIO_InitStruct.Pin = PulseOut_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(PulseOut_GPIO_Port, &GPIO_InitStruct);
	}
	else if ( timHandle -> Instance == TIM8 ) {
		GPIO_InitStruct.Pin = DacPWM_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(DacPWM_GPIO_Port, &GPIO_InitStruct);
	}
}
void HAL_TIM_Encoder_MspDeInit 	( TIM_HandleTypeDef* tim_encoderHandle ) {
	if( tim_encoderHandle -> Instance == TIM1 ) {
		__HAL_RCC_TIM1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, EncoderAIn_Pin|EncoderBIn_Pin);
		HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
	}
}
void HAL_TIM_Base_MspDeInit 	( TIM_HandleTypeDef* tim_baseHandle ) {
	if(tim_baseHandle -> Instance == TIM3 ) {
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
	else if ( tim_baseHandle -> Instance == TIM4 ) {
		__HAL_RCC_TIM4_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(TIM4_IRQn);
	}
	else if	( tim_baseHandle -> Instance == TIM8 ) {
		__HAL_RCC_TIM8_CLK_DISABLE();
	}
} 
void Timer3_OutputFrequency_Update( uint32_t TenTimesFreq ) {
	uint32_t psc_crossed_arr;
	uint16_t register_psc, register_arr;

	psc_crossed_arr = 900000000 / TenTimesFreq;
	if   ( psc_crossed_arr <= 65535 ) {
		register_psc = 1;
		register_arr = psc_crossed_arr;
	}
	else{
		uint16_t casual_val = psc_crossed_arr/65535;
		register_psc = casual_val+1;
		register_arr = psc_crossed_arr/register_psc;
	}
	TIM3->PSC = register_psc-1;
	TIM3->ARR = register_arr-1;
	TIM3->CCR1= register_arr/2;
}
void Timer3_AutoConsolidation_SpecialFunc( uint32_t value ) {
	uint16_t register_psc, register_arr;
	uint32_t val;

	if(value < 6710886  ) {
		val		= value / 2;
		value = val*50;
		value = value / 10;
	}
	else if(value < 11000000 ) {
		value = 11000000;
	}
	else if(value < 0xFFFFFF ) {
		value = 16777000;
	}
	else{
		value = 0xFFFFFF;
	}

	if(value <= 65535 ) {
		register_psc = 0;
		register_arr = value;
	}
	else{
		if(value == 16777000 ) {
			register_arr = (value/256);
			register_psc = 1024;
		}
		else if( value == 11000000 ) {
			register_arr = (value/256);
			register_psc = 768;
		}
		else if(value == 0xFFFFFF ) {
			register_arr = 0;
			register_psc = 0;
		}
		else{
			register_arr = (value/256);
			register_psc = 256;
		}
	}
	TIM3->PSC = register_psc;
	TIM3->ARR = register_arr;
	TIM3->CCR1= (TIM3->ARR)/2;
}
int32_t Timer1_CalculateEncoderValue ( void ) {
	int32_t encoderval = enc_signal_msb * ( 65536 ) + TIM1->CNT;;

	return (int32_t)(encoderval);
}
