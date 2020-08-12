#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h" 

#define PRESS_RELAY_ON_AutoManual	  	HAL_GPIO_WritePin( RELAY_DRV_2_GPIO_Port , RELAY_DRV_2_Pin , GPIO_PIN_SET 	)
#define PRESS_RELAY_OFF_AutoManual		HAL_GPIO_WritePin( RELAY_DRV_2_GPIO_Port , RELAY_DRV_2_Pin , GPIO_PIN_RESET )
#define PRESS_RELAY_ON_StartStop 		HAL_GPIO_WritePin( RELAY_DRV_1_GPIO_Port , RELAY_DRV_1_Pin , GPIO_PIN_SET 	)
#define PRESS_RELAY_OFF_StartStop		HAL_GPIO_WritePin( RELAY_DRV_1_GPIO_Port , RELAY_DRV_1_Pin , GPIO_PIN_RESET )

#define USART1_TX_ARRAY_SIZE 			(uint8_t)40
#define USART1_RX_ARRAY_SIZE 			(uint8_t)90

extern uint8_t usarttx[USART1_TX_ARRAY_SIZE];
extern uint8_t usartrx[USART1_RX_ARRAY_SIZE];
extern uint8_t rx_indeks;
extern uint8_t casual_rx_data;
extern uint8_t usart_tx_size;
extern uint8_t TxAmound;

extern uint32_t  buffer_clear_timer;	
extern uint8_t 	 buffer_cleared;			

extern UART_HandleTypeDef huart1; 
extern UART_HandleTypeDef huart4;

extern 	void _Error_Handler			( char *, int );
		void MX_USART1_UART_Init	( void );
		void MX_UART4_Init			( void );

uint32_t CyclicRedundancyCheck 			( uint8_t* data, uint8_t length );
void 	 UsartReceiveData_SearchCommand ( void );
void 	 PRESS_CONV_CommandOperating	( void );
void	 PRESS_GAIN_CommandOperating	( void );
void 	 PRESS_TARE_CommandOperating	( void );
void 	 PRESS_PRINT_CommandOperating	( void );
void	 PRESS_CALSEND_CommandOperating ( void );

void 		 PRESS_ANS_Command 			( void );

/*	A.C.AKINCA eklemeleri	*/
struct chan {
    uint32_t tare;
    int8_t tare_sign;

    uint32_t zero_raw;
    int8_t zero_raw_sign;

    uint32_t raw;
    int8_t raw_sign;

    uint8_t point_number;
    int8_t cal_zero_sign;

    float calibrated;
    uint32_t cal_raw_value[8];
    float cal_point_value[8];
    double slope[7];
};
extern struct chan channel[8];
extern uint8_t active_cal_channel,calculate_slopes;	//	unsigned char->uint8_t

void evaluate_calibrated_values 		( uint8_t no );
void slope_calculation					( uint8_t i  );

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */
