#ifndef __usart_H
#define __usart_H

#include "stm32f4xx_hal.h"
#include "main.h" 

#define USART_TX_ARRAY_SIZE 			(uint8_t)120
#define USART_RX_ARRAY_SIZE 			(uint8_t)120


void MX_USART1_UART_Init(void);
void MX_USART2_Init(void);
void MX_UART4_Init(void);

uint32_t CyclicRedundancyCheck(uint8_t* data, uint8_t length );
void USART1_receive_operations(void);
void PRESS_CONV_CommandOperating(void);
void PRESS_GAIN_CommandOperating(void);
void PRESS_TARE_CommandOperating(void);
void PRESS_PRINT_CommandOperating(void);
void PRESS_CALSEND_CommandOperating(void);
void PRESS_ANS_Command(void);
void MASTER_send_RS485_data_to_motor(void);
void usart2_handle(void);

double evaluate_calibrated_values(uint8_t no);
void slope_calculation(uint8_t no);

extern UART_HandleTypeDef huart1,huart2,huart4;

struct _usart{
	uint32_t buffer_clear_timer;
	uint8_t clear_buffer;
	uint8_t instant_data;
	uint8_t tx_amount;
	uint8_t rx_indeks;
	uint8_t data_received;
	uint8_t tx[USART_TX_ARRAY_SIZE];
	uint8_t rx[USART_RX_ARRAY_SIZE];
};
extern struct _usart usart1,usart2,usart4;

union 	_char_to_f {
	float float_val;
	u8 u8_val[4];
	char s8_val[4];
	s32 int_val;
};
extern union _char_to_f char_to_f;

u32 stepper_abs_pos;
u8 u1_ctrl1,u2_ctrl1,u3_ctrl1,u4_ctrl1;
u8 active_cal_channel;
u8 usart_debugger;
u8 usart1_received;
u8 usart1_transmit;

#endif /*__ usart_H */
