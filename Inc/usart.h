#ifndef __usart_H
#define __usart_H

#include "stm32f4xx_hal.h"
#include "main.h" 

typedef char s8;
typedef unsigned char u8;
typedef unsigned int u32;
typedef signed int s32;

#define USART_TX_ARRAY_SIZE 			(uint8_t)120
#define USART_RX_ARRAY_SIZE 			(uint8_t)120

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

struct _cal{
    double slope[7];
    double assigned_val[8];
    float tare_val;
    float absolute_calibrated;
    int real_val[8];
    u8 point_no;
};
extern struct _cal cal[4];

extern UART_HandleTypeDef huart1; 
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

void MX_USART1_UART_Init(void);
void MX_USART2_Init(void);
void MX_UART4_Init(void);

uint32_t CyclicRedundancyCheck(uint8_t* data, uint8_t length );
void UsartReceiveData_SearchCommand(void);
void PRESS_CONV_CommandOperating(void);
void PRESS_GAIN_CommandOperating(void);
void PRESS_TARE_CommandOperating(void);
void PRESS_PRINT_CommandOperating(void);
void PRESS_CALSEND_CommandOperating(void);
void PRESS_ANS_Command(void);
void MASTER_send_RS485_data_to_motor(void);
void usart2_handle(void);

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

    s32 signed_raw;
};
extern struct chan channel[8];

double evaluate_calibrated_values(uint8_t no);
void slope_calculation(uint8_t no);

unsigned int stepper_abs_pos;
uint8_t active_cal_channel;
u8 usart_debugger;

#endif /*__ usart_H */
