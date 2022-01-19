#ifndef __MAIN_H
#define __MAIN_H

typedef char s8;
typedef unsigned char u8;
typedef unsigned int u32;
typedef signed int s32;

#define Tx2En 					GPIO_PIN_12

#define Led_Pin 				GPIO_PIN_11
#define Led_GPIO_Port 			GPIOA

#define RELAY_DRV_3_Pin       	GPIO_PIN_3
#define RELAY_DRV_3_GPIO_Port	GPIOC
#define RELAY_DRV_4_Pin       	GPIO_PIN_4
#define RELAY_DRV_4_GPIO_Port	GPIOC

#define ADC_RST_Pin 			GPIO_PIN_7
#define ADC_RST_GPIO_Port 		GPIOC
#define ADC_1_CS_Pin 			GPIO_PIN_12
#define ADC_1_CS_GPIO_Port 		GPIOB
#define ADC_1_RDYB_Pin 			GPIO_PIN_6
#define ADC_1_RDYB_GPIO_Port 	GPIOC
#define ADC_1_RDYB_EXTI_IRQn 	EXTI9_5_IRQn
#define ADC_2_CS_Pin 			GPIO_PIN_10
#define ADC_2_CS_GPIO_Port 		GPIOB  
#define ADC_2_RDYB_Pin 			GPIO_PIN_2
#define ADC_2_RDYB_GPIO_Port 	GPIOB
#define ADC_2_RDYB_EXTI_IRQn 	EXTI2_IRQn  
#define ADC_3_CS_Pin 			GPIO_PIN_4
#define ADC_3_CS_GPIO_Port 		GPIOA
#define ADC_3_RDYB_Pin 			GPIO_PIN_1
#define ADC_3_RDYB_GPIO_Port 	GPIOB
#define ADC_3_RDYB_EXTI_IRQn 	EXTI1_IRQn  
#define ADC_4_CS_Pin 			GPIO_PIN_5
#define ADC_4_CS_GPIO_Port 		GPIOC
#define ADC_4_RDYB_Pin 			GPIO_PIN_0
#define ADC_4_RDYB_GPIO_Port 	GPIOB
#define ADC_4_RDYB_EXTI_IRQn 	EXTI0_IRQn 

#define Dac_Sync_Pin 			GPIO_PIN_10
#define Dac_Sync_GPIO_Port    	GPIOA
#define DacPWM_Pin 				GPIO_PIN_9
#define DacPWM_GPIO_Port		GPIOC

#define PulseOut_Pin 			GPIO_PIN_4
#define PulseOut_GPIO_Port		GPIOB
#define PulseOutEn_Pin 			GPIO_PIN_5
#define PulseOutEn_GPIO_Port  	GPIOB
#define MotorDir_Pin 			GPIO_PIN_8
#define MotorDir_GPIO_Port    	GPIOC

#define RELAY_DRV_1_Pin       	GPIO_PIN_1
#define RELAY_DRV_1_GPIO_Port	GPIOC
#define RELAY_DRV_2_Pin       	GPIO_PIN_2
#define RELAY_DRV_2_GPIO_Port	GPIOC

#define INPUT_1_Pin 			GPIO_PIN_0
#define INPUT_2_Pin 			GPIO_PIN_15
#define INPUT_3_Pin 			GPIO_PIN_14
#define INPUT_4_Pin 			GPIO_PIN_13
#define INPUT_Port		 		GPIOC

#define ExtInp_1_Pin 			GPIO_PIN_0
#define ExtInp_1_GPIO_Port 		GPIOC
#define ExtInp_2_Pin 			GPIO_PIN_15
#define ExtInp_2_GPIO_Port 		GPIOC
#define ExtInp_3_Pin 			GPIO_PIN_14
#define ExtInp_3_GPIO_Port 		GPIOC
#define ExtInp_4_Pin 			GPIO_PIN_13
#define ExtInp_4_GPIO_Port 		GPIOC

#define EncoderAIn_Pin        	GPIO_PIN_8
#define EncoderAIn_GPIO_Port  	GPIOA
#define EncoderBIn_Pin        	GPIO_PIN_9
#define EncoderBIn_GPIO_Port  	GPIOA
#define EncoderZF_Pin         	GPIO_PIN_3
#define EncoderZF_GPIO_Port		GPIOB
#define EncoderZF_EXTI_IRQn		EXTI3_IRQn

#define Electromechanic_RELAY_ON_AutoManual	  	HAL_GPIO_WritePin( RELAY_DRV_2_GPIO_Port , RELAY_DRV_2_Pin , GPIO_PIN_SET 	)
#define Electromechanic_RELAY_OFF_AutoManual	HAL_GPIO_WritePin( RELAY_DRV_2_GPIO_Port , RELAY_DRV_2_Pin , GPIO_PIN_RESET )
#define Electromechanic_RELAY_ON_StartStop 		HAL_GPIO_WritePin( RELAY_DRV_1_GPIO_Port , RELAY_DRV_1_Pin , GPIO_PIN_SET 	)
#define Electromechanic_RELAY_OFF_StartStop		HAL_GPIO_WritePin( RELAY_DRV_1_GPIO_Port , RELAY_DRV_1_Pin , GPIO_PIN_RESET )

#define Electromechanic_ServoStop           	HAL_GPIO_WritePin( PulseOutEn_GPIO_Port	 , PulseOutEn_Pin  , GPIO_PIN_RESET )
#define Electromechanic_ServoStart          	HAL_GPIO_WritePin( PulseOutEn_GPIO_Port  , PulseOutEn_Pin  , GPIO_PIN_SET	  )
#define Electromechanic_ServoReverse        	HAL_GPIO_WritePin( MotorDir_GPIO_Port	 , MotorDir_Pin 	 , GPIO_PIN_RESET )
#define Electromechanic_ServoForward        	HAL_GPIO_WritePin( MotorDir_GPIO_Port 	 , MotorDir_Pin    , GPIO_PIN_SET   )

#define STEPPER_COMMAND_RUN_DOWN    1
#define STEPPER_COMMAND_RUN_UP      2
#define STEPPER_COMMAND_STOP        3
#define STEPPER_COMMAND_POS_RESET   4
#define STEPPER_COMMAND_GO_POS      5

#define TMC_STOP    				0
#define TMC_RUN     				1
#define TMC_AUTOTUNING     			2
#define TMC_PENDULUM_PID        	5
#define TMC_PENDULUM_HEADUP     	6

#define NB               0
#define NM               1
#define NS               2
#define ZO               3
#define PS               4
#define PM               5
#define PB               6

void SystemClock_Config(void);
void _Error_Handler(char *, int);
s32 EMA_raw(s32 raw_signal, u8 filter_coefficient);
s32 SMA_raw(s32 raw_signal, u8 filter_coefficient);
float EMA_load(float *raw_signal, u8 filter_coefficient);
float SMA_load(float load_signal,u8 filter_coefficient);
float SMA_pace(float pace_val,u8 filter_coefficient);
float alpha_beta_filter(float input);
s32 SMA_mid_point(s32 raw_signal,u8 filter_coefficient);

void bessel_filter_coeffs_for_raw(void);
float bessel_filter_for_raw(float input);
void bessel_filter_coeffs_for_pace(void);
float bessel_filter_for_pace(float input);
void butterworth_lpf_coeffs(float *a, float *b);
float butterworth_filter(float input, float *a, float *b, float *x, float *y);

void my_debugger(u8 u8_v, s32 s32_v, float f_0, float f_1, float f_2);

float uf(float x,float a,float  b,float c);
float cuf(float x,float a,float b,float c);
float ufl(float x,float a,float b);
float cufl(float x,float a,float b);
float ufr(float x,float a,float b);
float cufr(float x,float a,float b);
float fand(float a,float b);
float forr(float a,float b);
float PID_with_fuzzy(void);

struct _cal{
    double slope[7];
    double assigned_val[8];
    float tare_val;
    float absolute_calibrated;
    float calibrated;
    s32 signed_raw;
    s32 signed_raw_filtered;
    u32 unsigned_raw;
    int real_val[8];
    u8 point_no;
};
extern struct _cal cal[4];

struct _par{
	u32 test_start_speed;
	u32 step_first_speed;
	u32 step_second_speed;
	u32 step_transition_time;
	float failure_threshold;
	float zero_suppression;
	float pace_rate;
	u8 break_percentage;
	float kp;
	float ki;
	float kd;
};
extern struct _par parameters;

float old_load;
float filtered_load;
float max_load_value;
float unfiltered_pace_rate;
float filtered_pace_rate;
float filtered_SMA;
float filtered_pace_bessel;
float filtered_pace_butterworth;
float bessel_raw_ax[3], bessel_raw_by[3];
float bessel_pace_ax[3], bessel_pace_by[3];
float butterworth_a[3],butterworth_b[3];
float butterworth_x[2],butterworth_y[2];
float calculated_kp;
float calculated_ki;
float calculated_kd;

s32 encoder_value;
s32 abs_encoder;

u32 PID_delta_t;
u32 step_motor_requested_pos;
u32 _10_usec_counter;
u32 step_timer;
u32 plot_counter_1_msec;
u8 PID_first_in;
u8 step_motor_command;
u8 step_motor_speed[3];
u8 timer_1_msec;
u8 timer_2_msec;
u8 timer_10_msec;
u8 timer_100_msec;
u8 input_status[4];
u8 send_RS485;
u8 max1_dataready;
u8 max2_dataready;
u8 max3_dataready;
u8 max4_dataready;
u8 control_process_tmp;
u8 TMC_command;
u8 step_response_first_in;
u8 autotuning_in_operation;
u8 PID_in_operation;
u8 autotuning_is_finished;
u8 mid_point_up_cmd;
u8 mid_point_down_cmd;

struct _pend{
	u32 head_change_timer;
	u32 headshake_speed;

	u32 mid_point;
	u32 top_boundary;
	u32 filtered_mid_point;

	float kp;
	float ki;
	float kd;
	u8 pid_tmp;

	u8 head_up_tmp;

};
extern struct _pend pendulum;


#endif /* __MAIN_H */
