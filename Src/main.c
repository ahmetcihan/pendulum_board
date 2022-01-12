#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "max11254.h"
#include "math.h"

struct _cal cal[4];
struct _par parameters;
struct _pend pendulum;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1) {
		usart1.buffer_clear_timer = 200;
		usart1.clear_buffer = 1;
		usart1.rx[usart1.rx_indeks] = usart1.instant_data;
		if (usart1.rx[usart1.rx_indeks] == 0x0A && usart1.rx[usart1.rx_indeks - 1] == 0x0D) {
			usart1_received = 1;
		}
		usart1.rx_indeks++;
		if (usart1.rx_indeks > USART_RX_ARRAY_SIZE) {
			usart1.rx_indeks--;
			for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE - 1; i++)
				usart1.rx[i] = usart1.rx[i + 1];
		}
	}
	if (huart->Instance == USART2) {
		usart2.buffer_clear_timer = 200;
		usart2.clear_buffer = 1;
		usart2.rx[usart2.rx_indeks] = usart2.instant_data;

		if (usart2.rx[usart2.rx_indeks] == 0x0A && usart2.rx[usart2.rx_indeks - 1] == 0x0D) {
			usart2.data_received = 1;
		}
		if (usart2.rx_indeks < USART_RX_ARRAY_SIZE) usart2.rx_indeks++;
	    HAL_UART_Receive_IT(&huart2, &usart2.instant_data,1);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		HAL_GPIO_WritePin( GPIOA , Tx2En , GPIO_PIN_RESET );
	}
}
void HAL_TIM_PeriodElapsedCallback	( TIM_HandleTypeDef *htim ) {		//	Timer Interrupt Fonksiyonu
	static uint32_t usn10;

	if (htim->Instance == TIM1) {
		((TIM1->CR1 & TIM_CR1_DIR) == TIM_CR1_DR_CW ) ?
				enc_signal_msb++ : enc_signal_msb--;
	}
	if (htim->Instance == TIM4) {
		usn10++;
		_10_usec_counter++;

		if(usart1.buffer_clear_timer > 0) usart1.buffer_clear_timer--;
		if(usart2.buffer_clear_timer > 0) usart2.buffer_clear_timer--;

		if ((usn10 % 100) == 0) {
			timer_1_msec = 1;
			PID_delta_t++;
			step_timer++;
			plot_counter_1_msec++;
		}
		if ((usn10 % 200) == 0) {
			timer_2_msec = 1;
		}
		if ((usn10 % 1000) == 0) {
			timer_10_msec = 1;
		}
		if ((usn10 % 10000) == 0) {
			timer_100_msec = 1;
		}
		if (usn10 == 100000) {
			usn10 = 0;
		}
	}
}
void HAL_SYSTICK_Callback			( void ) {
	static uint8_t ledcount = 0;
	ledcount++;
	if( ledcount >= 99 ) {
		ledcount = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { 						//	EXTI Interrupt Fonksiyonu
	if (GPIO_Pin == ADC_1_RDYB_Pin)
		max1_dataready = 1;
	if (GPIO_Pin == ADC_2_RDYB_Pin)
		max2_dataready = 1;
	if (GPIO_Pin == ADC_3_RDYB_Pin)
		max3_dataready = 1;
	if (GPIO_Pin == ADC_4_RDYB_Pin)
		max4_dataready = 1;
	if (GPIO_Pin == EncoderZF_Pin){
		((TIM1->CR1 & TIM_CR1_DIR) == TIM_CR1_DR_CW ) ?	signal_z_count++ : signal_z_count--;
	}
}
void read_inputs(void){
	input_status[0] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_1_Pin);
	input_status[1] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_2_Pin);
	input_status[2] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_3_Pin);
	input_status[3] = HAL_GPIO_ReadPin(INPUT_Port,INPUT_4_Pin);
}
void channel_operation(u8 no){
	OperatingMaxExtiRdbyControl(no+1);
	cal[no].unsigned_raw = MAX[resultBinding[no]/6].chResult[resultBinding[no]%6];
	cal[no].signed_raw = cal[no].unsigned_raw;
	if   ( MAX[no].polarity == POLARITY_BIPOLAR ) {
		if (cal[no].unsigned_raw > 0x7FFFFF) {
			cal[no].unsigned_raw = (0xFFFFFF + 1) - cal[no].unsigned_raw;
			cal[no].signed_raw = -1* (s32)cal[no].unsigned_raw;
		}
	}
	if(no == 0){
		cal[0].signed_raw_filtered = bessel_filter_for_raw((float)cal[0].signed_raw);
	}
	else{
		cal[no].signed_raw_filtered = cal[no].signed_raw;
	}

	cal[no].calibrated = evaluate_calibrated_values(no);
}
void my_debugger(u8 u8_v, s32 s32_v, float f_0, float f_1, float f_2){
	usart_debugger_u8 = u8_v;
	usart_debugger_s32 = s32_v;
	usart_debugger_float[0] = f_0;
	usart_debugger_float[1] = f_1;
	usart_debugger_float[2] = f_2;
}

void usart_buffer_clearance(void){
	if (usart2.data_received == 1) {
		usart2.data_received = 0;
		usart2_handle();
	}
	if (usart2.clear_buffer == 1) {
		if (usart2.buffer_clear_timer == 0) {
			usart2.clear_buffer = 0;
			usart2.rx_indeks = 0;
			for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE ; i++) {
				usart2.rx[i] = 0;
			}
		}
	}

	if (usart1.clear_buffer == 1) {
		if (usart1.buffer_clear_timer == 0) {
			usart1.clear_buffer = 0;
			usart1.rx_indeks = 0;
			for (uint8_t i = 0; i < USART_RX_ARRAY_SIZE ; i++) {
				usart1.rx[i] = 0;
			}
		}
	}
}
float PID(void){
    static float last_error[3] = {0};
    static float output = 0;
    //static u8 first_move = 0;

    float error = 0;
    float a,b,c;
    float Ts = (float)PID_delta_t;
    static float kp,ki,kd;

    if(PID_first_in){
        PID_first_in = 0;
        //first_move = 7;
        output = parameters.test_start_speed;
        last_error[0] = 0;
        last_error[1] = 0;
        last_error[2] = 0;
        kp = parameters.kp;
        ki = parameters.ki;
        kd = parameters.kd;
    }
    else{
        error = parameters.pace_rate - filtered_pace_rate;

        last_error[2] = last_error[1];
        last_error[1] = last_error[0];
        last_error[0] = error;

        a = kp + kd / (float)Ts;
        b = -kp + (ki * (float)Ts) - ((float)2 * kd)/(float)Ts;
        c = kd / Ts;

        output = output + (a * last_error[0] + b * last_error[1] + c * last_error[2]);
        if(output > 200000) output = 200000;

        if(output >= 0){
            step_motor_command = STEPPER_COMMAND_RUN_UP;
        }
        else{
            step_motor_command = STEPPER_COMMAND_RUN_DOWN;
        }

    }
    //my_debugger(PID_delta_t,output,error,filtered_pace_rate,parameters.pace_rate);

    PID_delta_t = 0;

    return fabs(output);
}
void pendulum_PID(void){
    static float last_error[3] = {0};
    static float output = 0;
    float err = 0;
    float a,b,c;
    float Ts = (float)PID_delta_t;
    static float kp,ki,kd;

    u32 plain_speed;

	switch(pendulum.pid_tmp){
	case 0:
        output = 0;
        last_error[0] = 0;
        last_error[1] = 0;
        last_error[2] = 0;
        kp = pendulum.kp * 1000;
        ki = pendulum.ki * 1000;
        kd = pendulum.kd * 1000;
        pendulum.pid_tmp = 1;
		step_motor_speed[0] = 0;
		step_motor_speed[1] = 0;
		step_motor_speed[2] = 0;
		break;
	case 1:
		err = (s32)pendulum.mid_point - abs_encoder;

		if(abs_encoder < (pendulum.mid_point - pendulum.top_boundary)){
			plain_speed = 0;
			output = 0;
	        last_error[0] = 0;
	        last_error[1] = 0;
	        last_error[2] = 0;
		}
		else if(abs_encoder > (pendulum.mid_point + pendulum.top_boundary)){
			plain_speed = 0;
			output = 0;
	        last_error[0] = 0;
	        last_error[1] = 0;
	        last_error[2] = 0;
		}
		else{
	        last_error[2] = last_error[1];
	        last_error[1] = last_error[0];
	        last_error[0] = err;

	        a = kp + kd / (float)Ts;
	        b = -kp + (ki * (float)Ts) - ((float)2 * kd)/(float)Ts;
	        c = kd / Ts;

	        //output = output + (a * last_error[0] + b * last_error[1] + c * last_error[2]);
	        output = (a * last_error[0] + b * last_error[1] + c * last_error[2]);

	        if(output >= 0){
	            step_motor_command = STEPPER_COMMAND_RUN_UP;
	        }
	        else{
	            step_motor_command = STEPPER_COMMAND_RUN_DOWN;
	        }
	        plain_speed = fabs(output);

	        if(fabs(err) < pendulum.tolerance){
	        	if(fabs(output) < (2000 * pendulum.tolerance)){
		        	plain_speed = 0;
	        	}
	        }
//	        if((abs_encoder < 2050)&&(abs_encoder > 1950)){
//	        	//filter top point
//	        	//pendulum.filtered_mid_point = SMA_mid_point(abs_encoder,63);
//	        	pendulum.filtered_mid_point = butterworth_filter(abs_encoder,butterworth_a,butterworth_b,butterworth_x,butterworth_y);
//
//	        }
//	        else{
//	        	butterworth_x[0] = 0;
//	        	butterworth_x[1] = 0;
//	        	butterworth_x[2] = 0;
//	        	butterworth_y[0] = 0;
//	        	butterworth_y[1] = 0;
//	        	butterworth_y[2] = 0;
//	        }
		}
		step_motor_speed[0] = ((plain_speed / 65536) % 256);
		step_motor_speed[1] = ((plain_speed / 256) % 256);
		step_motor_speed[2] = ((plain_speed) % 256);
		break;
	}

    my_debugger(PID_delta_t,pendulum.filtered_mid_point,err,plain_speed,output);

    PID_delta_t = 0;

}

void pendulum_plain_algorithm(void){
	u32 plain_speed;
	s32 enc_error;

	enc_error = (s32)pendulum.mid_point - abs_encoder;

	if(enc_error < 0){
        step_motor_command = STEPPER_COMMAND_RUN_DOWN;
	}
	else{
        step_motor_command = STEPPER_COMMAND_RUN_UP;
	}

	if(abs_encoder < (pendulum.mid_point - pendulum.top_boundary)){
		plain_speed = 0;
	}
	else if(abs_encoder > (pendulum.mid_point + pendulum.top_boundary)){
		plain_speed = 0;
	}
	else{
		plain_speed = fabs(enc_error) * fabs(enc_error) * fabs(enc_error) * fabs(enc_error) * pendulum.speed_multiplier;
	}

	step_motor_speed[0] = ((plain_speed / 65536) % 256);
	step_motor_speed[1] = ((plain_speed / 256) % 256);
	step_motor_speed[2] = ((plain_speed) % 256);

}
void pendulum_head_shake(void){
	//5 msec loop
	static u32 local_timer = 0;

	step_motor_speed[0] = ((pendulum.headshake_speed / 65536) % 256);
	step_motor_speed[1] = ((pendulum.headshake_speed / 256) % 256);
	step_motor_speed[2] = ((pendulum.headshake_speed) % 256);

	if(local_timer > 0) local_timer = local_timer - 5;

	switch (pendulum.headshake_tmp) {
		case 0:
            step_motor_command = STEPPER_COMMAND_RUN_UP;
            local_timer = pendulum.head_change_timer;
            pendulum.headshake_tmp++;
			break;
		case 1:
			if(local_timer == 0){
	            step_motor_command = STEPPER_COMMAND_RUN_DOWN;
	            local_timer = pendulum.head_change_timer;
	            pendulum.headshake_tmp++;
			}
			break;
		case 2:
			if(local_timer == 0){
	            step_motor_command = STEPPER_COMMAND_RUN_UP;
	            local_timer = pendulum.head_change_timer;
	            pendulum.headshake_tmp = 1;
			}
			break;
		default:
			break;
	}
}
void step_response(void){
    static u8 step_tmp = 0;
    static float first_step_values[5] = {0};
    static float last_step_values[17] = {0};
    static float average_first_step = 0;
    static float average_last_step = 0;
    static int meta_count = 0;
    static float meta_val[400] = {0};
    static int time_val[400] = {0};

    float tmc_y1 = 0;
    float tmc_y2 = 0;
    int tmc_x1 = 0;
    int tmc_x2 = 0;
    float dead_time = 0;
    float delta_p = 0;
    float K_halt = 0;
    float T_halt = 0;
    u8 checker;

    if(step_response_first_in == 1){
        step_response_first_in = 0;
        step_timer = 0;
        autotuning_in_operation = 1;

        step_tmp = 0;
        average_first_step = 0;
        average_last_step = 0;
        meta_count = 0;
        for(u8 j = 0; j < 5; j++){
            first_step_values[j] = 0;
        }
        for(u8 j = 0; j < 17; j++){
            last_step_values[j] = 0;
        }
        for(u32 j = 0; j < 400; j++){
            meta_val[j] = 0;
            time_val[j] = 0;
        }
    }

    switch (step_tmp) {
    case 0:
        step_motor_command = STEPPER_COMMAND_RUN_UP;
        step_tmp++;
        break;
    case 1:
		step_motor_speed[0] = ((parameters.step_first_speed / 65536) % 256);
		step_motor_speed[1] = ((parameters.step_first_speed / 256) % 256);
		step_motor_speed[2] = ((parameters.step_first_speed) % 256);
        step_timer = 0;
        meta_count = 0;
        step_tmp++;
        break;
    case 2:
        for(u8 i = 0; i < 4 ; i++){
            first_step_values[i] = first_step_values[i+1];
        }
        first_step_values[4] = filtered_pace_rate;

        if(step_timer >= parameters.step_transition_time * 1000){
    		step_motor_speed[0] = ((parameters.step_second_speed / 65536) % 256);
    		step_motor_speed[1] = ((parameters.step_second_speed / 256) % 256);
    		step_motor_speed[2] = ((parameters.step_second_speed) % 256);

            step_tmp++;
            average_first_step = (first_step_values[0] + first_step_values[1] +
                    first_step_values[2] + first_step_values[3] + first_step_values[4]) / (float)5;

            time_val[meta_count] = step_timer;
            meta_val[meta_count++] = average_first_step;
        }
        break;
    case 3:
        average_last_step = filtered_pace_rate;
        for(u8 i = 0; i < 16 ; i++){
            last_step_values[i] = last_step_values[i+1];
            average_last_step += last_step_values[i];
        }
        last_step_values[16] = filtered_pace_rate;

        average_last_step = average_last_step / (float)17;

        if(meta_count < 399){
            time_val[meta_count] = step_timer;
            meta_val[meta_count++] = filtered_pace_rate;
        }
        checker = 0;
        for(u8 i = 0; i < 17 ; i++){
            if((fabs(average_last_step - last_step_values[i])) <= (((float)0.001) * average_last_step)){
                checker++;
            }
        }
        if(checker == 17){
            step_tmp++;
            //qDebug() << "meta_count : " << meta_count;
        }
        break;
    case 4:
    	tmc_y1 = ((float)0.632) * (meta_val[meta_count-1] - meta_val[0]) + meta_val[0]; //y1 detected
    	tmc_y2 = ((float)0.283) * (meta_val[meta_count-1] - meta_val[0]) + meta_val[0]; //y2 detected
    	tmc_x1 = 0;
    	tmc_x2 = 0;

        for (int i = 0; i < meta_count; i++){
            if(tmc_y1 >= meta_val[i]){
            	tmc_x1 = i; //x1 detected, the point for y1
            }
            if(tmc_y2 >= meta_val[i]){
            	tmc_x2 = i; //x2 detected, the point for y2
            }
        }
        dead_time = (((float)time_val[tmc_x1] - (float)time_val[0]) - ((float)1.5) * ((float)time_val[tmc_x1] - (float)time_val[tmc_x2]));
        delta_p = (float)parameters.step_second_speed - (float)parameters.step_first_speed;
        K_halt = (meta_val[meta_count-1] - meta_val[0]) / delta_p;
        T_halt = (float)time_val[tmc_x1] - (float)time_val[tmc_x2];

        calculated_kp = (T_halt / (K_halt * dead_time)) * (dead_time / ((float)4 * T_halt) + (float)4 / (float)3);
        calculated_ki = ((T_halt / (K_halt * dead_time)) * (dead_time / ((float)4*T_halt) + (float)4 / (float)3))
        		/ (dead_time * (((float)32 * T_halt + (float)6 * dead_time)/((float)13 * T_halt + (float)8 * dead_time)));
        calculated_kd = ((T_halt / (K_halt * dead_time)) * (dead_time/((float)4 * T_halt) + (float)4 / (float)3))
        		* (dead_time * ((float)4 * T_halt / ((float)2 * dead_time + (float)11 * T_halt)));
        step_tmp++;
        break;
    case 5:
		step_motor_command = STEPPER_COMMAND_STOP;
		autotuning_in_operation = 0;
		autotuning_is_finished = 1;
        step_tmp++;
    	break;
    case 6:
    	break;
    default:
        break;
    }
    //my_debugger(step_tmp,step_timer,average_last_step,meta_count,filtered_pace_rate);
    my_debugger(0,meta_count,calculated_kp,calculated_ki,calculated_kd);

}
void control_process(void){
	u32 PID_speed;

	switch (control_process_tmp) {
		case 0:
			PID_in_operation = 1;
			step_motor_speed[0] = ((parameters.test_start_speed / 65536) % 256);
			step_motor_speed[1] = ((parameters.test_start_speed / 256) % 256);
			step_motor_speed[2] = ((parameters.test_start_speed) % 256);
			control_process_tmp++;
			break;
		case 1:
			step_motor_command = STEPPER_COMMAND_RUN_UP;
			control_process_tmp++;
			break;
		case 2:
			if(filtered_load > parameters.zero_suppression){
				//step_motor_command = STEPPER_COMMAND_STOP;
				max_load_value = 0;
				PID_first_in = 1;
				PID_delta_t = 0;
				control_process_tmp++;
				plot_counter_1_msec = 0;
			}
			break;
		case 3:
            if(filtered_load > max_load_value){
                max_load_value = filtered_load;
            }
            if(filtered_load <= (max_load_value - (max_load_value * parameters.break_percentage)/100)){
				control_process_tmp++;
            }
            PID_speed = PID();
            //PID_speed = PID_with_fuzzy();
			step_motor_speed[0] = ((PID_speed / 65536) % 256);
			step_motor_speed[1] = ((PID_speed / 256) % 256);
			step_motor_speed[2] = ((PID_speed) % 256);
			break;
		case 4:
			step_motor_command = STEPPER_COMMAND_STOP;
			break;
		default:
			break;
	}

}
int main(void) {
	//float aux_float;

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	MX_TIM1_Init();		//	Encoder
	//MX_TIM8_Init(); 	// 	Duty Modilation
	//MX_TIM3_Init();   //  Frequency Modilation
	MX_TIM4_Init();
	MX_DMA_Init();
	MX_SPI2_Init();
	Max11254_Init();
	MX_USART1_UART_Init();
	MX_USART2_Init();
	MX_UART4_Init();
	//Timer3_AutoConsolidation_SpecialFunc(0);
//    bessel_filter_coeffs_for_raw();
//    bessel_filter_coeffs_for_pace();
    butterworth_lpf_coeffs(butterworth_a, butterworth_b);

	step_motor_command = STEPPER_COMMAND_STOP;
	step_motor_requested_pos = 0;
	stepper_abs_pos = 0;
	timer_1_msec = 0;
	timer_2_msec = 0;
	timer_10_msec = 0;
	timer_100_msec = 0;
	old_load = 0;
	_10_usec_counter = 0;
	send_RS485 = 0;
	usart_debugger_u8 = 0;
	usart_debugger_s32 = 0;
	usart_debugger_float[0] = 0;
	usart_debugger_float[1] = 0;
	usart_debugger_float[2] = 0;
	control_process_tmp = 0;
	TMC_command = TMC_STOP;
	step_timer = 0;
	step_response_first_in = 1;
	autotuning_in_operation = 0;
	autotuning_is_finished = 0;
	PID_in_operation = 0;
	pendulum.headshake_tmp = 0;
	pendulum.pid_tmp = 0;

	while (1) {
		usart_buffer_clearance();

		if(send_RS485 == 1){
			send_RS485 = 0;
			MASTER_send_RS485_data_to_motor();
			//read_inputs();
		}
		if (timer_1_msec == 1) {
			timer_1_msec = 0;
		}
		if (timer_2_msec == 1) {
			timer_2_msec = 0;
			encoder_value = Timer1_CalculateEncoderValue();
			abs_encoder = fabs(encoder_value % 4000);
			HAL_GPIO_TogglePin( Led_GPIO_Port, Led_Pin );

            if(TMC_command == TMC_PENDULUM_HEADSHAKE){
				pendulum_head_shake();
			}
			else if(TMC_command == TMC_PENDULUM_PLAIN_ALG){
				pendulum_plain_algorithm();
			}
			else if(TMC_command == TMC_PENDULUM_PID){
				pendulum_PID();
			}
			else if(TMC_command == TMC_STOP){
				pendulum.headshake_tmp = 0;
				pendulum.pid_tmp = 0;
			}
			send_RS485 = 1;
		}
		if (timer_10_msec == 1) {
			timer_10_msec = 0;
		}
		if (max1_dataready == 1) {
			max1_dataready = 0;


			//channel_operation(0);
//
//			filtered_load = SMA_load(cal[0].calibrated,16);
//			//filtered_load = cal[0].calibrated;
//
//			unfiltered_pace_rate = (filtered_load - old_load);
//			aux_float = (float)100000 / (float)_10_usec_counter;
//			unfiltered_pace_rate = unfiltered_pace_rate * aux_float;
//
//			old_load = filtered_load;
//			_10_usec_counter = 0;
//
//            //filtered_SMA = SMA_pace(unfiltered_pace_rate,16);
//            //filtered_pace_bessel = bessel_filter_for_pace(unfiltered_pace_rate);
//            //filtered_pace_butterworth = butterworth_filter(unfiltered_pace_rate,butterworth_a,butterworth_b,butterworth_x,butterworth_y);
//			//filtered_pace_rate = alpha_beta_filter(unfiltered_pace_rate);
//
//			filtered_pace_rate = butterworth_filter(unfiltered_pace_rate,butterworth_a,butterworth_b,butterworth_x,butterworth_y);


            //my_debugger(0,unfiltered_pace_rate,filtered_pace_bessel ,filtered_pace_butterworth, filtered_pace_rate);

		}
		if (max2_dataready == 1) {
			max2_dataready = 0;
			//channel_operation(1);
		}
		if (max3_dataready == 1) {
			max3_dataready = 0;
			//channel_operation(2);
		}
		if (max4_dataready == 1) {
			max4_dataready = 0;
			//channel_operation(3);
		}

		if (usart1_received == 1) {
			usart1_received = 0;
			USART1_receive_operations();
		}
		if (usart1_transmit == 1) {
			usart1_transmit = 0;
			HAL_UART_Transmit_DMA(&huart1, &usart1.tx[0], usart1.tx_amount);
		}
	}
}
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
void _Error_Handler(char * file, int line) {
	while (1) {
	}
}
