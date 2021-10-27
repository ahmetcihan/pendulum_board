#include "main.h"
#include "math.h"

s32 SMA_raw(s32 raw_signal,u8 filter_coefficient){
    static s32 running_average[64];
    float processed_value;
    u8 j;

    if(filter_coefficient > 63) filter_coefficient = 63;

    running_average[filter_coefficient-1] = raw_signal;
    processed_value = raw_signal;
    for (j = 0; j < (filter_coefficient-1); j++){
        processed_value += running_average[j];
            running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/((float)filter_coefficient);

    return (s32)processed_value;
}

s32 EMA_raw(s32 raw_signal, u8 filter_coefficient){
    float EMA = 0;
    static float past_EMA = 0;
    float alpha = (float)2/(filter_coefficient + (float)1);

    EMA = (float)raw_signal*alpha + past_EMA*((float)1-alpha);
    past_EMA = EMA;

    return (s32)EMA;
}
float EMA_load(float *raw_signal, u8 filter_coefficient){
    float EMA = 0;
    static float past_EMA = 0;
    float alpha = (float)2/(filter_coefficient+1);

    EMA = (*raw_signal)*alpha + past_EMA*(1-alpha);
    past_EMA = EMA;

    return EMA;
}
float SMA_load(float load_signal,u8 filter_coefficient){
    static float running_average[64];
    float processed_value;
    u8 j;

    if(filter_coefficient > 63) filter_coefficient = 63;

    running_average[filter_coefficient-1] = load_signal;
    processed_value = load_signal;
    for (j = 0; j < (filter_coefficient-1); j++){
        processed_value += running_average[j];
            running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/((float)filter_coefficient);

    return processed_value;
}
float SMA_pace(float pace_val,u8 filter_coefficient){
    static float running_average[64];
    float processed_value;
    u8 j;

    if(filter_coefficient > 63) filter_coefficient = 63;

    running_average[filter_coefficient-1] = pace_val;
    processed_value = pace_val;
    for (j = 0; j < (filter_coefficient-1); j++){
        processed_value += running_average[j];
            running_average[j] = running_average[j+1];
    }
    processed_value = (processed_value)/((float)filter_coefficient);

    return processed_value;
}

void bessel_filter_coeffs_for_raw(void){
    float samplerate = 400;
    float cutoff = 40;

    float QcRaw  = ((float)2.0 * (float)M_PI * cutoff) / ((float)1.0 * samplerate);
    float QcWarp = (float)1.0* tan(QcRaw); // Warp cutoff frequency

    float gain = (float)1.0 / ((float)1.0 + (float)M_SQRT2/QcWarp + (float)2.0/(QcWarp*QcWarp));

    bessel_raw_by[2] = ((float)1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
    bessel_raw_by[1] = ((float)2.0 - (float)4.0/(QcWarp*QcWarp)) * gain;
    bessel_raw_by[0] = (float)1.0;
    bessel_raw_ax[0] = (float)1.0 * gain;
    bessel_raw_ax[1] = (float)2.0 * gain;
    bessel_raw_ax[2] = (float)1.0 * gain;
}
float bessel_filter_for_raw(float input){
    static float xv[3] = {0};
    static float yv[3] = {0};

    xv[2] = xv[1];
    xv[1] = xv[0];
    xv[0] = input;
    yv[2] = yv[1];
    yv[1] = yv[0];
    yv[0] =   (bessel_raw_ax[0] * xv[0] + bessel_raw_ax[1] * xv[1] + bessel_raw_ax[2] * xv[2]
                - bessel_raw_by[1] * yv[0]
                - bessel_raw_by[2] * yv[1]);

    return yv[0];
}
void bessel_filter_coeffs_for_pace(void){
    float samplerate = 400;
    float cutoff = 10;

    float QcRaw  = ((float)2.0 * (float)M_PI * cutoff) / ((float)1.0 * samplerate);
    float QcWarp = (float)1.0* tan(QcRaw); // Warp cutoff frequency

    float gain = (float)1.0 / ((float)1.0 + (float)M_SQRT2/QcWarp + (float)2.0/(QcWarp*QcWarp));

    bessel_pace_by[2] = ((float)1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
    bessel_pace_by[1] = ((float)2.0 - (float)4.0/(QcWarp*QcWarp)) * gain;
    bessel_pace_by[0] = (float)1.0;
    bessel_pace_ax[0] = (float)1.0 * gain;
    bessel_pace_ax[1] = (float)2.0 * gain;
    bessel_pace_ax[2] = (float)1.0 * gain;
}
float bessel_filter_for_pace(float input){
    static float xv[3] = {0};
    static float yv[3] = {0};

    xv[2] = xv[1];
    xv[1] = xv[0];
    xv[0] = input;
    yv[2] = yv[1];
    yv[1] = yv[0];
    yv[0] =   (bessel_pace_ax[0] * xv[0] + bessel_pace_ax[1] * xv[1] + bessel_pace_ax[2] * xv[2]
                - bessel_pace_by[1] * yv[0]
                - bessel_pace_by[2] * yv[1]);

    return yv[0];
}

void butterworth_lpf_coeffs(float *a, float *b){
	float OmegaC = (float)0.01;
	float A = (float)1, B = M_SQRT2, C = (float)1, D = (float)0, E = (float)0, F = (float)1, T, Arg;

    T = (float)2.0 * tan(OmegaC * M_PI_2);

    Arg = ((float)4.0*A + (float)2.0*B*T + C*T*T);
    a[2] = ((float)4.0*A - (float)2.0*B*T + C*T*T) / Arg;
    a[1] = ((float)2.0*C*T*T - (float)8.0*A) / Arg;
    a[0] = (float)1.0;

    // With all pole filters, our LPF numerator is (z+1)^2, so all our Z Plane zeros are at -1
    b[2] = ((float)4.0*D - (float)2.0*E*T + F*T*T) / Arg * C/F;
    b[1] = ((float)2.0*F*T*T - (float)8.0*D) / Arg * C/F;
    b[0] = ((float)4*D + F*T*T + (float)2.0*E*T) / Arg * C/F;

}
float butterworth_filter(float input, float *a, float *b, float *x, float *y){
    float output, CenterTap;

    CenterTap = input * b[0] + b[1] * x[0] + b[2] * x[1];
    output = a[0] * CenterTap - a[1] * y[0] - a[2] * y[1];

    x[1] = x[0];
    x[0] = input;
    y[1] = y[0];
    y[0] = output;

    return output;
}

float alpha_beta_filter(float input){
	static float dt = 0.001;
	static float alpha = 0.05;
	static float beta = 0.005;
	static float xk = 0;
	static float vk = 0;
	static float rk = 0;
	static float xm = 0;
	static float xk_old = 0;
	static float vk_old = 0;

	xm = input;

	xk = xk_old + (vk_old * dt);
	vk = vk_old;

	rk = xm - xk;

	xk = xk + alpha * rk;
	vk = vk + (beta/dt)*rk;

	xk_old = xk;
	vk_old = vk;

	return xk;
}
