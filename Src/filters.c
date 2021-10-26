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

void bessel_filter_coeffs(void){
    float samplerate = 400;
    float cutoff = 10;

    float QcRaw  = ((float)2.0 * (float)M_PI * cutoff) / ((float)1.0 * samplerate);
    float QcWarp = (float)1.0* tan(QcRaw); // Warp cutoff frequency

    float gain = (float)1.0 / ((float)1.0 + (float)M_SQRT2/QcWarp + (float)2.0/(QcWarp*QcWarp));

    by[2] = ((float)1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
    by[1] = ((float)2.0 - (float)4.0/(QcWarp*QcWarp)) * gain;
    by[0] = (float)1.0;
    ax[0] = (float)1.0 * gain;
    ax[1] = (float)2.0 * gain;
    ax[2] = (float)1.0 * gain;
}
float bessel_filter(float input){
    static float xv[3] = {0};
    static float yv[3] = {0};

    xv[2] = xv[1];
    xv[1] = xv[0];
    xv[0] = input;
    yv[2] = yv[1];
    yv[1] = yv[0];
    yv[0] =   (ax[0] * xv[0] + ax[1] * xv[1] + ax[2] * xv[2]
                - by[1] * yv[0]
                - by[2] * yv[1]);

    return yv[0];
}

