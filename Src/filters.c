#include "main.h"
#include "math.h"

s32 EMA_raw(s32 *raw_signal, u8 filter_coefficient){
    float EMA = 0;
    static float past_EMA = 0;
    float alpha = (float)2/(filter_coefficient+1);

    EMA = (*raw_signal)*alpha + past_EMA*(1-alpha);
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

void bessel_filter_coeffs(void){
    float samplerate = 400;
    float cutoff = 2;

    float QcRaw  = (2.0 * M_PI * cutoff) / (1.0*samplerate);
    float QcWarp = 1.0* tan(QcRaw); // Warp cutoff frequency

    float gain = 1.0 / (1.0 + M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp));

    by[2] = (1.0 - M_SQRT2/QcWarp + 2.0/(QcWarp*QcWarp)) * gain;
    by[1] = (2.0 - 4.0/(QcWarp*QcWarp)) * gain;
    by[0] = 1.0;
    ax[0] = 1.0 * gain;
    ax[1] = 2.0 * gain;
    ax[2] = 1.0 * gain;
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

