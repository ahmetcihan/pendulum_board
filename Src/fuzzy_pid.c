#include "main.h"
#include "math.h"

float uf(float x,float a,float  b,float c){
    if(x <= a)
        return 0;
    else if((a < x) && (x <= b))
        return  (x - a) / (b - a);
    else if((b < x) && (x <= c))
        return (c - x) / (c - b);
    else if(x > c)
        return 0;

    return 0;
}
float cuf(float x,float a,float b,float c){
    float y,z;

    z = ((b - a) * x + a);
    y = (c - (c - b) * x);

    return ((y + z) / (float)2);
}
float ufl(float x,float a,float b){
    if(x <= a) return 1;
    else if((a < x) && (x <= b)) return ((b - x) / (b - a));
    else if(x > b) return 0;

    return 0;
}
float cufl(float x,float a,float b){
    return (b - (b - a) * x);
}
float ufr(float x,float a,float b){
    if(x <= a) return 0;
    if((a < x) && (x < b)) return ((x - a) / (b - a));
    if(x >= b) return 1;

    return 0;
}
float cufr(float x,float a,float b){
    return ((b - a) * x + a);
}
float fand(float a,float b){
    return ((a < b) ? a : b);
}
float forr(float a,float b){
    return ((a < b) ? b : a);
}
float fuzzy_control(void){
    int kp[7][7]={
        {PB,PB,PB,PB,PM,PS,ZO},
        {PM,PM,PS,PS,PS,ZO,ZO},
        {PM,PS,ZO,ZO,ZO,NS,NM},
        {NB,NM,ZO,ZO,ZO,PS,PS},
        {NM,NS,ZO,ZO,ZO,PM,PB},
        {ZO,ZO,PS,PM,PM,PM,PB},
        {ZO,PS,PB,PB,PB,PB,PB}};
    int ki[7][7]={
        {PB,PB,PB,PM,PM,PS,ZO},
        {PB,PB,PB,PM,PS,ZO,ZO},
        {PB,PM,PM,PS,ZO,NS,NM},
        {NM,NS,ZO,ZO,ZO,NS,NM},
        {NM,NS,ZO,PS,PM,PM,PB},
        {ZO,ZO,PS,PM,PM,PB,PB},
        {ZO,PS,PB,PB,PB,PB,PB}};
    int kd[7][7]={
        {PB,PM,PS,NB,NB,NB,NB},
        {PM,PS,ZO,ZO,NB,NS,ZO},
        {PB,PM,PS,PS,PS,PM,PB},
        {PB,PM,PS,PS,PS,PM,PB},
        {PB,PM,PS,PS,PS,PM,PB},
        {ZO,NS,NM,ZO,ZO,PS,PM},
        {NB,NB,NB,PS,PS,PM,PB}};

    float error = 0;
    float error_diff = 0;
    static float last_error = 0;
    static float total_error = 0;

    error = parameters.pace_rate - filtered_pace_rate;
    error_diff = error - last_error;
    last_error = error;
    total_error += error;

    float es[7],ecs[7];
    float fuzzyfication_e_factor = (parameters.pace_rate/(float)15);
    float fuzzyfication_ec_factor = (fuzzyfication_e_factor * (float)0.2);

    es[NB] = ufl(error,   (float)-3*fuzzyfication_e_factor,  (float)-1*fuzzyfication_e_factor);
    es[NM] = uf(error,    (float)-3*fuzzyfication_e_factor,  (float)-2*fuzzyfication_e_factor,   (float)0*fuzzyfication_e_factor);
    es[NS] = uf(error,    (float)-3*fuzzyfication_e_factor,  (float)-1*fuzzyfication_e_factor,   (float)1*fuzzyfication_e_factor);
    es[ZO] = uf(error,    (float)-2*fuzzyfication_e_factor,   (float)0*fuzzyfication_e_factor,   (float)2*fuzzyfication_e_factor);
    es[PS] = uf(error,    (float)-1*fuzzyfication_e_factor,   (float)1*fuzzyfication_e_factor,   (float)3*fuzzyfication_e_factor);
    es[PM] = uf(error,     (float)0*fuzzyfication_e_factor,   (float)2*fuzzyfication_e_factor,   (float)3*fuzzyfication_e_factor);
    es[PB] = ufr(error,    (float)1*fuzzyfication_e_factor,   (float)3*fuzzyfication_e_factor);

    ecs[NB] = ufl(error_diff,   (float)-3*fuzzyfication_ec_factor,  (float)-1*fuzzyfication_ec_factor);
    ecs[NM] = uf(error_diff,    (float)-3*fuzzyfication_ec_factor,  (float)-2*fuzzyfication_ec_factor,   (float)0*fuzzyfication_ec_factor);
    ecs[NS] = uf(error_diff,    (float)-3*fuzzyfication_ec_factor,  (float)-1*fuzzyfication_ec_factor,   (float)1*fuzzyfication_ec_factor);
    ecs[ZO] = uf(error_diff,    (float)-2*fuzzyfication_ec_factor,   (float)0*fuzzyfication_ec_factor,   (float)2*fuzzyfication_ec_factor);
    ecs[PS] = uf(error_diff,    (float)-1*fuzzyfication_ec_factor,   (float)1*fuzzyfication_ec_factor,   (float)3*fuzzyfication_ec_factor);
    ecs[PM] = uf(error_diff,     (float)0*fuzzyfication_ec_factor,   (float)2*fuzzyfication_ec_factor,   (float)3*fuzzyfication_ec_factor);
    ecs[PB] = ufr(error_diff,    (float)1*fuzzyfication_ec_factor,   (float)3*fuzzyfication_ec_factor);

    float form[7][7];
    int i,j;
    float w;

    for(i = 0; i < 7 ; i++){
        for(j = 0; j < 7; j++){
            w = fand(es[i], ecs[j]);
            form[i][j] = w;
        }
    }

    int a = 0,b = 0;

    for(i = 0; i < 7; i++){
        for(j = 0; j < 7; j++){
            if(form[a][b] < form[i][j]){
                a = i;
                b = j;
            }
        }
    }

    float lsd;
    int p,d,in;

    lsd = form[a][b];
    p  = kp[a][b];
    d  = kd[a][b];
    in = ki[a][b];

    float detkp = 0,detkd = 0,detki = 0;

    if(p == NB)
        detkp = cufl(lsd,-3,-1);
    else if(p == NM)
        detkp = cuf(lsd,-3,2,0);
    else if(p == NS)
        detkp = cuf(lsd,-3,1,1);
    else if(p == ZO)
        detkp = cuf(lsd,-2,0,2);
    else if(p == PS)
        detkp = cuf(lsd,-1,1,3);
    else if(p == PM)
        detkp = cuf(lsd,0,2,3);
    else if(p == PB)
        detkp = cufr(lsd,1,3);

    if(in == NB)
        detki = cufl(lsd,-3,-1);
    else if(in == NM)
        detki = cuf(lsd,-3,2,0);
    else if(in == NS)
        detki = cuf(lsd,-3,1,1);
    else if(in == ZO)
        detki = cuf(lsd,-2,0,2);
    else if(in == PS)
        detki = cuf(lsd,-1,1,3);
    else if(in == PM)
        detki = cuf(lsd,0,2,3);
    else if(in == PB)
        detki = cufr(lsd,1,3);

    if(d == NB)
        detkd = cufl(lsd,-3,-1);
    else if(d == NM)
        detkd = cuf(lsd,-3,2,0);
    else if(d == NS)
        detkd = cuf(lsd,-3,1,1);
    else if(d == ZO)
        detkd = cuf(lsd,-2,0,2);
    else if(d == PS)
        detkd = cuf(lsd,-1,1,3);
    else if(d == PM)
        detkd = cuf(lsd,0,2,3);
    else if(d == PB)
        detkd = cufr(lsd,1,3);

    float det_u;
//    static double u = parameters.test_start_speed; //ahmet, burada ilk parametre test start speed olsa iyi olur
    static float u = 0;

    float f = (float)1; //ahmet:what is this parameter?
    float kp_fuzzy = (float)84;
    float ki_fuzzy = (float)0.32;
    float kd_fuzzy = (float)3920;

    kp_fuzzy = f * kp_fuzzy;
    ki_fuzzy = f * ki_fuzzy;
    kd_fuzzy = f * kd_fuzzy;

    kp_fuzzy += ((float)3 * kp_fuzzy * detkp) / (float)10;    //%90 of Kp_fuzzy
    ki_fuzzy += ((float)3 * ki_fuzzy * detki) / (float)10;
    kd_fuzzy += ((float)3 * kd_fuzzy * detkd) / (float)10;

//    static double kp_fuzzy = 4.0;
//    static double ki_fuzzy = 0.01;
//    static double kd_fuzzy = 1.1;
//    kp_fuzzy += 0.5 * detkp;
//    ki_fuzzy += 0.01 * detki;
//    kd_fuzzy += 0.01 * detkd;
//    if(kp_fuzzy <= 0) kp_fuzzy = 0.001;
//    if(ki_fuzzy <= 0) ki_fuzzy = 0.0001;
//    if(kd_fuzzy <= 0) kd_fuzzy = 0.001;

    det_u = kp_fuzzy * error + ki_fuzzy * total_error + kd_fuzzy * error_diff;
    u = det_u + u;

    if(u > 200000) u = 200000;

    if(u >= 0){
        step_motor_command = STEPPER_COMMAND_RUN_UP;
    }
    else{
        step_motor_command = STEPPER_COMMAND_RUN_DOWN;
    }

    return fabs(u);

}
float PID_with_fuzzy(void){
    static float last_error[3] = {0};
    static float output = 0;
    //static u8 first_move = 0;

    float error = 0;
    float error_diff = 0;
    float ak,bk,ck;
    float Ts = (float)PID_delta_t;
    float detkp = 0,detkd = 0,detki = 0;

    static float kp_fuzzy,ki_fuzzy,kd_fuzzy;

    if(PID_first_in){
        PID_first_in = 0;
        //first_move = 7;
        output = parameters.test_start_speed;
        last_error[0] = 0;
        last_error[1] = 0;
        last_error[2] = 0;
        kp_fuzzy = (float)49;
        ki_fuzzy = (float)0.21;
        kd_fuzzy = (float)1915;
    }
    else{
        kp_fuzzy = (float)49;
        ki_fuzzy = (float)0.21;
        kd_fuzzy = (float)1915;

        error = parameters.pace_rate - filtered_pace_rate;

        last_error[2] = last_error[1];
        last_error[1] = last_error[0];
        last_error[0] = error;

        // eklemeler

        int kp[7][7]={
            {PB,PB,PB,PB,PM,PS,ZO},
            {PM,PM,PS,PS,PS,ZO,ZO},
            {PM,PS,ZO,ZO,ZO,NS,NM},
            {NB,NM,ZO,ZO,ZO,PS,PS},
            {NM,NS,ZO,ZO,ZO,PM,PB},
            {ZO,ZO,PS,PM,PM,PM,PB},
            {ZO,PS,PB,PB,PB,PB,PB}};
        int ki[7][7]={
            {PB,PB,PB,PM,PM,PS,ZO},
            {PB,PB,PB,PM,PS,ZO,ZO},
            {PB,PM,PM,PS,ZO,NS,NM},
            {NM,NS,ZO,ZO,ZO,NS,NM},
            {NM,NS,ZO,PS,PM,PM,PB},
            {ZO,ZO,PS,PM,PM,PB,PB},
            {ZO,PS,PB,PB,PB,PB,PB}};
        int kd[7][7]={
            {PB,PM,PS,NB,NB,NB,NB},
            {PM,PS,ZO,ZO,NB,NS,ZO},
            {PB,PM,PS,PS,PS,PM,PB},
            {PB,PM,PS,PS,PS,PM,PB},
            {PB,PM,PS,PS,PS,PM,PB},
            {ZO,NS,NM,ZO,ZO,PS,PM},
            {NB,NB,NB,PS,PS,PM,PB}};

        float es[7],ecs[7];
        //float fuzzyfication_e_factor = (parameters.pace_rate/(float)15);
        float fuzzyfication_e_factor = (parameters.pace_rate)/(float)5;
        float fuzzyfication_ec_factor = (fuzzyfication_e_factor * (float)0.2);

        es[NB] = ufl(error,   (float)-3*fuzzyfication_e_factor,  (float)-1*fuzzyfication_e_factor);
        es[NM] = uf(error,    (float)-3*fuzzyfication_e_factor,  (float)-2*fuzzyfication_e_factor,   (float)0*fuzzyfication_e_factor);
        es[NS] = uf(error,    (float)-3*fuzzyfication_e_factor,  (float)-1*fuzzyfication_e_factor,   (float)1*fuzzyfication_e_factor);
        es[ZO] = uf(error,    (float)-2*fuzzyfication_e_factor,   (float)0*fuzzyfication_e_factor,   (float)2*fuzzyfication_e_factor);
        es[PS] = uf(error,    (float)-1*fuzzyfication_e_factor,   (float)1*fuzzyfication_e_factor,   (float)3*fuzzyfication_e_factor);
        es[PM] = uf(error,     (float)0*fuzzyfication_e_factor,   (float)2*fuzzyfication_e_factor,   (float)3*fuzzyfication_e_factor);
        es[PB] = ufr(error,    (float)1*fuzzyfication_e_factor,   (float)3*fuzzyfication_e_factor);

        ecs[NB] = ufl(error_diff,   (float)-3*fuzzyfication_ec_factor,  (float)-1*fuzzyfication_ec_factor);
        ecs[NM] = uf(error_diff,    (float)-3*fuzzyfication_ec_factor,  (float)-2*fuzzyfication_ec_factor,   (float)0*fuzzyfication_ec_factor);
        ecs[NS] = uf(error_diff,    (float)-3*fuzzyfication_ec_factor,  (float)-1*fuzzyfication_ec_factor,   (float)1*fuzzyfication_ec_factor);
        ecs[ZO] = uf(error_diff,    (float)-2*fuzzyfication_ec_factor,   (float)0*fuzzyfication_ec_factor,   (float)2*fuzzyfication_ec_factor);
        ecs[PS] = uf(error_diff,    (float)-1*fuzzyfication_ec_factor,   (float)1*fuzzyfication_ec_factor,   (float)3*fuzzyfication_ec_factor);
        ecs[PM] = uf(error_diff,     (float)0*fuzzyfication_ec_factor,   (float)2*fuzzyfication_ec_factor,   (float)3*fuzzyfication_ec_factor);
        ecs[PB] = ufr(error_diff,    (float)1*fuzzyfication_ec_factor,   (float)3*fuzzyfication_ec_factor);

        float form[7][7];
        int i,j;
        float w;

        for(i = 0; i < 7 ; i++){
            for(j = 0; j < 7; j++){
                w = fand(es[i], ecs[j]);
                form[i][j] = w;
            }
        }

        int a = 0,b = 0;

        for(i = 0; i < 7; i++){
            for(j = 0; j < 7; j++){
                if(form[a][b] < form[i][j]){
                    a = i;
                    b = j;
                }
            }
        }

        float lsd;
        int p,d,in;

        lsd = form[a][b];
        p  = kp[a][b];
        d  = kd[a][b];
        in = ki[a][b];


        if(p == NB)
            detkp = cufl(lsd,-3,-1);
        else if(p == NM)
            detkp = cuf(lsd,-3,2,0);
        else if(p == NS)
            detkp = cuf(lsd,-3,1,1);
        else if(p == ZO)
            detkp = cuf(lsd,-2,0,2);
        else if(p == PS)
            detkp = cuf(lsd,-1,1,3);
        else if(p == PM)
            detkp = cuf(lsd,0,2,3);
        else if(p == PB)
            detkp = cufr(lsd,1,3);

        if(in == NB)
            detki = cufl(lsd,-3,-1);
        else if(in == NM)
            detki = cuf(lsd,-3,2,0);
        else if(in == NS)
            detki = cuf(lsd,-3,1,1);
        else if(in == ZO)
            detki = cuf(lsd,-2,0,2);
        else if(in == PS)
            detki = cuf(lsd,-1,1,3);
        else if(in == PM)
            detki = cuf(lsd,0,2,3);
        else if(in == PB)
            detki = cufr(lsd,1,3);

        if(d == NB)
            detkd = cufl(lsd,-3,-1);
        else if(d == NM)
            detkd = cuf(lsd,-3,2,0);
        else if(d == NS)
            detkd = cuf(lsd,-3,1,1);
        else if(d == ZO)
            detkd = cuf(lsd,-2,0,2);
        else if(d == PS)
            detkd = cuf(lsd,-1,1,3);
        else if(d == PM)
            detkd = cuf(lsd,0,2,3);
        else if(d == PB)
            detkd = cufr(lsd,1,3);

        float f = (float)1; //ahmet:what is this parameter?
        kp_fuzzy = f * kp_fuzzy;
		ki_fuzzy = f * ki_fuzzy;
		kd_fuzzy = f * kd_fuzzy;

		kp_fuzzy += ((float)3 * kp_fuzzy * detkp) / (float)100;    //%90 of Kp_fuzzy
		ki_fuzzy += ((float)3 * ki_fuzzy * detki) / (float)100;
		kd_fuzzy += ((float)3 * kd_fuzzy * detkd) / (float)100;
        //

        ak = kp_fuzzy + kd_fuzzy / (float)Ts;
        bk = -kp_fuzzy + (ki_fuzzy * (float)Ts) - ((float)2 * kd_fuzzy)/(float)Ts;
        ck = kd_fuzzy / Ts;

        output = output + (ak * last_error[0] + bk * last_error[1] + ck * last_error[2]);
        if(output > 200000) output = 200000;

        if(output >= 0){
            step_motor_command = STEPPER_COMMAND_RUN_UP;
        }
        else{
            step_motor_command = STEPPER_COMMAND_RUN_DOWN;
        }

    }
    //my_debugger(PID_delta_t,output,error,filtered_pace_rate,parameters.pace_rate);
    my_debugger(PID_delta_t,output,error,kp_fuzzy,detkp);

    PID_delta_t = 0;

    return fabs(output);
}
