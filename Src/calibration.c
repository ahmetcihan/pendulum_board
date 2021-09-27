#include "usart.h"
#include "gpio.h"
#include "main.h"

void slope_calculation(uint8_t no){
    u8 validation_1 = 0xFF;
    u8 validation_2 = 0xFF;

    if(cal[no].point_no > 8) cal[no].point_no = 2;
    for(u8 i = 0; i < (cal[no].point_no - 1); i++){
        if(cal[no].real_val[i] < cal[no].real_val[i+1]){
            validation_1 = validation_1 & 0xFF;
        }
        else{
            validation_1 = validation_1 & 0x00;
        }
    }
    if(validation_1 == 0xFF){
        //"calibration is ascending";
    }
    else{
        for(u8 i = 0; i < (cal[no].point_no - 1); i++){
            if(cal[no].real_val[i] > cal[no].real_val[i+1]){
                validation_2 = validation_2 & 0xFF;
            }
            else{
                validation_2 = validation_2 & 0x00;
            }
        }
        if(validation_2 == 0xFF){
            //"calibration is descending";
        }
        else{
            //"calibration is faulty";
        }
    }

    for(u8 i = 0; i < (cal[no].point_no - 1); i++){
        cal[no].slope[i] = ((1.0*(double)(cal[no].assigned_val[i+1]-cal[no].assigned_val[i]))/
                (1.0*(double)(cal[no].real_val[i+1] - cal[no].real_val[i])));
    }
    cal[no].tare_val = 0;

}
double evaluate_calibrated_values(uint8_t no){
    double value = 0;
    double aux = 0;
    s32 tared = cal[no].signed_raw;

    switch (cal[no].point_no) {
    case 8:
        if (tared <= cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[0]);
            value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        }
        else if ((tared <= cal[no].real_val[2])){
            aux  = (tared - cal[no].real_val[1]);
            value  = cal[no].slope[1]*(aux) + cal[no].assigned_val[1];
        }
        else if ((tared <= cal[no].real_val[3])){
            aux  = (tared - cal[no].real_val[2]);
            value  = cal[no].slope[2]*(aux) + cal[no].assigned_val[2];
        }
        else if ((tared <= cal[no].real_val[4])){
            aux  = (tared - cal[no].real_val[3]);
            value  = cal[no].slope[3]*(aux) + cal[no].assigned_val[3];
        }
        else if ((tared <= cal[no].real_val[5])){
            aux  = (tared - cal[no].real_val[4]);
            value  = cal[no].slope[4]*(aux) + cal[no].assigned_val[4];
        }
        else if ((tared <= cal[no].real_val[6])){
            aux  = (tared - cal[no].real_val[5]);
            value  = cal[no].slope[5]*(aux) + cal[no].assigned_val[5];
        }
        else if ((tared > cal[no].real_val[6])){
            aux  = (tared - cal[no].real_val[6]);
            value  = cal[no].slope[6]*(aux) + cal[no].assigned_val[6];
        }
        break;
    case 7:
        if (tared <= cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[0]);
            value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        }
        else if ((tared <= cal[no].real_val[2])){
            aux  = (tared - cal[no].real_val[1]);
            value  = cal[no].slope[1]*(aux) + cal[no].assigned_val[1];
        }
        else if ((tared <= cal[no].real_val[3])){
            aux  = (tared - cal[no].real_val[2]);
            value  = cal[no].slope[2]*(aux) + cal[no].assigned_val[2];
        }
        else if ((tared <= cal[no].real_val[4])){
            aux  = (tared - cal[no].real_val[3]);
            value  = cal[no].slope[3]*(aux) + cal[no].assigned_val[3];
        }
        else if ((tared <= cal[no].real_val[5])){
            aux  = (tared - cal[no].real_val[4]);
            value  = cal[no].slope[4]*(aux) + cal[no].assigned_val[4];
        }
        else if (tared > cal[no].real_val[5]){
            aux  = (tared - cal[no].real_val[5]);
            value  = cal[no].slope[5]*(aux) + cal[no].assigned_val[5];
        }
        break;
    case 6:
        if (tared <= cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[0]);
            value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        }
        else if ((tared <= cal[no].real_val[2])){
            aux  = (tared - cal[no].real_val[1]);
            value  = cal[no].slope[1]*(aux) + cal[no].assigned_val[1];
        }
        else if ((tared <= cal[no].real_val[3])){
            aux  = (tared - cal[no].real_val[2]);
            value  = cal[no].slope[2]*(aux) + cal[no].assigned_val[2];
        }
        else if ((tared <= cal[no].real_val[4])){
            aux  = (tared - cal[no].real_val[3]);
            value  = cal[no].slope[3]*(aux) + cal[no].assigned_val[3];
        }
        else if (tared > cal[no].real_val[4]){
            aux  = (tared - cal[no].real_val[4]);
            value  = cal[no].slope[4]*(aux) + cal[no].assigned_val[4];
        }
        break;
    case 5:
        if (tared <= cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[0]);
            value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        }
        else if ((tared <= cal[no].real_val[2])){
            aux  = (tared - cal[no].real_val[1]);
            value  = cal[no].slope[1]*(aux) + cal[no].assigned_val[1];
        }
        else if ((tared <= cal[no].real_val[3])){
            aux  = (tared - cal[no].real_val[2]);
            value  = cal[no].slope[2]*(aux) + cal[no].assigned_val[2];
        }
        else if (tared > cal[no].real_val[3]){
            aux  = (tared - cal[no].real_val[3]);
            value  = cal[no].slope[3]*(aux) + cal[no].assigned_val[3];
        }
        break;
    case 4:
        if (tared <= cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[0]);
            value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        }
        else if ((tared <= cal[no].real_val[2])){
            aux  = (tared - cal[no].real_val[1]);
            value  = cal[no].slope[1]*(aux) + cal[no].assigned_val[1];
        }
        else if (tared > cal[no].real_val[2]){
            aux  = (tared - cal[no].real_val[2]);
            value  = cal[no].slope[2]*(aux) + cal[no].assigned_val[2];
        }
        break;
    case 3:
        if (tared <= cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[0]);
            value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        }
        else if (tared > cal[no].real_val[1]){
            aux  = (tared - cal[no].real_val[1]);
            value  = cal[no].slope[1]*(aux) + cal[no].assigned_val[1];
        }
        break;
    case 2:
        aux  = (tared - cal[no].real_val[0]);
        value  = cal[no].slope[0]*(aux) + cal[no].assigned_val[0];
        break;
    }

    cal[no].absolute_calibrated = value;
    return (cal[no].absolute_calibrated - cal[no].tare_val);
}

