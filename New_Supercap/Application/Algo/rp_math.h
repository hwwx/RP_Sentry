#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "stm32f3xx_hal.h"


typedef struct SmoothAcceleration {
    int16_t current_speed;    // 当前速度
    int16_t target_speed;     // 目标速度
    int16_t max_acceleration; // 最大加速度
}Max_Acc_Fliter;



int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
float RampFloat(float final, float now, float ramp);
float DeathZoom(float input, float center, float death);
float Low_Pass_Fliter(float data , float last_data , float a);
int16_t SmoothAccelerationUpdate(struct SmoothAcceleration *smooth_acc); 
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
#endif

