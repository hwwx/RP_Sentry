/**
 * @file        rp_math.c
 * @author      RobotPilots@2023
 * @Version     V1.0
 * @date        11-September-2023
 * @brief       RP Algorithm.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"
#include <math.h> 
#include <stdlib.h> 
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;	
}

float DeathZoom(float input, float center, float death)
{
	if(fabs(input - center) < death)
		return center;
	return input;
}

float Low_Pass_Fliter(float data , float last_data , float a)
{
	return a*data + (1 - a) * last_data;
}

int16_t SmoothAccelerationUpdate(struct SmoothAcceleration *smooth_acc) 
{
    // 计算速度变化的增量
    int16_t speed_diff = smooth_acc->target_speed - smooth_acc->current_speed;

    // 限制速度变化的增量不超过给定的加速度
    float max_speed_change = smooth_acc->max_acceleration;
    if (abs(speed_diff) > max_speed_change) {
        speed_diff = (speed_diff > 0) ? max_speed_change : -max_speed_change;
    }

    // 更新当前速度
    smooth_acc->current_speed += speed_diff;

    return smooth_acc->current_speed;
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}
