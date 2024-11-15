/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/
/**
  ******************************************************************************
  * @file			pid.c
  * @version		V1.0.0
  * @date			2016年11月11日17:21:36
  * @brief   		对于PID， 反馈/测量习惯性叫get/measure/real/fdb,
						  期望输入一般叫set/target/ref
  *******************************************************************************/
  
  
  
/* Includes ------------------------------------------------------------------*/
#include "dji_pid.h"
#include "mytype.h"
#include <math.h>
#include "cmsis_os.h"

#define ABS(x)		((x>0)? (x): (-x)) 

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}



pid_t pid_current =
{
    .f_param_init = PID_struct_init,
    .f_pid_cacl = pid_calc,
    .f_pid_cacl_dead = pid_calc_dead,
};
pid_t pid_power = 
{
    .f_param_init = PID_struct_init,
    .f_pid_cacl = pid_calc,
    .f_pid_cacl_dead = pid_calc_dead,
};
pid_t pid_buffer = 
{
    .f_param_init = PID_struct_init,
    .f_pid_cacl = pid_calc,
    .f_pid_cacl_dead = pid_calc_dead,
};


/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;

    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
    {
		return 0;
    }
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
    {
		return 0;
    }

    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->pos_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->pos_out), pid->MaxOutput);
    pid->last_pos_out = pid->pos_out;	//update last time 
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    return pid->pos_out;
	
}

float pid_calc_dead(pid_t* pid, float get, float set,float dead_zone)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
	pid->err[NOW] = set - get;
	
    if(fabs(pid->err[NOW]) <= dead_zone)
    {
        pid->err[NOW] = 0;
    }
    
    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->pos_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->pos_out), pid->MaxOutput);
    pid->last_pos_out = pid->pos_out;
    
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    return  pid->pos_out;
}


/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
	
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}
