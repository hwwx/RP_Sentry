#ifndef CAP_CONFIG_H
#define CAP_CONFIG_H

#include "stm32f3xx_hal.h"
#include "can_protocol.h"
#include "bsp_adc.h"
#include "dji_pid.h"
#include "judge_protocol.h"
#include "judge.h"
#include "driver.h"

#define CAP_1 

#define CURRENT_KP           1.0f
#define CURRENT_KI           1.0f
#define CURRENT_KD           1.0f
#define CURRENT_OUTPUTMAX    1.0f
#define CURRENT_INTERGMAX    1.0f

#define POWER_KP             1.0f
#define POWER_KI             1.0f
#define POWER_KD             1.0f
#define POWER_OUTPUTMAX      1.0f
#define POWER_INTERGMAX      1.0f

#define BUFFER_KP            1.0f
#define BUFFER_KI            1.0f
#define BUFFER_KD            1.0f
#define BUFFER_OUTPUTMAX     1.0f
#define BUFFER_INTERGMAX     1.0f


#define MAX_ERROR_CNT   (25)

#define MAX_CAP_VOLTAGE (25)
#define MAX_CAP_CURREMT (14)
#define MIN_CAP_VOLTAGE (3)
#define MIN_BAT_VOLTAGE (10)

#ifdef CAP_1
#define BAT_V_K  1.0f
#define BAT_V_B  0.0f
#define BAT_I_K  1.0f
#define BAT_I_B  0.0f
#define CAP_V_K  1.0f
#define CAP_V_B  0.0f
#define CAP_I_K  1.0f
#define CAP_I_B  0.0f
#define CHAS_V_K 1.0f
#define CHAS_V_B 0.0f
#define CHAS_I_K 1.0f
#define CHAS_I_B 0.0f
#elif defined CAP_2
#define BAT_V_K  1.0f
#define BAT_V_B  0.0f
#define BAT_I_K  1.0f
#define BAT_I_B  0.0f
#define CAP_V_K  1.0f
#define CAP_V_B  0.0f
#define CAP_I_K  1.0f
#define CAP_I_B  0.0f
#define CHAS_V_K 1.0f
#define CHAS_V_B 0.0f
#define CHAS_I_K 1.0f
#define CHAS_I_B 0.0f
#elif defined CAP_3
#define BAT_V_K  1.0f
#define BAT_V_B  0.0f
#define BAT_I_K  1.0f
#define BAT_I_B  0.0f
#define CAP_V_K  1.0f
#define CAP_V_B  0.0f
#define CAP_I_K  1.0f
#define CAP_I_B  0.0f
#define CHAS_V_K 1.0f
#define CHAS_V_B 0.0f
#define CHAS_I_K 1.0f
#define CHAS_I_B 0.0f
#elif defined CAP_4
#define BAT_V_K  1.0f
#define BAT_V_B  0.0f
#define BAT_I_K  1.0f
#define BAT_I_B  0.0f
#define CAP_V_K  1.0f
#define CAP_V_B  0.0f
#define CAP_I_K  1.0f
#define CAP_I_B  0.0f
#define CHAS_V_K 1.0f
#define CHAS_V_B 0.0f
#define CHAS_I_K 1.0f
#define CHAS_I_B 0.0f
#elif defined CAP_5
#define BAT_V_K  1.0f
#define BAT_V_B  0.0f
#define BAT_I_K  1.0f
#define BAT_I_B  0.0f
#define CAP_V_K  1.0f
#define CAP_V_B  0.0f
#define CAP_I_K  1.0f
#define CAP_I_B  0.0f
#define CHAS_V_K 1.0f
#define CHAS_V_B 0.0f
#define CHAS_I_K 1.0f
#define CHAS_I_B 0.0f
#elif defined CAP_6
#define BAT_V_K  1.0f
#define BAT_V_B  0.0f
#define BAT_I_K  1.0f
#define BAT_I_B  0.0f
#define CAP_V_K  1.0f
#define CAP_V_B  0.0f
#define CAP_I_K  1.0f
#define CAP_I_B  0.0f
#define CHAS_V_K 1.0f
#define CHAS_V_B 0.0f
#define CHAS_I_K 1.0f
#define CHAS_I_B 0.0f
#endif


// typedef enum
// {
//     BAT_OVER_VOLTAGE = 0,
//     BAT_UNDER_VOLTAGE,
//     BAT_OVER_CURRENT,
//     BAT_UNDER_CURRENT,
    
// }CAP_Error_e;


typedef struct cap_target_s
{
    float target_power;
    float target_current;
    float target_buffer;
}cap_target_t;

typedef struct cap_output_s
{
    int16_t buck_duty;
    int16_t boost_duty;
    int16_t OutDutyCycle;

}cap_output_t;

typedef struct cap_config_s
{
    ADC_Data_t*           adc;
    Cap_communication_t*  can;
    judge_t*              judge;

    pid_t* pid_current;
    pid_t* pid_power;
    pid_t* pid_buffer;

    cap_target_t target;
    cap_output_t output;

    struct
    {
        uint16_t warning : 1;             //报警
        uint16_t warning_cnt;

        uint16_t cap_v_over : 1;          //电容过压
        uint16_t cap_v_over_cnt;

        uint16_t cap_i_over : 1;          //电容过流
        uint16_t cap_i_over_cnt;

        uint16_t cap_v_low : 1;           //电容欠压
        uint16_t cap_v_low_cnt;

        uint16_t bat_v_low : 1;           //电池欠压
        uint16_t bat_v_low_cnt;

        uint16_t can_receive_miss : 1;    //未读到CAN通信数据
        uint16_t can_receive_miss_cnt;

    }ERROR;


    void (*PWM_output)(int16_t OutDutyCycle);
    
}CAP_t; 


void Cap_Init();
void Cap_Check();
void Cap_Work();
#endif /* CAP_CONFIG_H */
