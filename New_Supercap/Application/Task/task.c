#include "cap_config.h"

CAP_t cap = 
{
    .adc            = &ADC_Data,
    .can            = &CAN_Data,
    .judge          = &judge,

    .pid_buffer     = &pid_buffer,
    .pid_current    = &pid_current,
    .pid_power      = &pid_power,

    .PWM_output     = PWM_output,
};


/**
 * @brief   Cap_Init
 * @note    超电初始化函数
 */
void Cap_Init()
{
    cap.pid_current->f_param_init(cap.pid_current,POSITION_PID,\
    CURRENT_OUTPUTMAX,CURRENT_INTERGMAX,CURRENT_KP,CURRENT_KI,CURRENT_KD);

    cap.pid_power->f_param_init(cap.pid_power,POSITION_PID,\
    POWER_OUTPUTMAX,POWER_INTERGMAX,POWER_KP,POWER_KI,POWER_KD);

    cap.pid_buffer->f_param_init(cap.pid_buffer,POSITION_PID,\
    BUFFER_OUTPUTMAX,BUFFER_INTERGMAX,BUFFER_KP,BUFFER_KI,BUFFER_KD);
}

/**
 * @brief   Cap_Check
 * @note    超电自检函数
 */
void Cap_Check()
{
    /*电容过压检测*/
    if(cap.adc->adc_value.cap_v > MAX_CAP_VOLTAGE)
    {
        cap.ERROR.cap_v_over_cnt++;
        if(cap.ERROR.cap_v_over_cnt > MAX_ERROR_CNT)
        {
            cap.ERROR.cap_v_over = true;
        }
    }
    else
    {
        cap.ERROR.cap_i_over_cnt = 0;
        cap.ERROR.cap_v_over = false;
    }

    /*电容过流检测*/
    if(cap.adc->adc_value.cap_i > MAX_CAP_CURREMT)
    {
        cap.ERROR.cap_i_over_cnt++;
        if(cap.ERROR.cap_i_over_cnt > MAX_ERROR_CNT)
        {
            cap.ERROR.cap_i_over = true;
        }
    }
    else
    {
        cap.ERROR.cap_i_over_cnt = 0;
        cap.ERROR.cap_i_over = false;
    }

    /*电容欠压检测*/
    if(cap.adc->adc_value.cap_v < MIN_CAP_VOLTAGE)
    {
        cap.ERROR.cap_v_low_cnt++;
        if(cap.ERROR.cap_v_low_cnt > MAX_ERROR_CNT)
        {
            cap.ERROR.cap_v_low = true;
        }
    }
    else
    {
        cap.ERROR.cap_v_low_cnt = 0;
        cap.ERROR.cap_v_low = false;
    }

    /*电池欠压检测*/
    if(cap.adc->adc_value.bat_v < MIN_BAT_VOLTAGE)
    {
        cap.ERROR.bat_v_low_cnt++;
        if(cap.ERROR.bat_v_low_cnt > MAX_ERROR_CNT)
        {
            cap.ERROR.bat_v_low = true;
        }
    }
    else
    {
        cap.ERROR.bat_v_low_cnt = 0;
        cap.ERROR.bat_v_low = false;
    }

    /*CAN掉线*/
    if(cap.can->E_state.status == DEV_OFFLINE || cap.can->E_state.status == DEV_OFFLINE)
    {
        cap.ERROR.can_receive_miss = true;
    }
    else
    {
        cap.ERROR.can_receive_miss = false;
    }

    /*错误变量赋给通讯值*/
    cap.can->transimit.cap_state.bit.bat_v_low          = cap.ERROR.bat_v_low;
    cap.can->transimit.cap_state.bit.cap_i_over         = cap.ERROR.cap_i_over;
    cap.can->transimit.cap_state.bit.cap_v_low          = cap.ERROR.cap_v_low;
    cap.can->transimit.cap_state.bit.cap_v_over         = cap.ERROR.cap_v_over;
    cap.can->transimit.cap_state.bit.can_receive_miss   = cap.ERROR.can_receive_miss;
}

/**
 * @brief   Cap_Work
 * @note    ADC采样更新以及数值计算函数
 */
void Cap_Work()
{
    /*ADC采样数据更新*/
    cap.adc->f_adc_update(cap.adc); 

    /*信息获取（中断中完成）*/

    /*buffer环计算*/
    cap.target.target_power = cap.pid_buffer->f_pid_cacl(cap.pid_buffer,\
                                                          cap.can->recive.chassis_power_buffer,\
                                                          cap.target.target_buffer);

    /*功率环计算*/
    cap.target.target_current = cap.pid_power->f_pid_cacl(cap.pid_power,\
                                                          cap.target.target_power,\
                                                          cap.adc->adc_value.bat_power);
                                                          
    /*电流环计算*/
    cap.output.OutDutyCycle   = cap.pid_current->f_pid_cacl(cap.pid_current,\
                                                            cap.adc->adc_value.cap_i,\
                                                            cap.target.target_current);

    /*最终修改pwm占空比*/
    cap.PWM_output(cap.output.OutDutyCycle);
}





