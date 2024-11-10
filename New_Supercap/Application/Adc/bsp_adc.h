#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f3xx_hal.h"
#include "adc_def.h"

/*
除了计算功率以外

CAP_I 用于控制电容充电放电电流环
CAP_V 用于检测电容充电放电电压

BAT_V 用于计算功率环
BAT_I 用于计算功率环

CHAS_I （基本无用）
CHAS_V 计算是否存在反电动势

*/

typedef struct adc_data
{

  uint16_t          adc_list[ADC_LIST_NUM];  /*ADC原始数据*/
  float             adc_cacl[ADC_LIST_NUM];  /*ADC矫正前数据*/
  ADC_Value         adc_value;               /*ADC矫正后数据*/
  ADC_Calibration_t adc_calibration;         /*ADC矫正参数*/

  void (*f_adc_update)(struct adc_data *self);
}ADC_Data_t;

extern ADC_Data_t ADC_Data;

static void ADC_Calibration(ADC_Data_t *self);
static void ADC_Cacl(ADC_Data_t *self);
static void ADC_power_cacl(ADC_Data_t *self);

void ADC_update(ADC_Data_t *self);

#endif /* __BSP_ADC_H */