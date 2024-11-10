#ifndef __ADC_DEF_H
#define __ADC_DEF_H

#include "stm32f3xx_hal.h"


typedef enum
{
  ADC_BAT_V = 0,
  ADC_CAP_V,
  ADC_CHAS_I,
  ADC_BAT_I,
  ADC_CAP_I,
  ADC_CHAS_V,

  ADC_LIST_NUM,

}ADC_Channel_e;

typedef struct adc_value /*ADC*/
{
  float bat_v;
  float bat_i;
  float cap_v;
  float cap_i;
  float chas_v;
  float chas_i;

  float chas_power;
  float cap_power;
  float bat_power;
}ADC_Value;


typedef struct adc_calibration_k_b /**/
{
  float adc_k;
  float adc_b;
}ADC_k_b;

typedef struct adc_calibration /**/
{
  ADC_k_b bat_v;
  ADC_k_b bat_i;
  ADC_k_b cap_v;
  ADC_k_b cap_i;
  ADC_k_b chas_v;
  ADC_k_b chas_i;

}ADC_Calibration_t;








#endif /* __ADC_DEF_H */