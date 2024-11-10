#include "adc_def.h"
#include "bsp_adc.h"
#include "cap_config.h"

ADC_Data_t ADC_Data = 
{
  .adc_calibration.bat_v.adc_k  = BAT_V_K,
  .adc_calibration.bat_v.adc_b  = BAT_V_B,
  .adc_calibration.bat_i.adc_k  = BAT_I_K,
  .adc_calibration.bat_i.adc_b  = BAT_I_B,
  .adc_calibration.cap_v.adc_k  = CAP_V_K,
  .adc_calibration.cap_v.adc_b  = CAP_V_B,
  .adc_calibration.cap_i.adc_k  = CAP_I_K,
  .adc_calibration.cap_i.adc_b  = CAP_I_B,
  .adc_calibration.chas_v.adc_k = CHAS_V_K,
  .adc_calibration.chas_v.adc_b = CHAS_V_B,
  .adc_calibration.chas_i.adc_k = CHAS_I_K,
  .adc_calibration.chas_i.adc_b = CHAS_I_B,

  .f_adc_update = ADC_update,
};


/**
 * @brief ADC_Calibration
 * @note  采样值校准
 * @author HWX
 */
static void ADC_Calibration(ADC_Data_t *self)
{
  self->adc_value.bat_v  = self->adc_calibration.bat_v.adc_k  * self->adc_cacl[ADC_BAT_V]   + self->adc_calibration.bat_v.adc_b;
  self->adc_value.cap_v  = self->adc_calibration.cap_v.adc_k  * self->adc_cacl[ADC_CAP_V]   + self->adc_calibration.cap_v.adc_b;
  self->adc_value.chas_i = self->adc_calibration.chas_i.adc_k * self->adc_cacl[ADC_CHAS_I]  + self->adc_calibration.chas_i.adc_b;
  self->adc_value.bat_i  = self->adc_calibration.bat_i.adc_k  * self->adc_cacl[ADC_BAT_I]   + self->adc_calibration.bat_i.adc_b;
  self->adc_value.cap_i  = self->adc_calibration.cap_i.adc_k  * self->adc_cacl[ADC_CAP_I]   + self->adc_calibration.cap_i.adc_b;
  self->adc_value.chas_v = self->adc_calibration.chas_v.adc_k * self->adc_cacl[ADC_CHAS_V]  + self->adc_calibration.chas_v.adc_b;
}



/**
 * @brief ADC_Cacl
 * @note  处理DMA采样数据，计算电压电流值，存入adc_cacl数组中，供校准使用
 * @author HWX
 */
static void ADC_Cacl(ADC_Data_t *self)
{
    self->adc_cacl[ADC_BAT_V] = self->adc_list[ADC_BAT_V]/ 4096.0f * 3.3f * 15;
	self->adc_cacl[ADC_BAT_I] = (self->adc_list[ADC_BAT_I] / 4096.0f * 3.3f - 1.65f) / 0.075f ;

    self->adc_cacl[ADC_CAP_V] = self->adc_list[ADC_CAP_V]/ 4096.0f * 3.3f * 15;
    self->adc_cacl[ADC_CAP_I] = -(((float)self->adc_list[ADC_CAP_I] / 4096 * 3.3f - 1.65f) / 0.075f);

    self->adc_cacl[ADC_CHAS_I] = -(((float)self->adc_list[ADC_CHAS_I] / 4096 * 3.3f - 1.65f) / 0.062f);
    self->adc_cacl[ADC_CHAS_V] = self->adc_list[ADC_CHAS_V]/ 4096.0f * 3.3f * 15;
}
/**
 * @brief ADC_power_cacl
 * @note  计算功率
 * @author HWX
 */
static void ADC_power_cacl(ADC_Data_t *self)
{
  self->adc_value.bat_power  = self->adc_value.bat_v  * self->adc_value.bat_i;
  self->adc_value.cap_power  = self->adc_value.cap_v  * self->adc_value.cap_i;
  self->adc_value.chas_power = self->adc_value.chas_v * self->adc_value.chas_i;
}

/**
 * @brief ADC_update
 * @note  采样值更新，包括数值计算和校准，供外部调用，每次DMA采样完成后调用一次，
 *        更新一次采样值，供外部调用，每次DMA采样完成后调用一次，更新一次采样值
 * @author HWX
 */
void ADC_update(ADC_Data_t *self)
{
    /*数值计算*/
    ADC_Cacl(self);

    /*数值校准*/
    ADC_Calibration(self);

    /*功率计算*/
    ADC_power_cacl(self);

}
