/*
 * @Author: hwwx 93569047+hwwx@users.noreply.github.com
 * @Date: 2023-10-08 14:15:29
 * @LastEditors: hwwx 93569047+hwwx@users.noreply.github.com
 * @LastEditTime: 2023-10-22 21:33:59
 * @FilePath: \MDK-ARMc:\Users\HWX\Documents\GitHub\2023_HWX\New_Supercap\Application\Driver\driver.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file        driver.c
 * @author      RobotPilots@2023
 * @Version     V1.0
 * @date        9-September-2023
 * @brief       Drivers' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"
#include "hrtim.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "hrtim.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_adc.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void)
{
	PWM_Init();
	CAN1_Init();
}

void DEVICE_Init(void)
{
	//opamp
	HAL_OPAMP_Start(&hopamp2);

	//hrtim
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER); 
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); 
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2);

	//adc
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(&ADC_Data.adc_list[0]), 3);

    
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(&ADC_Data.adc_list[3]), 3);

	//dma中断
    // hdma_adc1.XferCpltCallback = hdma_adc1_cplt;
    // hdma_adc2.XferCpltCallback = hdma_adc2_cplt;

	//tim
    HAL_TIM_Base_Start_IT(&htim2);
}
